from dataclasses import replace

import pytest

from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.lib.turbo_intent import PROTOCOL_VERSION, IntentConfig, IntentHealth, TurboIntentManager, MANEUVER_DESIRES


HEALTH = IntentHealth(session_id="session", link_fresh=True, lateral_active=True, vehicle_healthy=True,
                      operator_fresh=True, speed=3.0)


def tick(m, now, probability=0.0, opposite=0.0, health=HEALTH, *, valid=True):
  m.update(health, None, now)
  m.evaluated(m.desire, probability, (m.last_frame or 0) + 1, now,
              opposite_probability=opposite,
              lane_change_probability=probability + opposite if m.request.get("maneuver") != "turn" else 0.0,
              model_valid=valid)


def run_until(m, end, probability=0.0, opposite=0.0, health=HEALTH):
  while m.last_evaluation is None or m.last_evaluation + 0.05 <= end + 1e-8:
    tick(m, round((m.last_evaluation or 8.45) + 0.05, 8), probability, opposite, health)


def manager(mode="execute", health=HEALTH, **config):
  m = TurboIntentManager(IntentConfig(mode=mode, **config))
  run_until(m, 10.0, health=health)
  return m


def request(m, now=10.01, **overrides):
  return {"protocolVersion": PROTOCOL_VERSION, "maneuver": "laneChange", "operatorId": "gcs", "requestId": 1,
          "sessionId": m.session_id, "epoch": m.epoch, "action": "request", "direction": "left",
          "baseFeedbackLogMonoTime": int(now * 1e9), "ready": True, "reverse": False, **overrides}


def start(m, maneuver="laneChange", direction="left", probability=0.9):
  r = request(m, maneuver=maneuver, direction=direction)
  m.update(HEALTH, r, 10.01)
  assert m.status == "awaitingEvaluation"
  desire = m.desire
  assert desire == MANEUVER_DESIRES[maneuver, direction]
  tick(m, 10.05, probability)
  assert m.status == "executing" and m.desire == log.Desire.none
  assert m.consumed_desire == desire
  return r


@pytest.mark.parametrize("maneuver,direction", MANEUVER_DESIRES)
def test_one_press_one_consumed_pulse_sustained_response_and_clear(maneuver, direction):
  m = manager()
  r = start(m, maneuver, direction)
  assert not m.responded
  run_until(m, 10.35, 0.9)
  assert m.responded
  run_until(m, 11.15, 0.25 if maneuver == "turn" else 0.001, 0.30 if maneuver == "turn" else 0.001)
  assert m.status == "responseCleared" and not m.busy
  assert m.snapshot(11.15)["phase"] == "cooldown"
  assert not m.snapshot(11.15)["available"]
  m.update(HEALTH, {**r, "baseFeedbackLogMonoTime": 11_150_000_000}, 11.15)
  assert m.status == "responseCleared" and m.desire == log.Desire.none
  run_until(m, 16.1)
  assert m.snapshot(16.1)["available"]
  assert m.history_remaining == 0


@pytest.mark.parametrize("mode,status", [("shadow", "shadow"), ("off", "rejected")])
def test_non_executing_modes_never_emit_desire(mode, status):
  m = manager(mode)
  m.update(HEALTH, request(m), 10.01)
  assert m.status == status and m.desire == log.Desire.none and m.pulse_time == 0


@pytest.mark.parametrize("changes,reason", [
  ({"link_fresh": False}, "link_stale"), ({"vehicle_healthy": False}, "vehicle_unhealthy"),
  ({"operator_fresh": False}, "operator_stale"), ({"operator_override": True}, "steering_override"),
  ({"reverse": True}, "reverse"), ({"speed": 0.0}, "speed_out_of_range"),
  ({"speed": 5.01}, "speed_out_of_range"), ({"speed": float("nan")}, "speed_out_of_range"),
  ({"standstill": True}, "standstill"), ({"lateral_active": False}, "not_engaged"),
])
def test_health_gates_reject_and_never_queue(changes, reason):
  m = manager()
  h = replace(HEALTH, **changes)
  m.update(h, None, 10.01)
  r = request(m)
  m.update(h, r, 10.01)
  assert (m.status, m.reason) == ("rejected", reason)
  m.update(HEALTH, r, 10.02)
  assert m.desire == log.Desire.none


@pytest.mark.parametrize("field,value,reason", [
  ("baseFeedbackLogMonoTime", 9_500_000_000, "stale_context"),
  ("baseFeedbackLogMonoTime", 10_500_000_000, "stale_context"),
  ("baseFeedbackLogMonoTime", 0, "stale_context"),
  ("protocolVersion", 2, "unsupported_protocol"),
  ("maneuver", "none", "invalid_maneuver"), ("maneuver", "bogus", "invalid_maneuver"),
  ("direction", "turnLeft", "invalid_direction"), ("ready", False, "operator_not_ready"),
  ("reverse", True, "operator_not_ready"),
])
def test_bad_request_rejected_with_serializable_feedback(field, value, reason):
  m = manager()
  m.update(HEALTH, request(m, **{field: value}), 10.01)
  assert (m.status, m.reason) == ("rejected", reason)
  msg = messaging.new_message("turboIntentState", valid=True)
  msg.turboIntentState = m.snapshot(10.01)
  assert msg.turboIntentState.reason == reason


@pytest.mark.parametrize("cause", ["expiry", "override", "disconnect", "disengage", "cancel"])
def test_pending_pulse_can_be_invalidated_before_evaluation(cause):
  m = manager()
  r = request(m)
  m.update(HEALTH, r, 10.01)
  if cause == "expiry":
    m.update(HEALTH, None, 10.4)
  elif cause == "cancel":
    m.update(HEALTH, {**r, "action": "cancel"}, 10.1)
  else:
    field = {"override": "operator_override", "disconnect": "link_fresh", "disengage": "lateral_active"}[cause]
    m.update(replace(HEALTH, **{field: cause == "override"}), None, 10.1)
  assert m.status in ("expired", "interrupted", "canceled")
  assert m.desire == log.Desire.none and m.pulse_time == 0


def test_cancel_overtakes_request_and_is_acknowledged_without_execution():
  m = manager()
  r = request(m)
  m.update(HEALTH, {**r, "action": "cancel"}, 10.01)
  assert m.receipt["result"] == "canceled_before_consumption"
  m.update(HEALTH, r, 10.02)
  assert m.status == "idle" and m.desire == log.Desire.none
  assert m.receipt["result"] == "retired"


def test_busy_rejection_has_own_receipt_does_not_replace_or_queue_active():
  m = manager()
  start(m)
  r2 = request(m, requestId=2, direction="right")
  m.update(HEALTH, r2, 10.1)
  assert m.request["requestId"] == 1 and m.receipt["requestId"] == 2
  assert m.receipt["result"] == "rejected_busy"
  run_until(m, 16.1)
  m.update(HEALTH, {**r2, "baseFeedbackLogMonoTime": 16_100_000_000}, 16.1)
  assert m.request["requestId"] == 1 and m.desire == log.Desire.none
  assert m.receipt["result"] == "retired"


def test_cancel_after_consumption_never_claims_abort_or_extends_cooldown():
  m = manager()
  r = start(m)
  deadline = m.cooldown_until
  m.update(HEALTH, {**r, "action": "cancel"}, 10.1)
  assert m.receipt["result"] == "already_consumed"
  assert m.receipt["pulseMonoTime"] > 0
  assert m.status == "executing" and m.cooldown_until == deadline


def test_high_water_marks_reject_old_ids_after_many_busy_requests():
  m = manager()
  start(m)
  for request_id in range(2, 300):
    m.update(HEALTH, request(m, requestId=request_id), 10.1)
  assert m.request["requestId"] == 1 and m.seen["gcs"] == 299
  run_until(m, 16.1)
  for request_id in (1, 200, 298, 299):
    m.update(HEALTH, request(m, now=16.1, requestId=request_id), 16.1)
    assert m.desire == log.Desire.none


@pytest.mark.parametrize("maneuver", ["turn", "laneChange"])
def test_absent_or_opposite_only_response_times_out_without_permanent_busy(maneuver):
  for p, opposite in [(0, 0), (0, .9)]:
    m = manager()
    start(m, maneuver, probability=p)
    run_until(m, 12.1, p, opposite)
    assert m.status == "unconfirmed" and not m.busy
    assert not m.snapshot(12.1)["available"]
    run_until(m, 16.1)
    assert m.snapshot(16.1)["available"]


@pytest.mark.parametrize("maneuver,deadline", [("turn", 12), ("laneChange", 10)])
def test_constant_high_response_has_bounded_tracking_and_recovery(maneuver, deadline):
  m = manager()
  start(m, maneuver)
  run_until(m, 10.05 + deadline, .9)
  assert (m.status, m.reason) == ("expired", "observation_timeout") and not m.busy
  run_until(m, 11.1 + deadline, .9)
  assert m.snapshot(11.1 + deadline)["available"]
  # The continuing high score is still visible, not frozen at the terminal frame.
  tick(m, 11.15 + deadline, .8)
  assert m.snapshot(11.15 + deadline)["modelResponseProbability"] == .8


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -.1, 1.1])
def test_invalid_scores_fault_and_recover_only_with_fresh_valid_history(bad):
  m = manager()
  start(m)
  tick(m, 10.1, bad)
  assert m.status == "faulted" and not m.busy
  assert m.snapshot(10.1)["faultReason"] == "invalid_model_output"
  assert not m.snapshot(10.1)["modelSampleValid"]
  run_until(m, 16.1)
  assert m.snapshot(16.1)["available"]


def test_stopping_ends_tracking_but_never_allows_parked_request():
  m = manager()
  start(m, "turn")
  run_until(m, 10.4, .9)
  stopped = replace(HEALTH, speed=0, standstill=True)
  run_until(m, 12.55, .9, health=stopped)
  assert m.status == "stopped"
  run_until(m, 17, .9, health=stopped)
  assert m.snapshot(17)["availabilityReason"] == "standstill"
  run_until(m, 17.7)
  assert m.snapshot(17.7)["available"]


def test_override_does_not_mask_or_pause_intent_and_blocks_new_requests():
  m = manager()
  start(m, "turn")
  overriding = replace(HEALTH, operator_override=True)
  run_until(m, 10.5, .9, health=overriding)
  assert m.status == "executing"
  run_until(m, 22.1, .9, health=overriding)
  assert m.status == "expired" and m.snapshot(22.1)["availabilityReason"] == "steering_override"
  run_until(m, 23.2)
  assert m.snapshot(23.2)["available"]


def test_consumed_history_survives_disengage_and_session_change():
  m = manager()
  r = start(m)
  m.update(replace(HEALTH, lateral_active=False), None, 10.1)
  assert m.status == "interrupted" and m.history_remaining == 100
  old_epoch = m.epoch
  m.update(replace(HEALTH, session_id="new"), r, 10.2)
  assert m.epoch != old_epoch and m.desire == log.Desire.none and m.history_remaining == 100
  # Elapsed wall time alone never drains the explicit model input history.
  m.update(replace(HEALTH, session_id="new"), None, 30)
  assert not m.snapshot(30)["available"] and m.history_remaining == 100


def test_no_model_or_stale_model_never_allows_request():
  m = TurboIntentManager(IntentConfig(mode="execute"))
  m.update(HEALTH, None, 9)
  m.update(HEALTH, request(m), 10.01)
  assert m.reason == "model_unavailable"
  m = manager()
  m.update(HEALTH, request(m, now=11), 11)
  assert m.reason == "model_stale"


def test_gap_and_duplicate_evaluations_cannot_accumulate_dwell_or_history():
  m = manager()
  start(m, "turn")
  tick(m, 10.1, .9)
  tick(m, 10.3, .9)
  assert not m.responded and m.response_since == 10.3
  remaining = m.history_remaining
  m.evaluated(log.Desire.none, .9, m.last_frame, 10.35)
  assert m.history_remaining == remaining and m.status == "faulted"
  assert m.snapshot(10.35)["faultReason"] == "invalid_model_evaluation"


def test_clear_requires_half_second_of_consecutive_valid_evaluations():
  m = manager()
  start(m)
  run_until(m, 10.4, .9)
  run_until(m, 10.85, .001)
  tick(m, 10.9, .03)
  run_until(m, 11.4, .001)
  assert m.status == "executing"
  tick(m, 11.45, .001)
  assert m.status == "responseCleared"


def test_consumed_pulse_is_retained_even_with_duplicate_model_frame():
  m = manager()
  m.update(HEALTH, request(m), 10.01)
  m.evaluated(m.desire, .9, m.last_frame, 10.05)
  assert m.status == "faulted" and m.pulse_time == 10.05 and m.history_remaining == 100
  assert m.desire == log.Desire.none


def test_camera_frame_reset_does_not_permanently_latch_model_fault():
  m = manager()
  start(m)
  m.evaluated(log.Desire.none, .9, 0, 10.1)
  assert m.status == "faulted" and m.history_remaining == 100
  run_until(m, 16.2)
  assert m.history_remaining == 0 and m.snapshot(16.2)["available"]


@pytest.mark.parametrize("speed", [.01, .5, 2, 5, 10, 100])
@pytest.mark.parametrize("maneuver", ["turn", "laneChange"])
def test_uncapped_mode_still_accepts_valid_forward_motion(speed, maneuver):
  h = replace(HEALTH, speed=speed)
  m = manager(health=h, min_speed=0, max_speed=0)
  m.update(h, request(m, maneuver=maneuver), 10.01)
  assert m.status == "awaitingEvaluation"
  assert m.snapshot(10.01)["minSpeed"] == m.snapshot(10.01)["maxSpeed"] == 0


@pytest.mark.parametrize("kwargs", [
  {"mode": "auto"}, {"min_speed": -1}, {"min_speed": 6}, {"max_speed": -1},
  {"max_speed": float("nan")}, {"min_speed": float("inf")}, {"max_speed": float("inf")},
  {"motion_duration_s": 0}, {"execution_timeout_s": 0}, {"response_timeout_s": float("nan")},
  {"cooldown_s": 0}, {"pulse_interval_s": 0}, {"recovery_s": 0},
])
def test_invalid_configuration_fails_closed(kwargs):
  with pytest.raises(ValueError):
    IntentConfig(**kwargs)


def test_default_speed_gates_and_shadow_mode_unchanged():
  config = IntentConfig()
  assert config.mode == "shadow"
  assert config.speed_allowed(2) and config.speed_allowed(5)
  assert not config.speed_allowed(.5) and not config.speed_allowed(5.01)
