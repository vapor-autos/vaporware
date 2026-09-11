from dataclasses import replace

import pytest

from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.lib.turbo_intent import IntentConfig, IntentHealth, TurboIntentManager


HEALTH = IntentHealth(session_id="session", link_fresh=True, lateral_active=True, vehicle_healthy=True,
                      operator_fresh=True, speed=3.0)


def manager(mode="execute"):
  result = TurboIntentManager(IntentConfig(mode=mode))
  result.update(HEALTH, None, 9.0)
  result.update(HEALTH, None, 10.0)
  return result


def request(m, **overrides):
  return {"protocolVersion": 1, "operatorId": "gcs", "requestId": 1, "sessionId": m.session_id,
          "epoch": m.epoch, "action": "request", "direction": "left", "baseFeedbackLogMonoTime": 10_000_000_000,
          "ready": True, "reverse": False, **overrides}


@pytest.mark.parametrize("direction,desire", [("left", log.Desire.laneChangeLeft), ("right", log.Desire.laneChangeRight)])
def test_one_press_one_evaluated_pulse(direction, desire):
  m = manager()
  r = request(m, direction=direction)
  m.update(HEALTH, r, 10.01)
  assert m.status == "awaitingEvaluation" and m.desire == desire
  assert m.pulse_frame == 0
  # Repeated packets/skipped inference leave the pulse pending, not acknowledged as executed.
  m.update(HEALTH, r, 10.05)
  m.evaluated(log.Desire.none, 0.0, 100, 10.05)
  assert m.status == "awaitingEvaluation"
  m.evaluated(desire, 0.6, 101, 10.1)
  assert m.status == "executing" and m.pulse_frame == 101
  assert m.desire == log.Desire.none
  assert m.lane_change_state == log.LaneChangeState.laneChangeStarting
  m.update(HEALTH, r, 10.2)
  m.evaluated(log.Desire.none, 0.0, 111, 10.7)
  assert m.status == "completed"
  m.update(HEALTH, {**r, "baseFeedbackLogMonoTime": 10_700_000_000}, 10.71)
  assert m.status == "completed" and m.desire == log.Desire.none


@pytest.mark.parametrize("mode,status", [("shadow", "shadow"), ("off", "rejected")])
def test_non_executing_modes_never_emit_desire(mode, status):
  m = manager(mode)
  m.update(HEALTH, request(m), 10.01)
  assert m.status == status
  assert m.desire == log.Desire.none
  assert m.pulse_time == 0


@pytest.mark.parametrize("changes,reason", [
  ({"link_fresh": False}, "link_stale"), ({"vehicle_healthy": False}, "vehicle_unhealthy"),
  ({"operator_fresh": False}, "operator_stale"), ({"operator_override": True}, "steering_override"),
  ({"reverse": True}, "reverse"), ({"speed": 0.0}, "speed_out_of_range"),
  ({"speed": 5.01}, "speed_out_of_range"), ({"speed": float("nan")}, "speed_out_of_range"),
])
def test_health_gates_reject_and_never_queue(changes, reason):
  m = manager()
  r = request(m)
  m.update(replace(HEALTH, **changes), r, 10.01)
  assert (m.status, m.reason) == ("rejected", reason)
  m.update(HEALTH, r, 10.02)
  assert m.status == "rejected" and m.desire == log.Desire.none


@pytest.mark.parametrize("field,value,reason", [
  ("baseFeedbackLogMonoTime", 9_500_000_000, "stale_context"),
  ("baseFeedbackLogMonoTime", 10_500_000_000, "stale_context"),
  ("baseFeedbackLogMonoTime", 0, "stale_context"),
  ("protocolVersion", 2, "unsupported_protocol"),
  ("direction", "turnLeft", "invalid_direction"),
  ("ready", False, "operator_not_ready"), ("reverse", True, "operator_not_ready"),
])
def test_bad_request_rejected_with_serializable_feedback(field, value, reason):
  m = manager()
  m.update(HEALTH, request(m, **{field: value}), 10.01)
  assert (m.status, m.reason) == ("rejected", reason)
  msg = messaging.new_message("turboIntentState", valid=True)
  msg.turboIntentState = m.snapshot(10.01)
  assert msg.turboIntentState.reason == reason


def test_short_speed_spikes_do_not_unlock():
  m = manager()
  m.update(replace(HEALTH, speed=0.0), None, 10.01)
  m.update(HEALTH, None, 10.1)
  m.update(HEALTH, request(m, baseFeedbackLogMonoTime=10_200_000_000), 10.2)
  assert (m.status, m.reason) == ("rejected", "motion_not_stable")


@pytest.mark.parametrize("cause", ["expiry", "override", "disconnect", "disengage", "cancel"])
def test_pending_pulse_can_be_invalidated_before_evaluation(cause):
  m = manager()
  r = request(m)
  m.update(HEALTH, r, 10.01)
  if cause == "expiry":
    m.update(HEALTH, r, 10.36)
  elif cause == "cancel":
    m.update(HEALTH, {**r, "action": "cancel"}, 10.1)
  else:
    field = {"override": "operator_override", "disconnect": "link_fresh", "disengage": "lateral_active"}[cause]
    m.update(replace(HEALTH, **{field: cause == "override"}), None, 10.1)
  assert m.status in ("expired", "interrupted", "canceled")
  assert m.desire == log.Desire.none


def test_cancel_overtaking_request_tombstones_it():
  m = manager()
  r = request(m)
  m.update(HEALTH, {**r, "action": "cancel"}, 10.01)
  m.update(HEALTH, r, 10.02)
  assert m.status == "idle" and m.desire == log.Desire.none


def test_new_request_during_execution_is_never_queued():
  m = manager()
  m.update(HEALTH, request(m), 10.01)
  m.evaluated(m.desire, 0.5, 1, 10.02)
  r2 = request(m, requestId=2, direction="right")
  m.update(HEALTH, r2, 10.1)
  assert m.request["requestId"] == 1
  m.evaluated(log.Desire.none, 0.0, 20, 11.0)
  m.update(HEALTH, {**r2, "baseFeedbackLogMonoTime": 11_000_000_000}, 11.01)
  assert m.status == "completed" and m.request["requestId"] == 1


def test_replays_and_out_of_order_ids_remain_rejected_after_many_requests():
  m = manager("shadow")
  for request_id in range(1, 300):
    m.update(HEALTH, request(m, requestId=request_id), 10.01)
  assert m.request["requestId"] == 299
  for request_id in (1, 200, 298, 299):
    m.update(HEALTH, request(m, requestId=request_id), 10.02)
    assert m.request["requestId"] == 299


def test_session_and_engagement_reset_reject_old_packets():
  m = manager()
  r = request(m)
  m.update(replace(HEALTH, lateral_active=False), r, 10.01)
  assert m.epoch != r["epoch"] and m.desire == log.Desire.none
  m.update(HEALTH, r, 10.02)
  assert m.status == "idle"
  m.update(replace(HEALTH, session_id="new-session"), r, 10.03)
  assert m.status == "idle"


def test_inflight_model_intent_is_not_claimed_aborted_on_link_loss_or_override():
  m = manager()
  r = request(m)
  m.update(HEALTH, r, 10.01)
  m.evaluated(m.desire, 0.8, 1, 10.02)
  for h in (replace(HEALTH, operator_override=True), replace(HEALTH, link_fresh=False),
            replace(HEALTH, session_id="reconnect")):
    m.update(h, None, 10.1)
    assert m.status == "executing" and m.desire == log.Desire.none
  m.update(replace(HEALTH, session_id="reconnect", lateral_active=False), None, 10.2)
  assert m.status == "interrupted"


@pytest.mark.parametrize("probability", [0.0, float("nan"), float("inf"), -0.1, 1.1])
def test_no_or_invalid_model_response_locks_out_new_requests(probability):
  m = manager()
  m.update(HEALTH, request(m), 10.01)
  m.evaluated(m.desire, probability, 1, 10.02)
  m.evaluated(log.Desire.none, probability, 41, 12.1)
  assert m.status == "noModelResponse" and m.busy
  assert not m.snapshot(12.1)["available"]


def test_execution_timeout_requires_takeover_not_another_request():
  m = manager()
  m.update(HEALTH, request(m), 10.01)
  m.evaluated(m.desire, 0.6, 1, 10.02)
  m.update(HEALTH, None, 20.1)
  assert (m.status, m.reason) == ("timedOut", "takeover_required")
  assert m.busy


@pytest.mark.parametrize("kwargs", [{"mode": "auto"}, {"min_speed": 0}, {"min_speed": 6}, {"max_speed": float("nan")}])
def test_invalid_configuration_fails_closed(kwargs):
  with pytest.raises(ValueError):
    IntentConfig(**kwargs)
