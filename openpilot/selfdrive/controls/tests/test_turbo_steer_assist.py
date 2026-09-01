from dataclasses import FrozenInstanceError

import pytest

from openpilot.selfdrive.controls.lib.turbo_steer_assist import (
  TurboSteerAssistSource,
  compute_nudge_angle_deg,
  g29_steering_to_angle_deg,
  steering_angle_to_g29_target,
)


class FakeTurboSteerAssist:
  def __init__(
    self,
    active: bool = True,
    requested_angle_deg: float = 7.0,
    base_model_angle_deg: float = 5.0,
    sequence: int = 1,
    base_model_log_mono_time: int = 10_000_000_000,
  ):
    self.active = active
    self.requestedSteeringAngleDeg = requested_angle_deg
    self.baseModelSteeringAngleDeg = base_model_angle_deg
    self.sequence = sequence
    self.baseModelLogMonoTime = base_model_log_mono_time


class FakeSubMaster:
  def __init__(
    self,
    seen=True,
    valid=True,
    recv_time=10.0,
    active=True,
    requested_angle_deg=7.0,
    base_model_angle_deg=5.0,
    sequence=1,
    base_model_log_mono_time=10_000_000_000,
  ):
    self.seen = {"turboSteerAssist": seen}
    self.valid = {"turboSteerAssist": valid}
    self.recv_time = {"turboSteerAssist": recv_time}
    self.data = {
      "turboSteerAssist": FakeTurboSteerAssist(
        active,
        requested_angle_deg,
        base_model_angle_deg,
        sequence,
        base_model_log_mono_time,
      )
    }

  def __getitem__(self, service):
    return self.data[service]


def test_steering_angle_to_g29_target_uses_teleop_sign():
  assert steering_angle_to_g29_target(90.0) == pytest.approx(-0.5)
  assert steering_angle_to_g29_target(-90.0) == pytest.approx(0.5)


def test_g29_steering_to_angle_deg_uses_teleop_sign():
  assert g29_steering_to_angle_deg(-0.5) == pytest.approx(90.0)
  assert g29_steering_to_angle_deg(0.5) == pytest.approx(-90.0)


def test_compute_nudge_angle_deg_uses_soft_deadband():
  assert compute_nudge_angle_deg(wheel_steering=-94.0 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(0.0)
  assert compute_nudge_angle_deg(wheel_steering=-95.0 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(0.0)
  assert compute_nudge_angle_deg(wheel_steering=-96.0 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(0.624)
  assert compute_nudge_angle_deg(wheel_steering=-97.5 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(3.75)
  assert compute_nudge_angle_deg(wheel_steering=-100.0 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(10.0)
  assert compute_nudge_angle_deg(wheel_steering=-80.0 / 180.0, target_steering_angle_deg=90.0) == pytest.approx(-10.0)


def test_turbo_steer_assist_source_uses_fresh_active_target():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert decision.target_angle_deg == pytest.approx(7.0)
  assert decision.status == "active"
  assert decision.receive_age_s == pytest.approx(0.1)
  assert decision.context_age_s == pytest.approx(0.1)
  assert decision.base_model_delta_deg == pytest.approx(0.0)
  assert decision.sequence == 1
  assert decision.base_model_log_mono_time == 10_000_000_000
  with pytest.raises(FrozenInstanceError):
    decision.status = "stale"

  next_decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.11)
  assert next_decision.target_angle_deg == pytest.approx(7.0)
  assert next_decision.status == "active"


def test_turbo_steer_assist_source_accepts_zero_degree_target():
  source = TurboSteerAssistSource(FakeSubMaster(requested_angle_deg=0.0))

  decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)
  assert decision.target_angle_deg == pytest.approx(0.0)
  assert decision.status == "active"


def test_turbo_steer_assist_source_clips_target_to_steering_range():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=200.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert decision.target_angle_deg == pytest.approx(180.0)
  assert decision.status == "active"


def test_turbo_steer_assist_source_returns_absolute_blended_target():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=5.0, base_model_angle_deg=17.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  decision = source.update(lat_active=True, model_angle_deg=20.0, now=10.1)

  assert decision.status == "active"
  assert decision.target_angle_deg == pytest.approx(5.0)


def test_turbo_steer_assist_source_returns_none_when_inactive_or_stale():
  inactive = TurboSteerAssistSource(FakeSubMaster(active=False, requested_angle_deg=7.0))
  inactive_decision = inactive.update(lat_active=True, model_angle_deg=5.0, now=10.1)
  assert inactive_decision.target_angle_deg is None
  assert inactive_decision.status == "inactive"

  stale = TurboSteerAssistSource(FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0), stale_timeout_s=0.25)
  stale_decision = stale.update(lat_active=True, model_angle_deg=5.0, now=10.5)
  assert stale_decision.target_angle_deg is None
  assert stale_decision.status == "stale"

  lat_inactive = TurboSteerAssistSource(FakeSubMaster(requested_angle_deg=7.0))
  lat_inactive_decision = lat_inactive.update(lat_active=False, model_angle_deg=5.0, now=10.1)
  assert lat_inactive_decision.target_angle_deg is None
  assert lat_inactive_decision.status == "lat_inactive"


def test_turbo_steer_assist_source_rejects_stale_target_context():
  sm = FakeSubMaster(recv_time=10.4, requested_angle_deg=7.0, base_model_log_mono_time=10_000_000_000)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25, context_timeout_s=0.35)

  decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.4)
  assert decision.target_angle_deg is None
  assert decision.status == "stale_target_context"
  assert decision.context_age_s == pytest.approx(0.4)


def test_turbo_steer_assist_source_rejects_target_mismatch():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)

  decision = source.update(lat_active=True, model_angle_deg=21.0, now=10.1)
  assert decision.target_angle_deg is None
  assert decision.status == "target_mismatch"
  assert decision.base_model_delta_deg == pytest.approx(16.0)


def test_turbo_steer_assist_source_keeps_absolute_target_across_model_jump():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1).target_angle_deg == pytest.approx(7.0)

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(
    requested_angle_deg=7.0,
    base_model_angle_deg=5.0,
    sequence=2,
    base_model_log_mono_time=10_100_000_000,
  )
  sm.recv_time["turboSteerAssist"] = 10.1

  decision = source.update(lat_active=True, model_angle_deg=40.0, now=10.2)
  assert decision.status == "active"
  assert decision.target_angle_deg == pytest.approx(7.0)


def test_turbo_steer_assist_source_requires_lineage_match_after_release():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1).status == "active"

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(active=False, sequence=2, base_model_log_mono_time=10_100_000_000)
  sm.recv_time["turboSteerAssist"] = 10.1
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.2).status == "inactive"

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(sequence=3, base_model_log_mono_time=10_200_000_000)
  sm.recv_time["turboSteerAssist"] = 10.2
  assert source.update(lat_active=True, model_angle_deg=21.0, now=10.3).status == "target_mismatch"


def test_turbo_steer_assist_source_accepts_newer_target_after_sequence_restart():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, sequence=20)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1).status == "active"

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(
    requested_angle_deg=9.0,
    base_model_angle_deg=6.0,
    sequence=1,
    base_model_log_mono_time=10_200_000_000,
  )
  sm.recv_time["turboSteerAssist"] = 10.2
  decision = source.update(lat_active=True, model_angle_deg=6.0, now=10.25)
  assert decision.target_angle_deg == pytest.approx(9.0)
  assert decision.status == "active"


def test_turbo_steer_assist_source_rejects_replayed_sequence():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, sequence=2)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1).status == "active"

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(requested_angle_deg=8.0, sequence=1)
  sm.recv_time["turboSteerAssist"] = 10.11
  decision = source.update(lat_active=True, model_angle_deg=5.0, now=10.12)
  assert decision.target_angle_deg is None
  assert decision.status == "out_of_order"
