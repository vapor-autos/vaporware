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
    nudge_angle_deg: float = 0.0,
    target_angle_deg: float = 5.0,
    sequence: int = 1,
    base_target_log_mono_time: int = 10_000_000_000,
  ):
    self.active = active
    self.nudgeAngleDeg = nudge_angle_deg
    self.targetSteeringAngleDeg = target_angle_deg
    self.sequence = sequence
    self.baseTargetLogMonoTime = base_target_log_mono_time


class FakeSubMaster:
  def __init__(
    self,
    seen=True,
    valid=True,
    recv_time=10.0,
    active=True,
    nudge_angle_deg=0.0,
    target_angle_deg=5.0,
    sequence=1,
    base_target_log_mono_time=10_000_000_000,
  ):
    self.seen = {"turboSteerAssist": seen}
    self.valid = {"turboSteerAssist": valid}
    self.recv_time = {"turboSteerAssist": recv_time}
    self.data = {
      "turboSteerAssist": FakeTurboSteerAssist(
        active,
        nudge_angle_deg,
        target_angle_deg,
        sequence,
        base_target_log_mono_time,
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


def test_turbo_steer_assist_source_uses_fresh_active_nudge():
  sm = FakeSubMaster(recv_time=10.0, nudge_angle_deg=2.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  nudge, status = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert nudge == pytest.approx(2.0)
  assert status == "active"
  assert source.last_age_s == pytest.approx(0.1)
  assert source.last_context_age_s == pytest.approx(0.1)
  assert source.last_target_delta_deg == pytest.approx(0.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.11) == (2.0, "active")


def test_turbo_steer_assist_source_does_not_apply_small_nudge_cap():
  sm = FakeSubMaster(recv_time=10.0, nudge_angle_deg=8.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  nudge, status = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert nudge == pytest.approx(8.0)
  assert status == "active"
  assert source.last_raw_nudge_angle_deg == pytest.approx(8.0)


def test_turbo_steer_assist_source_returns_zero_when_inactive_or_stale():
  inactive = TurboSteerAssistSource(FakeSubMaster(active=False, nudge_angle_deg=2.0))
  assert inactive.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (0.0, "inactive")

  stale = TurboSteerAssistSource(FakeSubMaster(recv_time=10.0, nudge_angle_deg=2.0), stale_timeout_s=0.25)
  assert stale.update(lat_active=True, model_angle_deg=5.0, now=10.5) == (0.0, "stale")

  lat_inactive = TurboSteerAssistSource(FakeSubMaster(nudge_angle_deg=2.0))
  assert lat_inactive.update(lat_active=False, model_angle_deg=5.0, now=10.1) == (0.0, "lat_inactive")


def test_turbo_steer_assist_source_rejects_stale_target_context():
  sm = FakeSubMaster(recv_time=10.4, nudge_angle_deg=2.0, base_target_log_mono_time=10_000_000_000)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25, context_timeout_s=0.35)

  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.4) == (0.0, "stale_target_context")
  assert source.last_context_age_s == pytest.approx(0.4)


def test_turbo_steer_assist_source_rejects_target_mismatch():
  sm = FakeSubMaster(recv_time=10.0, nudge_angle_deg=2.0, target_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)

  assert source.update(lat_active=True, model_angle_deg=21.0, now=10.1) == (0.0, "target_mismatch")
  assert source.last_target_delta_deg == pytest.approx(16.0)


def test_turbo_steer_assist_source_accepts_newer_target_after_sequence_restart():
  sm = FakeSubMaster(recv_time=10.0, nudge_angle_deg=2.0, sequence=20)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (2.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(
    nudge_angle_deg=3.0,
    target_angle_deg=6.0,
    sequence=1,
    base_target_log_mono_time=10_200_000_000,
  )
  sm.recv_time["turboSteerAssist"] = 10.2
  assert source.update(lat_active=True, model_angle_deg=6.0, now=10.25) == (3.0, "active")


def test_turbo_steer_assist_source_rejects_replayed_sequence():
  sm = FakeSubMaster(recv_time=10.0, nudge_angle_deg=2.0, sequence=2)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (2.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(nudge_angle_deg=3.0, sequence=1)
  sm.recv_time["turboSteerAssist"] = 10.11
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.12) == (0.0, "out_of_order")
