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

  target, status = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert target == pytest.approx(7.0)
  assert status == "active"
  assert source.last_age_s == pytest.approx(0.1)
  assert source.last_context_age_s == pytest.approx(0.1)
  assert source.last_base_model_delta_deg == pytest.approx(0.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.11) == (7.0, "active")


def test_turbo_steer_assist_source_accepts_zero_degree_target():
  source = TurboSteerAssistSource(FakeSubMaster(requested_angle_deg=0.0))

  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (0.0, "active")


def test_turbo_steer_assist_source_clips_target_to_steering_range():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=200.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  target, status = source.update(lat_active=True, model_angle_deg=5.0, now=10.1)

  assert target == pytest.approx(180.0)
  assert status == "active"


def test_turbo_steer_assist_source_returns_absolute_blended_target():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=5.0, base_model_angle_deg=17.0)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25)

  target, status = source.update(lat_active=True, model_angle_deg=20.0, now=10.1)

  assert status == "active"
  assert source.last_requested_target_angle_deg == pytest.approx(5.0)
  assert target == pytest.approx(5.0)


def test_turbo_steer_assist_source_returns_none_when_inactive_or_stale():
  inactive = TurboSteerAssistSource(FakeSubMaster(active=False, requested_angle_deg=7.0))
  assert inactive.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (None, "inactive")

  stale = TurboSteerAssistSource(FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0), stale_timeout_s=0.25)
  assert stale.update(lat_active=True, model_angle_deg=5.0, now=10.5) == (None, "stale")

  lat_inactive = TurboSteerAssistSource(FakeSubMaster(requested_angle_deg=7.0))
  assert lat_inactive.update(lat_active=False, model_angle_deg=5.0, now=10.1) == (None, "lat_inactive")


def test_turbo_steer_assist_source_rejects_stale_target_context():
  sm = FakeSubMaster(recv_time=10.4, requested_angle_deg=7.0, base_model_log_mono_time=10_000_000_000)
  source = TurboSteerAssistSource(sm, stale_timeout_s=0.25, context_timeout_s=0.35)

  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.4) == (None, "stale_target_context")
  assert source.last_context_age_s == pytest.approx(0.4)


def test_turbo_steer_assist_source_rejects_target_mismatch():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)

  assert source.update(lat_active=True, model_angle_deg=21.0, now=10.1) == (None, "target_mismatch")
  assert source.last_base_model_delta_deg == pytest.approx(16.0)


def test_turbo_steer_assist_source_keeps_absolute_target_across_model_jump():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (7.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(
    requested_angle_deg=7.0,
    base_model_angle_deg=5.0,
    sequence=2,
    base_model_log_mono_time=10_100_000_000,
  )
  sm.recv_time["turboSteerAssist"] = 10.1

  target, status = source.update(lat_active=True, model_angle_deg=40.0, now=10.2)
  assert status == "active"
  assert target == pytest.approx(7.0)


def test_turbo_steer_assist_source_requires_lineage_match_after_release():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, base_model_angle_deg=5.0)
  source = TurboSteerAssistSource(sm, target_mismatch_deg=15.0)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (7.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(active=False, sequence=2, base_model_log_mono_time=10_100_000_000)
  sm.recv_time["turboSteerAssist"] = 10.1
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.2) == (None, "inactive")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(sequence=3, base_model_log_mono_time=10_200_000_000)
  sm.recv_time["turboSteerAssist"] = 10.2
  assert source.update(lat_active=True, model_angle_deg=21.0, now=10.3) == (None, "target_mismatch")


def test_turbo_steer_assist_source_accepts_newer_target_after_sequence_restart():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, sequence=20)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (7.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(
    requested_angle_deg=9.0,
    base_model_angle_deg=6.0,
    sequence=1,
    base_model_log_mono_time=10_200_000_000,
  )
  sm.recv_time["turboSteerAssist"] = 10.2
  assert source.update(lat_active=True, model_angle_deg=6.0, now=10.25) == (9.0, "active")


def test_turbo_steer_assist_source_rejects_replayed_sequence():
  sm = FakeSubMaster(recv_time=10.0, requested_angle_deg=7.0, sequence=2)
  source = TurboSteerAssistSource(sm)
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.1) == (7.0, "active")

  sm.data["turboSteerAssist"] = FakeTurboSteerAssist(requested_angle_deg=8.0, sequence=1)
  sm.recv_time["turboSteerAssist"] = 10.11
  assert source.update(lat_active=True, model_angle_deg=5.0, now=10.12) == (None, "out_of_order")
