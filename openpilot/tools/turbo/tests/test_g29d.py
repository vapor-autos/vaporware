from openpilot.tools.turbo.g29d import AssistTargetSource, SpeedSource, _steering_angle_to_g29_target
import pytest


class FakeCarState:
  def __init__(self, v_ego: float):
    self.vEgo = v_ego


class FakeActuatorsOutput:
  def __init__(self, steering_angle_deg: float):
    self.steeringAngleDeg = steering_angle_deg


class FakeCarOutput:
  def __init__(self, steering_angle_deg: float):
    self.actuatorsOutput = FakeActuatorsOutput(steering_angle_deg)


class FakeSelfdriveState:
  def __init__(self, enabled=False, active=False):
    self.enabled = enabled
    self.active = active


class FakeSubMaster:
  def __init__(
    self,
    seen=False,
    valid=False,
    recv_time=0.0,
    v_ego=0.0,
    caroutput_seen=None,
    caroutput_valid=None,
    caroutput_recv_time=None,
    steering_angle_deg=0.0,
    selfdrive_seen=None,
    selfdrive_valid=None,
    selfdrive_recv_time=None,
    selfdrive_enabled=False,
    selfdrive_active=False,
  ):
    self.seen = {
      "carState": seen,
      "carOutput": seen if caroutput_seen is None else caroutput_seen,
      "selfdriveState": seen if selfdrive_seen is None else selfdrive_seen,
    }
    self.valid = {
      "carState": valid,
      "carOutput": valid if caroutput_valid is None else caroutput_valid,
      "selfdriveState": valid if selfdrive_valid is None else selfdrive_valid,
    }
    self.recv_time = {
      "carState": recv_time,
      "carOutput": recv_time if caroutput_recv_time is None else caroutput_recv_time,
      "selfdriveState": recv_time if selfdrive_recv_time is None else selfdrive_recv_time,
    }
    self.data = {
      "carState": FakeCarState(v_ego),
      "carOutput": FakeCarOutput(steering_angle_deg),
      "selfdriveState": FakeSelfdriveState(enabled=selfdrive_enabled, active=selfdrive_active),
    }
    self.update_count = 0

  def update(self, timeout):
    assert timeout == 0
    self.update_count += 1

  def __getitem__(self, service):
    return self.data[service]


def test_speed_source_uses_fresh_carstate_speed():
  sm = FakeSubMaster(seen=True, valid=True, recv_time=10.0, v_ego=-3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 1.0}, now=10.1)

  assert velocity == 3.5
  assert name == "carState"
  assert source.last_carstate_age_s == pytest.approx(0.1)
  assert sm.update_count == 1


def test_speed_source_falls_back_to_pedal_when_carstate_is_stale():
  sm = FakeSubMaster(seen=True, valid=True, recv_time=10.0, v_ego=3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 0.0}, now=10.5)

  assert velocity == 10.0
  assert name == "pedal"


def test_speed_source_falls_back_to_pedal_when_carstate_is_invalid():
  sm = FakeSubMaster(seen=True, valid=False, recv_time=10.0, v_ego=3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": -1.0}, now=10.1)

  assert velocity == 0.0
  assert name == "pedal"


def test_speed_source_falls_back_to_pedal_before_carstate_seen():
  sm = FakeSubMaster(seen=False, valid=False)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 1.0}, now=10.1)

  assert velocity == 20.0
  assert name == "pedal"
  assert source.last_carstate_age_s is None


def test_steering_angle_to_g29_target_uses_teleop_sign():
  assert _steering_angle_to_g29_target(90.0) == pytest.approx(-0.5)
  assert _steering_angle_to_g29_target(-90.0) == pytest.approx(0.5)


def test_steering_angle_to_g29_target_clips_to_wheel_range():
  assert _steering_angle_to_g29_target(360.0) == pytest.approx(-1.0)
  assert _steering_angle_to_g29_target(-360.0) == pytest.approx(1.0)


def test_assist_target_source_uses_fresh_engaged_caroutput():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.0,
    steering_angle_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.1)

  assert target == pytest.approx(-0.25)
  assert name == "carOutput"
  assert source.last_caroutput_age_s == pytest.approx(0.1)
  assert source.last_selfdrive_age_s == pytest.approx(0.1)
  assert source.last_target_angle_deg == pytest.approx(45.0)
  assert sm.update_count == 1


def test_assist_target_source_falls_back_when_disengaged():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.0,
    steering_angle_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=False,
    selfdrive_active=False,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.1)

  assert target is None
  assert name == "disengaged"
  assert source.last_target_angle_deg is None


def test_assist_target_source_falls_back_when_caroutput_is_stale():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.0,
    steering_angle_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.2,
    selfdrive_active=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.3)

  assert target is None
  assert name == "carOutput_stale"


def test_assist_target_source_falls_back_when_selfdrive_state_is_stale():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.2,
    steering_angle_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.3)

  assert target is None
  assert name == "selfdriveState_stale"
