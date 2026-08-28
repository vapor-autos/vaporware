import math

import pytest

from openpilot.cereal import messaging
from openpilot.tools.turbo.g29d import (
  AssistTargetSource,
  PassiveWheelObserver,
  SpeedSource,
  SteerAssistNudgePublisher,
  _effect_position_to_steering_angle_deg,
  _make_assist_torque_controller,
  _steering_angle_to_g29_target,
)


class FakeCarState:
  def __init__(self, v_ego: float):
    self.vEgo = v_ego


class FakeActuatorsOutput:
  def __init__(self, steering_angle_deg: float):
    self.steeringAngleDeg = steering_angle_deg


class FakeCarOutput:
  def __init__(self, steering_angle_deg: float):
    self.actuatorsOutput = FakeActuatorsOutput(steering_angle_deg)


class FakeAngleState:
  def __init__(self, steering_angle_desired_deg: float):
    self.steeringAngleDesiredDeg = steering_angle_desired_deg


class FakeLateralControlState:
  def __init__(self, steering_angle_desired_deg: float, state: str = "angleState"):
    self.angleState = FakeAngleState(steering_angle_desired_deg)
    self.state = state

  def which(self):
    return self.state


class FakeControlsState:
  def __init__(self, steering_angle_desired_deg: float, state: str = "angleState"):
    self.lateralControlState = FakeLateralControlState(steering_angle_desired_deg, state=state)


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
    controlsstate_seen=None,
    controlsstate_valid=None,
    controlsstate_recv_time=None,
    steering_angle_desired_deg=0.0,
    lateral_control_state="angleState",
    selfdrive_seen=None,
    selfdrive_valid=None,
    selfdrive_recv_time=None,
    selfdrive_enabled=False,
    selfdrive_active=False,
  ):
    self.seen = {
      "carState": seen,
      "carOutput": seen if caroutput_seen is None else caroutput_seen,
      "controlsState": seen if controlsstate_seen is None else controlsstate_seen,
      "selfdriveState": seen if selfdrive_seen is None else selfdrive_seen,
    }
    self.valid = {
      "carState": valid,
      "carOutput": valid if caroutput_valid is None else caroutput_valid,
      "controlsState": valid if controlsstate_valid is None else controlsstate_valid,
      "selfdriveState": valid if selfdrive_valid is None else selfdrive_valid,
    }
    self.recv_time = {
      "carState": recv_time,
      "carOutput": recv_time if caroutput_recv_time is None else caroutput_recv_time,
      "controlsState": recv_time if controlsstate_recv_time is None else controlsstate_recv_time,
      "selfdriveState": recv_time if selfdrive_recv_time is None else selfdrive_recv_time,
    }
    self.data = {
      "carState": FakeCarState(v_ego),
      "carOutput": FakeCarOutput(steering_angle_deg),
      "controlsState": FakeControlsState(steering_angle_desired_deg, state=lateral_control_state),
      "selfdriveState": FakeSelfdriveState(enabled=selfdrive_enabled, active=selfdrive_active),
    }
    self.update_count = 0

  def update(self, timeout):
    assert timeout == 0
    self.update_count += 1

  def __getitem__(self, service):
    return self.data[service]


class FakeSocket:
  def __init__(self):
    self.sent = []

  def send(self, data: bytes):
    self.sent.append(data)


class FakeG29Effects:
  def __init__(self):
    self.anticenter_calls = []
    self.friction_calls = []

  def set_anticenter(self, **kwargs):
    self.anticenter_calls.append(kwargs)

  def set_friction(self, friction: float):
    self.friction_calls.append(friction)


def make_steer_assist_publisher(sock, **kwargs):
  options = {
    "response_tau_s": 0.0,
    "engage_warmup_s": 0.0,
    "max_target_step_deg": 1e6,
    "max_target_rate_deg_s": 1e6,
    "target_recovery_s": 0.0,
    "override_persistence_s": 0.0,
    "nudge_filter_tau_s": 0.0,
    "max_nudge_rate_deg_s": 1e6,
  }
  options.update(kwargs)
  return SteerAssistNudgePublisher(sock, **options)


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


def test_effect_position_to_steering_angle_deg_matches_g29_quantization():
  assert _effect_position_to_steering_angle_deg(0.5) == pytest.approx(-180.0 / 255.0)


def test_passive_wheel_observer_models_first_order_response():
  observer = PassiveWheelObserver(response_tau_s=0.12)

  assert observer.update(0.0, now=10.0) == pytest.approx(0.0)
  first = observer.update(100.0, now=10.12)
  assert first == pytest.approx(100.0 * (1.0 - math.exp(-1.0)))
  second = observer.update(100.0, now=10.24)
  assert second == pytest.approx(first + (100.0 - first) * (1.0 - math.exp(-1.0)))
  assert observer.ready


def test_passive_wheel_observer_resets_on_non_monotonic_time():
  observer = PassiveWheelObserver(response_tau_s=0.12)
  observer.update(0.0, now=10.0)
  observer.update(10.0, now=10.2)

  assert observer.update(20.0, now=9.0) == pytest.approx(20.0)
  assert observer.ready


def test_assist_torque_controller_uses_fixed_engaged_tune():
  g29 = FakeG29Effects()
  controller = _make_assist_torque_controller(g29)

  command = controller.update(longitudinal_velocity_m_s=1.0, steering=0.0, target_steering=0.25)

  assert command.force == pytest.approx(0.4)
  assert command.friction == pytest.approx(0.25)
  assert g29.anticenter_calls[-1]["force"] == pytest.approx(0.4)
  assert g29.friction_calls[-1] == pytest.approx(0.25)


def test_assist_target_source_uses_fresh_engaged_controlsstate_target():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.0,
    steering_angle_deg=12.0,
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    steering_angle_desired_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.1)

  assert target == pytest.approx(-0.25)
  assert name == "controlsState"
  assert source.last_controlsstate_age_s == pytest.approx(0.1)
  assert source.last_caroutput_age_s == pytest.approx(0.1)
  assert source.last_selfdrive_age_s == pytest.approx(0.1)
  assert source.last_target_angle_deg == pytest.approx(45.0)
  assert source.last_applied_angle_deg == pytest.approx(12.0)
  assert sm.update_count == 1


def test_assist_target_source_falls_back_when_disengaged():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.0,
    steering_angle_deg=45.0,
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    steering_angle_desired_deg=45.0,
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


def test_assist_target_source_falls_back_when_controlsstate_is_stale():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.2,
    steering_angle_deg=45.0,
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    steering_angle_desired_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.2,
    selfdrive_active=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.3)

  assert target is None
  assert name == "controlsState_stale"


def test_assist_target_source_falls_back_when_controlsstate_is_not_angle():
  sm = FakeSubMaster(
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    lateral_control_state="torqueState",
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_active=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.1)

  assert target is None
  assert name == "controlsState_not_angle"


def test_assist_target_source_falls_back_when_selfdrive_state_is_stale():
  sm = FakeSubMaster(
    caroutput_seen=True,
    caroutput_valid=True,
    caroutput_recv_time=10.2,
    steering_angle_deg=45.0,
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.2,
    steering_angle_desired_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25)

  target, name = source.update(now=10.3)

  assert target is None
  assert name == "selfdriveState_stale"


def test_steer_assist_nudge_publisher_sends_active_nudge_message():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, publish_interval_s=0.05)

  assert publisher.update(
    {"steering": -0.6},
    target_steering=-0.5,
    target_steering_angle_deg=90.0,
    haptic_target_angle_deg=90.0,
    now=10.0,
  )

  msg = messaging.log_from_bytes(sock.sent[0])
  assert msg.which() == "turboSteerAssist"
  assert msg.valid
  assert msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(18.0)
  assert msg.turboSteerAssist.wheelSteering == pytest.approx(-0.6)
  assert msg.turboSteerAssist.targetSteering == pytest.approx(-0.5)
  assert msg.turboSteerAssist.targetSteeringAngleDeg == pytest.approx(90.0)
  assert publisher.last_expected_wheel_angle_deg == pytest.approx(90.0)
  assert publisher.last_residual_angle_deg == pytest.approx(18.0)


def test_steer_assist_nudge_publisher_ignores_passive_wheel_response():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, publish_interval_s=0.0, response_tau_s=0.12)

  assert publisher.update(
    {"steering": 0.0},
    target_steering=0.0,
    target_steering_angle_deg=0.0,
    haptic_target_angle_deg=0.0,
    now=0.0,
  )

  expected_angle_deg = 20.0 * (1.0 - math.exp(-0.02 / 0.12))
  assert publisher.update(
    {"steering": -expected_angle_deg / 180.0},
    target_steering=_steering_angle_to_g29_target(20.0),
    target_steering_angle_deg=20.0,
    haptic_target_angle_deg=20.0,
    now=0.02,
  )

  msg = messaging.log_from_bytes(sock.sent[-1])
  assert msg.turboSteerAssist.active
  assert publisher.last_angle_error_deg == pytest.approx(expected_angle_deg - 20.0)
  assert publisher.last_expected_wheel_angle_deg == pytest.approx(expected_angle_deg)
  assert publisher.last_residual_angle_deg == pytest.approx(0.0)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)

  expected_angle_deg += (20.0 - expected_angle_deg) * (1.0 - math.exp(-0.02 / 0.12))
  assert publisher.update(
    {"steering": -(expected_angle_deg + 10.0) / 180.0},
    target_steering=_steering_angle_to_g29_target(20.0),
    target_steering_angle_deg=20.0,
    haptic_target_angle_deg=20.0,
    now=0.04,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_expected_wheel_angle_deg == pytest.approx(expected_angle_deg)
  assert publisher.last_residual_angle_deg == pytest.approx(10.0)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(10.0)


def test_steer_assist_nudge_publisher_gates_abrupt_target_changes():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    publish_interval_s=0.0,
    response_tau_s=0.12,
    max_target_step_deg=15.0,
    max_target_rate_deg_s=300.0,
    target_recovery_s=0.4,
  )

  assert publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  assert publisher.update({"steering": 0.0}, -20.0 / 180.0, 20.0, 20.0, now=10.1)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)
  assert publisher.last_observer_status == "target_unstable"

  assert publisher.update({"steering": -20.0 / 180.0}, -20.0 / 180.0, 20.0, 20.0, now=10.51)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert msg.turboSteerAssist.active
  assert publisher.last_observer_status == "ready"


def test_steer_assist_nudge_publisher_requires_persistence_and_rate_limits():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    publish_interval_s=0.0,
    override_persistence_s=0.1,
    max_nudge_rate_deg_s=100.0,
  )

  assert publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  assert publisher.update({"steering": -20.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  assert publisher.last_override_status == "candidate"
  assert publisher.last_nudge_angle_deg == pytest.approx(0.0)

  assert publisher.update({"steering": -20.0 / 180.0}, 0.0, 0.0, 0.0, now=10.13)
  assert publisher.last_override_status == "active"
  assert publisher.last_raw_nudge_angle_deg == pytest.approx(20.0)
  assert publisher.last_nudge_angle_deg == pytest.approx(11.0)

  assert publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.15)
  assert publisher.last_override_status == "releasing"
  assert publisher.last_nudge_angle_deg == pytest.approx(9.0)


def test_steer_assist_nudge_publisher_suppresses_engage_warmup():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, publish_interval_s=0.0, engage_warmup_s=0.5)

  assert publisher.update(
    {"steering": -20.0 / 180.0},
    target_steering=0.0,
    target_steering_angle_deg=0.0,
    haptic_target_angle_deg=0.0,
    now=10.0,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)
  assert publisher.last_observer_status == "engage_warmup"

  assert publisher.update(
    {"steering": -20.0 / 180.0},
    target_steering=0.0,
    target_steering_angle_deg=0.0,
    haptic_target_angle_deg=0.0,
    now=10.5,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(20.0)


def test_steer_assist_nudge_publisher_rate_limits_and_sends_inactive():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, publish_interval_s=0.05)

  assert publisher.update(
    {"steering": -0.6},
    target_steering=-0.5,
    target_steering_angle_deg=90.0,
    haptic_target_angle_deg=90.0,
    now=10.0,
  )
  assert not publisher.update(
    {"steering": -0.6},
    target_steering=-0.5,
    target_steering_angle_deg=90.0,
    haptic_target_angle_deg=90.0,
    now=10.02,
  )
  assert publisher.update(
    {"steering": -0.6},
    target_steering=None,
    target_steering_angle_deg=None,
    haptic_target_angle_deg=108.0,
    now=10.06,
  )

  msg = messaging.log_from_bytes(sock.sent[-1])
  assert msg.which() == "turboSteerAssist"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)
