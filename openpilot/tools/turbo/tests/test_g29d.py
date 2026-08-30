import pytest

from openpilot.cereal import messaging
from openpilot.tools.turbo.g29d import (
  AssistTargetSource,
  HapticTargetLimiter,
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
    controlsstate_log_mono_time=10_000_000_000,
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
    self.logMonoTime = {
      "carState": 0,
      "carOutput": 0,
      "controlsState": controlsstate_log_mono_time,
      "selfdriveState": 0,
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
    "tracking_duration_s": 0.0,
    "candidate_duration_s": 0.0,
    "min_opposing_velocity_deg_s": 0.0,
    "max_candidate_target_rate_deg_s": 1e6,
    "max_target_step_deg": 1e6,
    "max_target_rate_deg_s": 1e6,
    "wheel_velocity_tau_s": 0.0,
    "override_slew_rate_deg_s": float("inf"),
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


def test_haptic_target_limiter_starts_at_wheel_and_rate_limits():
  limiter = HapticTargetLimiter(max_rate_deg_s=180.0)

  assert limiter.update(90.0, wheel_angle_deg=10.0, now=10.0) == pytest.approx(10.0)
  assert limiter.update(90.0, wheel_angle_deg=10.0, now=10.1) == pytest.approx(28.0)
  assert limiter.update(20.0, wheel_angle_deg=10.0, now=10.2) == pytest.approx(20.0)


def test_haptic_target_limiter_resets_when_disengaged():
  limiter = HapticTargetLimiter(max_rate_deg_s=180.0)
  limiter.update(90.0, wheel_angle_deg=10.0, now=10.0)
  assert limiter.update(None, wheel_angle_deg=10.0, now=10.1) is None
  assert limiter.update(-90.0, wheel_angle_deg=-5.0, now=10.2) == pytest.approx(-5.0)


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
  assert source.last_target_log_mono_time == 10_000_000_000
  assert source.last_target_fresh
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
  assert source.last_target_log_mono_time == 0


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


def test_assist_target_source_holds_last_target_during_brief_feedback_staleness():
  sm = FakeSubMaster(
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    controlsstate_log_mono_time=10_000_000_000,
    steering_angle_desired_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25, stale_grace_s=0.15)
  source.update(now=10.1)

  target, name = source.update(now=10.3)

  assert target == pytest.approx(-0.25)
  assert name == "feedback_stale_hold"
  assert source.last_target_angle_deg == pytest.approx(45.0)
  assert source.last_target_log_mono_time == 10_000_000_000
  assert not source.last_target_fresh

  target, name = source.update(now=10.41)

  assert target is None
  assert name == "selfdriveState_stale"
  assert source.last_target_angle_deg is None


def test_assist_target_source_clears_held_target_on_fresh_disengagement():
  sm = FakeSubMaster(
    controlsstate_seen=True,
    controlsstate_valid=True,
    controlsstate_recv_time=10.0,
    steering_angle_desired_deg=45.0,
    selfdrive_seen=True,
    selfdrive_valid=True,
    selfdrive_recv_time=10.0,
    selfdrive_enabled=True,
  )
  source = AssistTargetSource(sm=sm, stale_timeout_s=0.25, stale_grace_s=0.15)
  source.update(now=10.1)
  sm.data["selfdriveState"].enabled = False
  sm.recv_time["selfdriveState"] = 10.2

  target, name = source.update(now=10.21)

  assert target is None
  assert name == "disengaged"
  assert source.last_target_angle_deg is None


def test_steer_assist_nudge_publisher_arms_after_measured_tracking():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, tracking_duration_s=0.3)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, base_target_log_mono_time=10_000_000_000, now=10.0)
  assert publisher.last_tracking_status == "acquiring_tracking"
  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, base_target_log_mono_time=10_000_000_000, now=10.29)
  assert publisher.last_tracking_status == "acquiring_tracking"
  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, base_target_log_mono_time=10_000_000_000, now=10.31)

  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active


def test_steer_assist_nudge_publisher_acquires_within_neutral_deadband():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, tracking_duration_s=0.3)

  publisher.update({"steering": -4.9 / 180.0}, 0.0, 0.0, 0.0, now=10.0)
  assert publisher.last_tracking_status == "acquiring_tracking"
  publisher.update({"steering": -4.9 / 180.0}, 0.0, 0.0, 0.0, now=10.31)

  assert publisher.last_tracking_status == "tracking"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_acquires_while_haptic_target_moves():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    tracking_duration_s=0.3,
    max_candidate_target_rate_deg_s=60.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -10.0 / 180.0}, -10.0 / 180.0, 10.0, 10.0, now=10.1)
  assert publisher.last_haptic_target_rate_deg_s == pytest.approx(100.0)
  assert publisher.last_tracking_status == "acquiring_tracking"

  publisher.update({"steering": -31.0 / 180.0}, -31.0 / 180.0, 31.0, 31.0, now=10.31)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_haptic_target_rate_deg_s == pytest.approx(100.0)
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active


def test_steer_assist_nudge_publisher_does_not_treat_haptic_lag_as_override():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    max_candidate_target_rate_deg_s=60.0,
    min_opposing_velocity_deg_s=10.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -2.0 / 180.0}, -10.0 / 180.0, 10.0, 10.0, now=10.02)

  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_haptic_target_rate_deg_s == pytest.approx(500.0)
  assert publisher.last_residual_angle_deg == pytest.approx(-8.0)
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)

  publisher.update({"steering": -6.0 / 180.0}, -10.0 / 180.0, 10.0, 10.0, now=10.04)
  assert publisher.last_tracking_status == "tracking"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_requires_opposing_motion():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, max_candidate_target_rate_deg_s=60.0)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=0.0)
  publisher.update(
    {"steering": -5.0 / 180.0},
    _steering_angle_to_g29_target(20.0),
    20.0,
    20.0,
    now=0.02,
  )
  assert publisher.last_tracking_status == "tracking"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active

  publisher.update(
    {"steering": -10.0 / 180.0},
    _steering_angle_to_g29_target(20.0),
    20.0,
    20.0,
    now=0.04,
  )
  assert publisher.last_tracking_status == "tracking"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_pauses_candidate_during_target_motion():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    candidate_duration_s=0.08,
    max_candidate_target_rate_deg_s=60.0,
    min_opposing_velocity_deg_s=10.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)
  assert publisher.last_tracking_status == "candidate"
  assert publisher.candidate_evidence_s == pytest.approx(0.02)

  publisher.update({"steering": -8.0 / 180.0}, -2.0 / 180.0, 2.0, 2.0, now=10.06)
  assert publisher.last_haptic_target_rate_deg_s == pytest.approx(100.0)
  assert publisher.last_tracking_status == "candidate"
  assert publisher.candidate_evidence_s == pytest.approx(0.02)

  publisher.update({"steering": -9.0 / 180.0}, -2.0 / 180.0, 2.0, 2.0, now=10.08)
  publisher.update({"steering": -10.0 / 180.0}, -2.0 / 180.0, 2.0, 2.0, now=10.10)
  publisher.update({"steering": -11.0 / 180.0}, -2.0 / 180.0, 2.0, 2.0, now=10.12)
  assert publisher.last_tracking_status == "candidate"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active

  publisher.update({"steering": -12.0 / 180.0}, -2.0 / 180.0, 2.0, 2.0, now=10.14)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert publisher.candidate_evidence_s == pytest.approx(0.1)
  assert msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(10.0)


def test_steer_assist_nudge_publisher_pauses_candidate_without_relative_motion():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    candidate_duration_s=0.08,
    max_candidate_target_rate_deg_s=60.0,
    min_opposing_velocity_deg_s=10.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)
  assert publisher.last_tracking_status == "candidate"
  assert publisher.candidate_evidence_s == pytest.approx(0.02)

  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.20)
  assert publisher.last_tracking_status == "candidate"
  assert publisher.candidate_evidence_s == pytest.approx(0.02)
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_latches_and_releases_override():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.08)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  assert publisher.last_tracking_status == "candidate"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active

  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)
  publisher.update({"steering": -8.0 / 180.0}, 0.0, 0.0, 0.0, now=10.06)
  publisher.update({"steering": -9.0 / 180.0}, 0.0, 0.0, 0.0, now=10.08)
  publisher.update({"steering": -10.0 / 180.0}, 0.0, 0.0, 0.0, now=10.10)
  publisher.update({"steering": -11.0 / 180.0}, 0.0, 0.0, 0.0, now=10.12)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(11.0)

  publisher.update({"steering": -10.0 / 180.0}, 0.0, 0.0, 0.0, now=10.13)
  assert messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.nudgeAngleDeg == pytest.approx(10.0)

  publisher.update({"steering": -2.0 / 180.0}, 0.0, 0.0, 0.0, now=10.15)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)


def test_steer_assist_nudge_publisher_slews_only_override_acquisition():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.0, override_slew_rate_deg_s=180.0)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -20.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert msg.turboSteerAssist.active
  assert publisher.last_desired_blended_target_angle_deg == pytest.approx(20.0)
  assert publisher.last_blended_target_angle_deg == pytest.approx(0.0)
  assert publisher.last_override_slewing
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)

  publisher.update({"steering": -20.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)
  assert publisher.last_blended_target_angle_deg == pytest.approx(3.6)
  assert publisher.last_override_slewing

  publisher.update({"steering": -20.0 / 180.0}, 0.0, 0.0, 0.0, now=10.14)
  assert publisher.last_blended_target_angle_deg == pytest.approx(20.0)
  assert not publisher.last_override_slewing

  publisher.update({"steering": -25.0 / 180.0}, 0.0, 0.0, 0.0, now=10.16)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_desired_blended_target_angle_deg == pytest.approx(25.0)
  assert publisher.last_blended_target_angle_deg == pytest.approx(25.0)
  assert not publisher.last_override_slewing
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(25.0)


def test_steer_assist_nudge_publisher_slew_starts_at_clipped_model_target():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.0, override_slew_rate_deg_s=180.0)

  publisher.update({"steering": -170.0 / 180.0}, _steering_angle_to_g29_target(200.0), 200.0, 170.0, now=10.0)
  publisher.update({"steering": -180.0 / 180.0}, _steering_angle_to_g29_target(200.0), 200.0, 170.0, now=10.02)
  msg = messaging.log_from_bytes(sock.sent[-1])

  assert publisher.last_tracking_status == "override"
  assert publisher.last_desired_blended_target_angle_deg == pytest.approx(180.0)
  assert publisher.last_blended_target_angle_deg == pytest.approx(180.0)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(-20.0)


def test_steer_assist_nudge_publisher_cancels_candidate_when_spring_recovers():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.1)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  assert publisher.last_tracking_status == "candidate"
  publisher.update({"steering": -5.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)

  assert publisher.last_tracking_status == "tracking"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_disarms_on_model_jump_until_wheel_tracks():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    tracking_duration_s=0.3,
    max_target_step_deg=15.0,
    max_target_rate_deg_s=300.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.31)
  assert publisher.last_tracking_status == "tracking"

  publisher.update({"steering": 0.0}, -100.0 / 180.0, 100.0, 18.0, now=10.41)
  assert publisher.last_tracking_status == "target_unstable"
  assert not messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active

  publisher.update({"steering": 0.0}, -100.0 / 180.0, 100.0, 90.0, now=11.0)
  assert publisher.last_tracking_status == "disarmed"
  publisher.update({"steering": -0.5}, -100.0 / 180.0, 100.0, 90.0, now=12.0)
  assert publisher.last_tracking_status == "acquiring_tracking"
  publisher.update({"steering": -0.5}, -100.0 / 180.0, 100.0, 90.0, now=12.31)
  assert publisher.last_tracking_status == "tracking"


def test_steer_assist_nudge_publisher_preserves_blended_target_across_model_jump():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(
    sock,
    candidate_duration_s=0.08,
    max_target_step_deg=15.0,
    max_target_rate_deg_s=300.0,
  )

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  publisher.update({"steering": -7.35 / 180.0}, 0.0, 0.0, 0.0, now=10.11)
  assert publisher.last_tracking_status == "override"

  publisher.update(
    {"steering": -10.5 / 180.0},
    _steering_angle_to_g29_target(17.38),
    17.38,
    4.24,
    now=10.134,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert msg.turboSteerAssist.active
  assert publisher.last_haptic_nudge_angle_deg == pytest.approx(0.993, abs=0.001)
  assert publisher.last_blended_target_angle_deg == pytest.approx(5.233, abs=0.001)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(5.233 - 17.38, abs=0.001)
  assert 17.38 + msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(publisher.last_blended_target_angle_deg)

  publisher.update(
    {"steering": -8.0 / 180.0},
    _steering_angle_to_g29_target(17.38),
    17.38,
    4.24,
    now=10.158,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert msg.turboSteerAssist.active
  assert publisher.last_haptic_nudge_angle_deg == pytest.approx(0.0)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(4.24 - 17.38)

  publisher.update(
    {"steering": -17.0 / 180.0},
    _steering_angle_to_g29_target(17.38),
    17.38,
    17.38,
    now=10.182,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)


def test_steer_assist_nudge_publisher_encodes_absolute_target_at_steering_limit():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  assert publisher.last_tracking_status == "override"

  publisher.update(
    {"steering": 65.0 / 180.0},
    _steering_angle_to_g29_target(-294.0),
    -294.0,
    -65.0,
    now=10.04,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert msg.turboSteerAssist.active
  assert publisher.last_blended_target_angle_deg == pytest.approx(-65.0)
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(229.0)
  assert -294.0 + msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(-65.0)

  publisher.update(
    {"steering": 1.0},
    _steering_angle_to_g29_target(-294.0),
    -294.0,
    -180.0,
    now=10.06,
  )
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)


def test_steer_assist_nudge_publisher_uses_source_timestamp_for_target_rate():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, max_target_step_deg=15.0, max_target_rate_deg_s=300.0)

  publisher.update(
    {"steering": 0.0},
    0.0,
    0.0,
    0.0,
    base_target_log_mono_time=10_000_000_000,
    now=10.0,
  )
  publisher.update(
    {"steering": 0.0},
    _steering_angle_to_g29_target(8.78),
    8.78,
    4.32,
    base_target_log_mono_time=10_050_000_000,
    now=10.024,
  )

  assert publisher.last_target_interval_s == pytest.approx(0.05)
  assert publisher.last_target_rate_deg_s == pytest.approx(175.6)
  assert publisher.last_tracking_status != "target_unstable"


def test_steer_assist_nudge_publisher_preserves_tracking_but_publishes_inactive_when_stale():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.08)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  assert publisher.last_tracking_status == "tracking"

  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, input_fresh=False, now=10.02)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "tracking"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)

  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.04)
  assert publisher.last_tracking_status == "candidate"


def test_steer_assist_nudge_publisher_preserves_override_but_publishes_inactive_when_stale():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock, candidate_duration_s=0.08)

  publisher.update({"steering": 0.0}, 0.0, 0.0, 0.0, now=10.0)
  publisher.update({"steering": -6.0 / 180.0}, 0.0, 0.0, 0.0, now=10.02)
  publisher.update({"steering": -7.0 / 180.0}, 0.0, 0.0, 0.0, now=10.11)
  assert publisher.last_tracking_status == "override"

  publisher.update({"steering": -8.0 / 180.0}, 0.0, 0.0, 0.0, input_fresh=False, now=10.13)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert publisher.last_tracking_status == "override"
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)

  publisher.update({"steering": -8.0 / 180.0}, 0.0, 0.0, 0.0, now=10.15)
  assert messaging.log_from_bytes(sock.sent[-1]).turboSteerAssist.active


def test_steer_assist_nudge_publisher_sends_every_update_with_target_lineage():
  sock = FakeSocket()
  publisher = make_steer_assist_publisher(sock)

  publisher.update(
    {"steering": 0.0},
    target_steering=0.0,
    target_steering_angle_deg=0.0,
    haptic_target_angle_deg=0.0,
    base_target_log_mono_time=10_000_000_000,
    now=10.0,
  )
  publisher.update(
    {"steering": 0.0},
    target_steering=0.0,
    target_steering_angle_deg=0.0,
    haptic_target_angle_deg=0.0,
    base_target_log_mono_time=10_000_000_000,
    now=10.02,
  )
  assert len(sock.sent) == 2
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert msg.turboSteerAssist.sequence == 2
  assert msg.turboSteerAssist.baseTargetLogMonoTime == 10_000_000_000

  publisher.update({"steering": 0.0}, None, None, 0.0, base_target_log_mono_time=10_000_000_000, now=10.04)
  msg = messaging.log_from_bytes(sock.sent[-1])
  assert not msg.turboSteerAssist.active
  assert msg.turboSteerAssist.nudgeAngleDeg == pytest.approx(0.0)
  assert msg.turboSteerAssist.baseTargetLogMonoTime == 0
