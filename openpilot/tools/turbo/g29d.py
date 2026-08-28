#!/usr/bin/env python3
import math
import time

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.turbo_steer_assist import (
  DEFAULT_FULL_ASSIST_ERROR_DEG,
  DEFAULT_INNER_DEADBAND_DEG,
  compute_nudge_angle_deg,
  g29_steering_to_angle_deg,
  steering_angle_to_g29_target,
)
from openpilot.tools.turbo.teleop_metrics import default_latest_json_path, write_metrics_payload

RETRY_DELAY = 2.0
PUBLISH_INTERVAL = 0.02
LOG_INTERVAL_FRAMES = 50

TORQUE_SIM_MAX_VELOCITY_M_S = 20.0
TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S = 8.0
TORQUE_SIM_CARSTATE_STALE_S = 0.25
TORQUE_SIM_ASSIST_STALE_S = 0.25
STEER_ASSIST_TRACKING_ERROR_DEG = 5.0
STEER_ASSIST_TRACKING_DURATION_S = 0.3
STEER_ASSIST_MIN_OPPOSING_VELOCITY_DEG_S = 10.0
STEER_ASSIST_CANDIDATE_DURATION_S = 0.08
STEER_ASSIST_MAX_ACQUIRE_TARGET_RATE_DEG_S = 60.0
STEER_ASSIST_MAX_TARGET_STEP_DEG = 15.0
STEER_ASSIST_MAX_TARGET_RATE_DEG_S = 300.0
STEER_ASSIST_WHEEL_VELOCITY_TAU_S = 0.06
STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S = 180.0
STEER_ASSIST_HAPTIC_FORCE = 0.4
STEER_ASSIST_HAPTIC_FRICTION = 0.25
STEER_ASSIST_METRICS_NAME = "g29-steer-assist"


def _clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def _accelerator_pedal(accelerator: float) -> float:
  return (_clip(accelerator, -1.0, 1.0) + 1.0) / 2.0


def _accelerator_to_simulated_velocity_m_s(accelerator: float, max_velocity_m_s: float) -> float:
  return _accelerator_pedal(accelerator) * max(0.0, max_velocity_m_s)


def _steering_angle_to_g29_target(steering_angle_deg: float) -> float:
  return steering_angle_to_g29_target(steering_angle_deg)


def _effect_position_to_steering_angle_deg(effect_position: float) -> float:
  quantized_position = round(_clip(effect_position, 0.0, 1.0) * 255.0) / 255.0
  return g29_steering_to_angle_deg(quantized_position * 2.0 - 1.0)


class HapticTargetLimiter:
  def __init__(self, max_rate_deg_s: float = STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S):
    self.max_rate_deg_s = max(0.0, max_rate_deg_s)
    self.last_update_time: float | None = None
    self.target_angle_deg: float | None = None

  def update(self, target_angle_deg: float | None, wheel_angle_deg: float, now: float | None = None) -> float | None:
    now = time.monotonic() if now is None else now
    if target_angle_deg is None:
      self.reset()
      return None

    target_angle_deg = _clip(float(target_angle_deg), -180.0, 180.0)
    if self.last_update_time is None or self.target_angle_deg is None or now < self.last_update_time:
      self.last_update_time = now
      self.target_angle_deg = _clip(float(wheel_angle_deg), -180.0, 180.0)
      return self.target_angle_deg

    dt = max(0.0, now - self.last_update_time)
    self.last_update_time = now
    max_delta_deg = self.max_rate_deg_s * dt
    self.target_angle_deg += _clip(target_angle_deg - self.target_angle_deg, -max_delta_deg, max_delta_deg)
    return self.target_angle_deg

  def reset(self) -> None:
    self.last_update_time = None
    self.target_angle_deg = None


class SpeedSource:
  def __init__(self, sm: messaging.SubMaster | None = None, stale_timeout_s: float = TORQUE_SIM_CARSTATE_STALE_S):
    self.sm = messaging.SubMaster(["carState"]) if sm is None else sm
    self.stale_timeout_s = stale_timeout_s
    self.last_carstate_age_s: float | None = None

  def update(self, state: dict, now: float | None = None) -> tuple[float, str]:
    self.sm.update(0)
    now = time.monotonic() if now is None else now
    seen = self.sm.seen["carState"]
    self.last_carstate_age_s = now - self.sm.recv_time["carState"] if seen else None

    if seen and self.sm.valid["carState"] and self.last_carstate_age_s <= self.stale_timeout_s:
      return abs(float(self.sm["carState"].vEgo)), "carState"

    return _accelerator_to_simulated_velocity_m_s(state["accelerator"], TORQUE_SIM_MAX_VELOCITY_M_S), "pedal"


class AssistTargetSource:
  def __init__(self, sm: messaging.SubMaster | None = None, stale_timeout_s: float = TORQUE_SIM_ASSIST_STALE_S):
    self.sm = messaging.SubMaster(["controlsState", "selfdriveState", "carOutput"]) if sm is None else sm
    self.stale_timeout_s = stale_timeout_s
    self.last_controlsstate_age_s: float | None = None
    self.last_caroutput_age_s: float | None = None
    self.last_selfdrive_age_s: float | None = None
    self.last_target_angle_deg: float | None = None
    self.last_target_log_mono_time = 0
    self.last_applied_angle_deg: float | None = None

  def update(self, now: float | None = None) -> tuple[float | None, str]:
    self.sm.update(0)
    now = time.monotonic() if now is None else now
    self.last_controlsstate_age_s = self._age("controlsState", now)
    self.last_caroutput_age_s = self._age("carOutput", now)
    self.last_selfdrive_age_s = self._age("selfdriveState", now)
    self.last_target_angle_deg = None
    self.last_target_log_mono_time = 0
    self.last_applied_angle_deg = self._applied_angle_deg()

    if not self._fresh("selfdriveState"):
      return None, "selfdriveState_stale"

    selfdrive_state = self.sm["selfdriveState"]
    if not (bool(selfdrive_state.enabled) or bool(selfdrive_state.active)):
      return None, "disengaged"

    if not self._fresh("controlsState"):
      return None, "controlsState_stale"

    controls_state = self.sm["controlsState"]
    lateral_state = controls_state.lateralControlState
    if lateral_state.which() != "angleState":
      return None, "controlsState_not_angle"

    angle_deg = float(lateral_state.angleState.steeringAngleDesiredDeg)
    self.last_target_angle_deg = angle_deg
    self.last_target_log_mono_time = int(self.sm.logMonoTime["controlsState"])
    return _steering_angle_to_g29_target(angle_deg), "controlsState"

  def _age(self, service: str, now: float) -> float | None:
    return now - self.sm.recv_time[service] if self.sm.seen[service] else None

  def _fresh(self, service: str) -> bool:
    ages = {
      "selfdriveState": self.last_selfdrive_age_s,
      "controlsState": self.last_controlsstate_age_s,
      "carOutput": self.last_caroutput_age_s,
    }
    age = ages[service]
    return self.sm.seen[service] and self.sm.valid[service] and age is not None and age <= self.stale_timeout_s

  def _applied_angle_deg(self) -> float | None:
    if not self.sm.seen["carOutput"] or not self.sm.valid["carOutput"]:
      return None
    return float(self.sm["carOutput"].actuatorsOutput.steeringAngleDeg)


def _dial_delta(events: list[dict]) -> int:
  return sum(int(event.get("delta", 0)) for event in events if event.get("type") == "dial")


def _button_down_events(events: list[dict]) -> set[str]:
  return {event["control"] for event in events if event.get("type") == "button_down" and "control" in event}


def _publish_state(sock, state: dict, events: list[dict]) -> None:
  buttons = state["buttons"]
  button_down = _button_down_events(events)

  msg = messaging.new_message("g29")
  msg.valid = True
  msg.g29.steering = state["steering"]
  msg.g29.accelerator = state["accelerator"]
  msg.g29.reverse = state["clutch"]
  msg.g29.dpadUp = "up" in button_down
  msg.g29.dpadDown = "down" in button_down
  msg.g29.dpadLeft = bool(buttons["left"])
  msg.g29.dpadRight = bool(buttons["right"])
  msg.g29.l2 = "L2" in button_down
  msg.g29.l3 = "L3" in button_down
  msg.g29.r2 = bool(buttons["R2"])
  msg.g29.r3 = bool(buttons["R3"])
  msg.g29.dial = _dial_delta(events)
  sock.send(msg.to_bytes())


class SteerAssistNudgePublisher:
  def __init__(
    self,
    sock,
    inner_deadband_deg: float = DEFAULT_INNER_DEADBAND_DEG,
    full_assist_error_deg: float = DEFAULT_FULL_ASSIST_ERROR_DEG,
    tracking_error_deg: float = STEER_ASSIST_TRACKING_ERROR_DEG,
    tracking_duration_s: float = STEER_ASSIST_TRACKING_DURATION_S,
    min_opposing_velocity_deg_s: float = STEER_ASSIST_MIN_OPPOSING_VELOCITY_DEG_S,
    candidate_duration_s: float = STEER_ASSIST_CANDIDATE_DURATION_S,
    max_acquire_target_rate_deg_s: float = STEER_ASSIST_MAX_ACQUIRE_TARGET_RATE_DEG_S,
    max_target_step_deg: float = STEER_ASSIST_MAX_TARGET_STEP_DEG,
    max_target_rate_deg_s: float = STEER_ASSIST_MAX_TARGET_RATE_DEG_S,
    wheel_velocity_tau_s: float = STEER_ASSIST_WHEEL_VELOCITY_TAU_S,
  ):
    self.sock = sock
    self.inner_deadband_deg = inner_deadband_deg
    self.full_assist_error_deg = full_assist_error_deg
    self.tracking_error_deg = max(0.0, tracking_error_deg)
    self.tracking_duration_s = max(0.0, tracking_duration_s)
    self.min_opposing_velocity_deg_s = max(0.0, min_opposing_velocity_deg_s)
    self.candidate_duration_s = max(0.0, candidate_duration_s)
    self.max_acquire_target_rate_deg_s = max(0.0, max_acquire_target_rate_deg_s)
    self.max_target_step_deg = max(0.0, max_target_step_deg)
    self.max_target_rate_deg_s = max(0.0, max_target_rate_deg_s)
    self.wheel_velocity_tau_s = max(0.0, wheel_velocity_tau_s)
    self.sequence = 0
    self.last_active = False
    self.last_tracking_status = "disengaged"
    self.last_wheel_delta = 0.0
    self.last_wheel_angle_deg = 0.0
    self.last_angle_error_deg = 0.0
    self.last_residual_angle_deg = 0.0
    self.last_raw_nudge_angle_deg = 0.0
    self.last_nudge_angle_deg = 0.0
    self.last_wheel_velocity_deg_s = 0.0
    self.last_relative_velocity_deg_s = 0.0
    self.last_haptic_target_angle_deg: float | None = None
    self.last_model_target_angle_deg: float | None = None
    self.last_target_step_deg = 0.0
    self.last_target_rate_deg_s = 0.0
    self.last_haptic_target_rate_deg_s = 0.0
    self.last_update_time: float | None = None
    self.last_motion_wheel_angle_deg: float | None = None
    self.tracking_since: float | None = None
    self.candidate_since: float | None = None
    self.candidate_error_sign = 0.0

  def _reset_detection(self, status: str) -> None:
    self.last_tracking_status = status
    self.tracking_since = None
    self.candidate_since = None
    self.candidate_error_sign = 0.0
    self.last_raw_nudge_angle_deg = 0.0
    self.last_nudge_angle_deg = 0.0

  def _reset_motion(self) -> None:
    self.last_update_time = None
    self.last_motion_wheel_angle_deg = None
    self.last_haptic_target_angle_deg = None
    self.last_model_target_angle_deg = None
    self.last_wheel_velocity_deg_s = 0.0
    self.last_relative_velocity_deg_s = 0.0
    self.last_target_step_deg = 0.0
    self.last_target_rate_deg_s = 0.0
    self.last_haptic_target_rate_deg_s = 0.0

  def _update_motion(
    self,
    wheel_angle_deg: float,
    model_target_angle_deg: float,
    haptic_target_angle_deg: float,
    now: float,
  ) -> bool:
    if self.last_update_time is None or now <= self.last_update_time:
      self._reset_motion()
      self.last_update_time = now
      self.last_motion_wheel_angle_deg = wheel_angle_deg
      self.last_model_target_angle_deg = model_target_angle_deg
      self.last_haptic_target_angle_deg = haptic_target_angle_deg
      return False

    dt = now - self.last_update_time
    raw_wheel_velocity_deg_s = (wheel_angle_deg - self.last_motion_wheel_angle_deg) / dt
    alpha = 1.0 if self.wheel_velocity_tau_s == 0.0 else 1.0 - math.exp(-dt / self.wheel_velocity_tau_s)
    self.last_wheel_velocity_deg_s += (raw_wheel_velocity_deg_s - self.last_wheel_velocity_deg_s) * alpha

    self.last_target_step_deg = model_target_angle_deg - self.last_model_target_angle_deg
    self.last_target_rate_deg_s = self.last_target_step_deg / dt
    self.last_haptic_target_rate_deg_s = (haptic_target_angle_deg - self.last_haptic_target_angle_deg) / dt
    self.last_relative_velocity_deg_s = self.last_wheel_velocity_deg_s - self.last_haptic_target_rate_deg_s

    self.last_update_time = now
    self.last_motion_wheel_angle_deg = wheel_angle_deg
    self.last_model_target_angle_deg = model_target_angle_deg
    self.last_haptic_target_angle_deg = haptic_target_angle_deg
    return abs(self.last_target_step_deg) > self.max_target_step_deg or abs(self.last_target_rate_deg_s) > self.max_target_rate_deg_s

  def _update_detection(self, target_unstable: bool, now: float) -> None:
    error_deg = self.last_residual_angle_deg
    target_rate_deg_s = self.last_haptic_target_rate_deg_s
    wheel_velocity_deg_s = self.last_wheel_velocity_deg_s

    if target_unstable and self.last_tracking_status != "override":
      self._reset_detection("target_unstable")
      return

    if self.last_tracking_status not in ("tracking", "candidate", "override"):
      tracking = abs(error_deg) <= self.tracking_error_deg and abs(target_rate_deg_s) <= self.max_acquire_target_rate_deg_s
      if not tracking:
        self._reset_detection("disarmed")
        return
      if self.tracking_since is None:
        self.tracking_since = now
      if now - self.tracking_since < self.tracking_duration_s:
        self.last_tracking_status = "acquiring_tracking"
        return
      self.last_tracking_status = "tracking"

    if self.last_tracking_status == "tracking":
      opposing = (
        abs(error_deg) > self.inner_deadband_deg
        and abs(target_rate_deg_s) <= self.max_acquire_target_rate_deg_s
        and abs(wheel_velocity_deg_s) >= self.min_opposing_velocity_deg_s
        and error_deg * wheel_velocity_deg_s > 0.0
      )
      if opposing:
        self.last_tracking_status = "candidate"
        self.candidate_since = now
        self.candidate_error_sign = math.copysign(1.0, error_deg)

    if self.last_tracking_status == "candidate":
      error_sign_changed = error_deg == 0.0 or math.copysign(1.0, error_deg) != self.candidate_error_sign
      moving_with_spring = abs(wheel_velocity_deg_s) >= self.min_opposing_velocity_deg_s and error_deg * wheel_velocity_deg_s < 0.0
      if abs(error_deg) <= self.inner_deadband_deg or abs(target_rate_deg_s) > self.max_acquire_target_rate_deg_s or error_sign_changed or moving_with_spring:
        self.last_tracking_status = "tracking"
        self.candidate_since = None
        self.candidate_error_sign = 0.0
      elif self.candidate_since is not None and now - self.candidate_since >= self.candidate_duration_s:
        self.last_tracking_status = "override"

    if self.last_tracking_status == "override" and abs(error_deg) <= self.tracking_error_deg:
      self.last_tracking_status = "tracking"
      self.candidate_since = None
      self.candidate_error_sign = 0.0

  def update(
    self,
    state: dict,
    target_steering: float | None,
    target_steering_angle_deg: float | None,
    haptic_target_angle_deg: float,
    base_target_log_mono_time: int = 0,
    now: float | None = None,
  ) -> bool:
    now = time.monotonic() if now is None else now
    wheel_steering = float(state["steering"])
    requested_active = target_steering is not None and target_steering_angle_deg is not None
    target = 0.0 if target_steering is None else float(target_steering)
    self.last_wheel_delta = wheel_steering - target
    self.last_wheel_angle_deg = g29_steering_to_angle_deg(wheel_steering)
    self.last_angle_error_deg = self.last_wheel_angle_deg - float(target_steering_angle_deg) if requested_active else 0.0
    self.last_residual_angle_deg = self.last_wheel_angle_deg - haptic_target_angle_deg if requested_active else 0.0

    if requested_active:
      target_unstable = self._update_motion(
        self.last_wheel_angle_deg,
        float(target_steering_angle_deg),
        haptic_target_angle_deg,
        now,
      )
      self._update_detection(target_unstable, now)
    else:
      self._reset_detection("disengaged")
      self._reset_motion()

    active = requested_active and self.last_tracking_status == "override"
    self.last_active = active
    self.last_raw_nudge_angle_deg = (
      compute_nudge_angle_deg(
        wheel_steering,
        haptic_target_angle_deg,
        inner_deadband_deg=self.inner_deadband_deg,
        full_assist_error_deg=self.full_assist_error_deg,
      )
      if active
      else 0.0
    )
    self.last_nudge_angle_deg = self.last_raw_nudge_angle_deg

    msg = messaging.new_message("turboSteerAssist")
    msg.valid = True
    msg.turboSteerAssist.active = active
    msg.turboSteerAssist.nudgeAngleDeg = self.last_nudge_angle_deg
    msg.turboSteerAssist.wheelSteering = wheel_steering
    msg.turboSteerAssist.targetSteering = target
    msg.turboSteerAssist.targetSteeringAngleDeg = 0.0 if target_steering_angle_deg is None else float(target_steering_angle_deg)
    self.sequence = (self.sequence + 1) & 0xFFFFFFFF
    if self.sequence == 0:
      self.sequence = 1
    msg.turboSteerAssist.sequence = self.sequence
    msg.turboSteerAssist.baseTargetLogMonoTime = int(base_target_log_mono_time) if requested_active else 0
    self.sock.send(msg.to_bytes())
    return True


def _make_torque_controller(g29):
  from g29py.advanced import SteeringTorqueConfig, SteeringTorqueController

  config = SteeringTorqueConfig(
    force_response_velocity_m_s=TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S,
  )
  return SteeringTorqueController(g29, config=config)


def _make_assist_torque_controller(g29):
  from g29py.advanced import SteeringTorqueConfig, SteeringTorqueController

  config = SteeringTorqueConfig(
    park_force=STEER_ASSIST_HAPTIC_FORCE,
    rolling_force=STEER_ASSIST_HAPTIC_FORCE,
    park_friction=STEER_ASSIST_HAPTIC_FRICTION,
    rolling_friction=STEER_ASSIST_HAPTIC_FRICTION,
    force_response_velocity_m_s=TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S,
  )
  return SteeringTorqueController(g29, config=config)


def _run(g29_sock, steer_assist_sock) -> None:
  from g29py import G29

  g29 = None
  try:
    g29 = G29()
    g29.set_range(400)
    torque_controller = _make_torque_controller(g29)
    assist_torque_controller = _make_assist_torque_controller(g29)
    speed_source = SpeedSource()
    assist_target_source = AssistTargetSource()
    haptic_target_limiter = HapticTargetLimiter()
    steer_assist_publisher = SteerAssistNudgePublisher(steer_assist_sock)
    steer_assist_metrics_file = default_latest_json_path(STEER_ASSIST_METRICS_NAME)
    g29.listen()

    print(
      " ".join(
        (
          "g29d torque_sim enabled",
          "speed_source=carState",
          "assist_target=controlsState",
          "pedal_fallback=True",
          f"carstate_stale={TORQUE_SIM_CARSTATE_STALE_S:.2f}s",
          f"assist_stale={TORQUE_SIM_ASSIST_STALE_S:.2f}s",
          f"steer_assist_inner_deadband={DEFAULT_INNER_DEADBAND_DEG:.1f}deg",
          f"steer_assist_full_error={DEFAULT_FULL_ASSIST_ERROR_DEG:.1f}deg",
          f"steer_assist_tracking_error={STEER_ASSIST_TRACKING_ERROR_DEG:.1f}deg",
          f"steer_assist_tracking_duration={STEER_ASSIST_TRACKING_DURATION_S:.2f}s",
          f"steer_assist_candidate_duration={STEER_ASSIST_CANDIDATE_DURATION_S:.2f}s",
          f"steer_assist_min_opposing_velocity={STEER_ASSIST_MIN_OPPOSING_VELOCITY_DEG_S:.0f}deg/s",
          f"steer_assist_haptic_max_rate={STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S:.0f}deg/s",
          f"steer_assist_haptic_force={STEER_ASSIST_HAPTIC_FORCE:.2f}",
          f"steer_assist_haptic_friction={STEER_ASSIST_HAPTIC_FRICTION:.2f}",
          f"max_velocity={TORQUE_SIM_MAX_VELOCITY_M_S:.1f}m/s",
          f"force_response={TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S:.1f}m/s",
        )
      ),
      flush=True,
    )

    frame = 0
    while True:
      time.sleep(PUBLISH_INTERVAL)
      now = time.monotonic()
      state = g29.get_state()
      events = g29.get_events()

      velocity, speed_source_name = speed_source.update(state, now=now)
      target_steering, assist_target_name = assist_target_source.update(now=now)
      wheel_angle_deg = g29_steering_to_angle_deg(float(state["steering"]))
      limited_target_angle_deg = haptic_target_limiter.update(
        assist_target_source.last_target_angle_deg,
        wheel_angle_deg,
        now=now,
      )
      haptic_target_steering = None if limited_target_angle_deg is None else _steering_angle_to_g29_target(limited_target_angle_deg)
      active_torque_controller = assist_torque_controller if target_steering is not None else torque_controller
      command = active_torque_controller.update(
        longitudinal_velocity_m_s=velocity,
        steering=state["steering"],
        target_steering=haptic_target_steering,
      )
      haptic_target_angle_deg = _effect_position_to_steering_angle_deg(command.target_position)
      assist_published = steer_assist_publisher.update(
        state,
        target_steering,
        assist_target_source.last_target_angle_deg,
        haptic_target_angle_deg,
        base_target_log_mono_time=assist_target_source.last_target_log_mono_time,
        now=now,
      )

      carstate_age = speed_source.last_carstate_age_s
      controlsstate_age = assist_target_source.last_controlsstate_age_s
      caroutput_age = assist_target_source.last_caroutput_age_s
      selfdrive_age = assist_target_source.last_selfdrive_age_s
      target_angle = assist_target_source.last_target_angle_deg
      applied_angle = assist_target_source.last_applied_angle_deg

      if assist_published:
        write_metrics_payload(
          {
            "steer_assist": {
              "requested_active": target_steering is not None,
              "active": steer_assist_publisher.last_active,
              "tracking_status": steer_assist_publisher.last_tracking_status,
              "tracking_error_deg": STEER_ASSIST_TRACKING_ERROR_DEG,
              "tracking_duration_s": STEER_ASSIST_TRACKING_DURATION_S,
              "min_opposing_velocity_deg_s": STEER_ASSIST_MIN_OPPOSING_VELOCITY_DEG_S,
              "candidate_duration_s": STEER_ASSIST_CANDIDATE_DURATION_S,
              "max_acquire_target_rate_deg_s": STEER_ASSIST_MAX_ACQUIRE_TARGET_RATE_DEG_S,
              "max_target_step_deg": STEER_ASSIST_MAX_TARGET_STEP_DEG,
              "max_target_rate_deg_s": STEER_ASSIST_MAX_TARGET_RATE_DEG_S,
              "haptic_max_rate_deg_s": STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S,
              "target_step_deg": steer_assist_publisher.last_target_step_deg,
              "target_rate_deg_s": steer_assist_publisher.last_target_rate_deg_s,
              "haptic_target_rate_deg_s": steer_assist_publisher.last_haptic_target_rate_deg_s,
              "wheel_velocity_deg_s": steer_assist_publisher.last_wheel_velocity_deg_s,
              "relative_velocity_deg_s": steer_assist_publisher.last_relative_velocity_deg_s,
              "velocity_m_s": velocity,
              "carstate_age_s": carstate_age,
              "controlsstate_age_s": controlsstate_age,
              "caroutput_age_s": caroutput_age,
              "selfdrive_age_s": selfdrive_age,
              "model_target_angle_deg": target_angle,
              "base_target_log_mono_time": assist_target_source.last_target_log_mono_time,
              "applied_angle_deg": applied_angle,
              "haptic_target_angle_deg": haptic_target_angle_deg,
              "wheel_angle_deg": steer_assist_publisher.last_wheel_angle_deg,
              "model_error_deg": steer_assist_publisher.last_angle_error_deg,
              "residual_deg": steer_assist_publisher.last_residual_angle_deg,
              "raw_nudge_deg": steer_assist_publisher.last_raw_nudge_angle_deg,
              "nudge_deg": steer_assist_publisher.last_nudge_angle_deg,
              "force": command.force,
              "friction": command.friction,
            },
          },
          latest_file=steer_assist_metrics_file,
          print_line=False,
        )

      if frame % LOG_INTERVAL_FRAMES == 0:
        carstate_age_text = "none" if carstate_age is None else f"{carstate_age:.3f}s"
        controlsstate_age_text = "none" if controlsstate_age is None else f"{controlsstate_age:.3f}s"
        caroutput_age_text = "none" if caroutput_age is None else f"{caroutput_age:.3f}s"
        selfdrive_age_text = "none" if selfdrive_age is None else f"{selfdrive_age:.3f}s"
        target_angle_text = "none" if target_angle is None else f"{target_angle:.2f}deg"
        target_steering_text = "none" if target_steering is None else f"{target_steering:.3f}"
        applied_angle_text = "none" if applied_angle is None else f"{applied_angle:.2f}deg"
        print(
          " ".join(
            (
              "g29d torque_sim",
              f"speed_source={speed_source_name}",
              f"assist_target={assist_target_name}",
              f"velocity={velocity:.2f}m/s",
              f"carstate_age={carstate_age_text}",
              f"controlsstate_age={controlsstate_age_text}",
              f"caroutput_age={caroutput_age_text}",
              f"selfdrive_age={selfdrive_age_text}",
              f"target_angle={target_angle_text}",
              f"applied_angle={applied_angle_text}",
              f"target_steering={target_steering_text}",
              f"tracking={steer_assist_publisher.last_tracking_status}",
              f"haptic_target={haptic_target_angle_deg:.2f}deg",
              f"wheel_angle={steer_assist_publisher.last_wheel_angle_deg:.2f}deg",
              f"model_error={steer_assist_publisher.last_angle_error_deg:.2f}deg",
              f"residual={steer_assist_publisher.last_residual_angle_deg:.2f}deg",
              f"wheel_velocity={steer_assist_publisher.last_wheel_velocity_deg_s:.2f}deg/s",
              f"relative_velocity={steer_assist_publisher.last_relative_velocity_deg_s:.2f}deg/s",
              f"raw_nudge={steer_assist_publisher.last_raw_nudge_angle_deg:.2f}deg",
              f"nudge={steer_assist_publisher.last_nudge_angle_deg:.2f}deg",
              f"factor={command.speed_factor:.2f}",
              f"force_factor={command.force_factor:.2f}",
              f"target={command.target_position:.3f}",
              f"force={command.force:.2f}",
              f"friction={command.friction:.2f}",
            )
          ),
          flush=True,
        )

      _publish_state(g29_sock, state, events)
      frame += 1
  finally:
    if g29 is not None:
      g29.force_off()
      g29.stop()


def main() -> None:
  g29_sock = messaging.pub_sock("g29")
  steer_assist_sock = messaging.pub_sock("turboSteerAssist")

  while True:
    try:
      _run(g29_sock, steer_assist_sock)
    except KeyboardInterrupt:
      raise
    except Exception as e:
      print(f"g29d failed to open/read G29: {e}; retrying in {RETRY_DELAY:g}s", flush=True)
      time.sleep(RETRY_DELAY)


if __name__ == "__main__":
  main()
