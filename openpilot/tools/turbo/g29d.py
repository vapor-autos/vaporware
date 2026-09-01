#!/usr/bin/env python3
from dataclasses import dataclass
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.controls.lib.turbo_steer_assist import g29_steering_to_angle_deg, steering_angle_to_g29_target
from openpilot.tools.turbo.steer_assist import SteerAssistConfig, SteerAssistController, SteerAssistDecision, SteerAssistInput
from openpilot.tools.turbo.teleop_metrics import default_latest_json_path, write_metrics_payload

RETRY_DELAY = 2.0
PUBLISH_RATE_HZ = 50
STEER_ASSIST_METRICS_INTERVAL_FRAMES = 5
LOG_INTERVAL_FRAMES = 50

TORQUE_SIM_MAX_VELOCITY_M_S = 20.0
TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S = 8.0
TORQUE_SIM_CARSTATE_STALE_S = 0.25
TORQUE_SIM_ASSIST_STALE_S = 0.25
TORQUE_SIM_ASSIST_STALE_GRACE_S = 0.15
STEER_ASSIST_CONFIG = SteerAssistConfig()
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


@dataclass(frozen=True)
class AssistFeedback:
  model_angle_deg: float | None
  model_log_mono_time: int
  fresh: bool
  engaged: bool
  source: str
  controlsstate_age_s: float | None
  caroutput_age_s: float | None
  selfdrive_age_s: float | None
  applied_angle_deg: float | None


class AssistTargetSource:
  def __init__(
    self,
    sm: messaging.SubMaster | None = None,
    stale_timeout_s: float = TORQUE_SIM_ASSIST_STALE_S,
    stale_grace_s: float = TORQUE_SIM_ASSIST_STALE_GRACE_S,
  ):
    self.sm = messaging.SubMaster(["controlsState", "selfdriveState", "carOutput"]) if sm is None else sm
    self.stale_timeout_s = stale_timeout_s
    self.stale_grace_s = max(0.0, stale_grace_s)
    self._cached_target_angle_deg: float | None = None
    self._cached_target_log_mono_time = 0

  def update(self, now: float | None = None) -> AssistFeedback:
    self.sm.update(0)
    now = time.monotonic() if now is None else now
    ages = {service: self._age(service, now) for service in ("controlsState", "carOutput", "selfdriveState")}
    applied_angle_deg = self._applied_angle_deg()
    engaged = self._engaged()

    if not self._fresh("selfdriveState", ages["selfdriveState"]):
      return self._stale_feedback("selfdriveState_stale", ages, applied_angle_deg, engaged)

    if not engaged:
      self._clear_cached_target()
      return self._feedback("disengaged", ages, applied_angle_deg, engaged)

    if not self._fresh("controlsState", ages["controlsState"]):
      return self._stale_feedback("controlsState_stale", ages, applied_angle_deg, engaged)

    controls_state = self.sm["controlsState"]
    lateral_state = controls_state.lateralControlState
    if lateral_state.which() != "angleState":
      self._clear_cached_target()
      return self._feedback("controlsState_not_angle", ages, applied_angle_deg, engaged)

    angle_deg = float(lateral_state.angleState.steeringAngleDesiredDeg)
    model_log_mono_time = int(self.sm.logMonoTime["controlsState"])
    self._cached_target_angle_deg = angle_deg
    self._cached_target_log_mono_time = model_log_mono_time
    return self._feedback(
      "controlsState",
      ages,
      applied_angle_deg,
      engaged,
      model_angle_deg=angle_deg,
      model_log_mono_time=model_log_mono_time,
      fresh=True,
    )

  def _clear_cached_target(self) -> None:
    self._cached_target_angle_deg = None
    self._cached_target_log_mono_time = 0

  def _stale_feedback(
    self,
    reason: str,
    ages: dict[str, float | None],
    applied_angle_deg: float | None,
    engaged: bool,
  ) -> AssistFeedback:
    services = ("selfdriveState", "controlsState")
    stale_limit_s = self.stale_timeout_s + self.stale_grace_s
    valid_feedback = all(self.sm.seen[service] and self.sm.valid[service] for service in services)
    within_grace = all(ages[service] is not None and ages[service] <= stale_limit_s for service in services)
    if self._cached_target_angle_deg is None or not valid_feedback or not within_grace or not engaged:
      self._clear_cached_target()
      return self._feedback(reason, ages, applied_angle_deg, engaged)

    return self._feedback(
      "feedback_stale_hold",
      ages,
      applied_angle_deg,
      engaged,
      model_angle_deg=self._cached_target_angle_deg,
      model_log_mono_time=self._cached_target_log_mono_time,
    )

  @staticmethod
  def _feedback(
    source: str,
    ages: dict[str, float | None],
    applied_angle_deg: float | None,
    engaged: bool,
    model_angle_deg: float | None = None,
    model_log_mono_time: int = 0,
    fresh: bool = False,
  ) -> AssistFeedback:
    return AssistFeedback(
      model_angle_deg=model_angle_deg,
      model_log_mono_time=model_log_mono_time,
      fresh=fresh,
      engaged=engaged,
      source=source,
      controlsstate_age_s=ages["controlsState"],
      caroutput_age_s=ages["carOutput"],
      selfdrive_age_s=ages["selfdriveState"],
      applied_angle_deg=applied_angle_deg,
    )

  def _age(self, service: str, now: float) -> float | None:
    return now - self.sm.recv_time[service] if self.sm.seen[service] else None

  def _fresh(self, service: str, age: float | None) -> bool:
    return self.sm.seen[service] and self.sm.valid[service] and age is not None and age <= self.stale_timeout_s

  def _engaged(self) -> bool:
    if not self.sm.seen["selfdriveState"] or not self.sm.valid["selfdriveState"]:
      return False
    selfdrive_state = self.sm["selfdriveState"]
    return bool(selfdrive_state.enabled) or bool(selfdrive_state.active)

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
  def __init__(self, sock, config: SteerAssistConfig = STEER_ASSIST_CONFIG):
    self.sock = sock
    self.controller = SteerAssistController(config)
    self.sequence = 0
    self.last_decision: SteerAssistDecision | None = None

  def update(
    self,
    state: dict,
    target_steering: float | None,
    target_steering_angle_deg: float | None,
    haptic_target_angle_deg: float,
    base_target_log_mono_time: int = 0,
    input_fresh: bool = True,
    now: float | None = None,
  ) -> SteerAssistDecision:
    now = time.monotonic() if now is None else now
    wheel_steering = float(state["steering"])
    model_target_angle_deg = target_steering_angle_deg if target_steering is not None else None
    decision = self.controller.update(
      SteerAssistInput(
        wheel_angle_deg=g29_steering_to_angle_deg(wheel_steering),
        model_target_angle_deg=model_target_angle_deg,
        haptic_target_angle_deg=haptic_target_angle_deg,
        base_target_log_mono_time=base_target_log_mono_time,
        fresh=input_fresh,
        now=now,
      )
    )
    self.last_decision = decision

    msg = messaging.new_message("turboSteerAssist")
    msg.valid = True
    msg.turboSteerAssist.active = decision.active
    msg.turboSteerAssist.requestedSteeringAngleDeg = decision.requested_steering_angle_deg
    msg.turboSteerAssist.wheelSteeringAngleDeg = decision.wheel_angle_deg
    msg.turboSteerAssist.baseModelSteeringAngleDeg = 0.0 if decision.model_target_angle_deg is None else decision.model_target_angle_deg
    self.sequence = (self.sequence + 1) & 0xFFFFFFFF
    if self.sequence == 0:
      self.sequence = 1
    msg.turboSteerAssist.sequence = self.sequence
    msg.turboSteerAssist.baseModelLogMonoTime = decision.base_target_log_mono_time
    self.sock.send(msg.to_bytes())
    return decision


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
          f"publish_rate={PUBLISH_RATE_HZ}Hz",
          f"metrics_rate={PUBLISH_RATE_HZ / STEER_ASSIST_METRICS_INTERVAL_FRAMES:g}Hz",
          "speed_source=carState",
          "assist_target=controlsState",
          "pedal_fallback=True",
          f"carstate_stale={TORQUE_SIM_CARSTATE_STALE_S:.2f}s",
          f"assist_stale={TORQUE_SIM_ASSIST_STALE_S:.2f}s",
          f"assist_stale_grace={TORQUE_SIM_ASSIST_STALE_GRACE_S:.2f}s",
          f"steer_assist_inner_deadband={STEER_ASSIST_CONFIG.inner_deadband_deg:.1f}deg",
          f"steer_assist_full_error={STEER_ASSIST_CONFIG.full_assist_error_deg:.1f}deg",
          f"steer_assist_tracking_error={STEER_ASSIST_CONFIG.tracking_error_deg:.1f}deg",
          f"steer_assist_tracking_duration={STEER_ASSIST_CONFIG.tracking_duration_s:.2f}s",
          f"steer_assist_candidate_duration={STEER_ASSIST_CONFIG.candidate_duration_s:.2f}s",
          f"steer_assist_min_opposing_velocity={STEER_ASSIST_CONFIG.min_opposing_velocity_deg_s:.0f}deg/s",
          f"steer_assist_max_candidate_target_rate={STEER_ASSIST_CONFIG.max_candidate_target_rate_deg_s:.0f}deg/s",
          f"steer_assist_haptic_max_rate={STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S:.0f}deg/s",
          f"steer_assist_override_slew_rate={STEER_ASSIST_CONFIG.override_slew_rate_deg_s:.0f}deg/s",
          f"steer_assist_release_duration={STEER_ASSIST_CONFIG.release_duration_s:.2f}s",
          f"steer_assist_max_release_relative_velocity={STEER_ASSIST_CONFIG.max_release_relative_velocity_deg_s:.0f}deg/s",
          f"steer_assist_haptic_force={STEER_ASSIST_HAPTIC_FORCE:.2f}",
          f"steer_assist_haptic_friction={STEER_ASSIST_HAPTIC_FRICTION:.2f}",
          f"max_velocity={TORQUE_SIM_MAX_VELOCITY_M_S:.1f}m/s",
          f"force_response={TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S:.1f}m/s",
        )
      ),
      flush=True,
    )

    frame = 0
    rk = Ratekeeper(PUBLISH_RATE_HZ, print_delay_threshold=None)
    while True:
      now = time.monotonic()
      state = g29.get_state()
      events = g29.get_events()

      velocity, speed_source_name = speed_source.update(state, now=now)
      assist_feedback = assist_target_source.update(now=now)
      target_angle = assist_feedback.model_angle_deg
      target_steering = None if target_angle is None else _steering_angle_to_g29_target(target_angle)
      wheel_angle_deg = g29_steering_to_angle_deg(float(state["steering"]))
      limited_target_angle_deg = haptic_target_limiter.update(
        target_angle,
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
      assist_decision = steer_assist_publisher.update(
        state,
        target_steering,
        target_angle,
        haptic_target_angle_deg,
        base_target_log_mono_time=assist_feedback.model_log_mono_time,
        input_fresh=assist_feedback.fresh,
        now=now,
      )

      carstate_age = speed_source.last_carstate_age_s
      controlsstate_age = assist_feedback.controlsstate_age_s
      caroutput_age = assist_feedback.caroutput_age_s
      selfdrive_age = assist_feedback.selfdrive_age_s
      applied_angle = assist_feedback.applied_angle_deg

      if frame % STEER_ASSIST_METRICS_INTERVAL_FRAMES == 0:
        write_metrics_payload(
          {
            "steer_assist": {
              "requested_active": assist_decision.requested_active,
              "engaged": assist_feedback.engaged,
              "target_source": assist_feedback.source,
              "target_fresh": assist_feedback.fresh,
              "stale_grace_s": TORQUE_SIM_ASSIST_STALE_GRACE_S,
              "active": assist_decision.active,
              "tracking_status": assist_decision.tracking_status,
              "tracking_error_deg": STEER_ASSIST_CONFIG.tracking_error_deg,
              "tracking_duration_s": STEER_ASSIST_CONFIG.tracking_duration_s,
              "min_opposing_velocity_deg_s": STEER_ASSIST_CONFIG.min_opposing_velocity_deg_s,
              "candidate_duration_s": STEER_ASSIST_CONFIG.candidate_duration_s,
              "candidate_evidence_s": assist_decision.candidate_evidence_s,
              "max_candidate_target_rate_deg_s": STEER_ASSIST_CONFIG.max_candidate_target_rate_deg_s,
              "max_target_step_deg": STEER_ASSIST_CONFIG.max_target_step_deg,
              "max_target_rate_deg_s": STEER_ASSIST_CONFIG.max_target_rate_deg_s,
              "haptic_max_rate_deg_s": STEER_ASSIST_HAPTIC_MAX_RATE_DEG_S,
              "override_slew_rate_deg_s": STEER_ASSIST_CONFIG.override_slew_rate_deg_s,
              "override_slewing": assist_decision.override_slewing,
              "release_duration_s": STEER_ASSIST_CONFIG.release_duration_s,
              "max_release_relative_velocity_deg_s": STEER_ASSIST_CONFIG.max_release_relative_velocity_deg_s,
              "release_pending": assist_decision.release_since is not None,
              "release_evidence_s": assist_decision.release_evidence_s,
              "target_step_deg": assist_decision.target_step_deg,
              "target_rate_deg_s": assist_decision.target_rate_deg_s,
              "target_interval_s": assist_decision.target_interval_s,
              "haptic_target_rate_deg_s": assist_decision.haptic_target_rate_deg_s,
              "wheel_velocity_deg_s": assist_decision.wheel_velocity_deg_s,
              "relative_velocity_deg_s": assist_decision.relative_velocity_deg_s,
              "velocity_m_s": velocity,
              "carstate_age_s": carstate_age,
              "controlsstate_age_s": controlsstate_age,
              "caroutput_age_s": caroutput_age,
              "selfdrive_age_s": selfdrive_age,
              "model_target_angle_deg": target_angle,
              "base_target_log_mono_time": assist_feedback.model_log_mono_time,
              "applied_angle_deg": applied_angle,
              "haptic_target_angle_deg": haptic_target_angle_deg,
              "wheel_angle_deg": assist_decision.wheel_angle_deg,
              "model_error_deg": assist_decision.model_error_deg,
              "residual_deg": assist_decision.residual_angle_deg,
              "model_haptic_delta_deg": assist_decision.model_haptic_delta_deg,
              "haptic_nudge_deg": assist_decision.haptic_nudge_angle_deg,
              "desired_blended_target_angle_deg": assist_decision.desired_blended_target_angle_deg,
              "blended_target_angle_deg": assist_decision.blended_target_angle_deg,
              "raw_nudge_deg": assist_decision.raw_nudge_angle_deg,
              "nudge_deg": assist_decision.nudge_angle_deg,
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
              f"assist_target={assist_feedback.source}",
              f"velocity={velocity:.2f}m/s",
              f"carstate_age={carstate_age_text}",
              f"controlsstate_age={controlsstate_age_text}",
              f"caroutput_age={caroutput_age_text}",
              f"selfdrive_age={selfdrive_age_text}",
              f"target_angle={target_angle_text}",
              f"applied_angle={applied_angle_text}",
              f"target_steering={target_steering_text}",
              f"tracking={assist_decision.tracking_status}",
              f"haptic_target={haptic_target_angle_deg:.2f}deg",
              f"wheel_angle={assist_decision.wheel_angle_deg:.2f}deg",
              f"model_error={assist_decision.model_error_deg:.2f}deg",
              f"residual={assist_decision.residual_angle_deg:.2f}deg",
              f"model_haptic_delta={assist_decision.model_haptic_delta_deg:.2f}deg",
              f"wheel_velocity={assist_decision.wheel_velocity_deg_s:.2f}deg/s",
              f"relative_velocity={assist_decision.relative_velocity_deg_s:.2f}deg/s",
              f"haptic_nudge={assist_decision.haptic_nudge_angle_deg:.2f}deg",
              f"desired_blended_target={assist_decision.desired_blended_target_angle_deg:.2f}deg",
              f"blended_target={assist_decision.blended_target_angle_deg:.2f}deg",
              f"override_slewing={assist_decision.override_slewing}",
              f"release_pending={assist_decision.release_since is not None}",
              f"release_evidence={assist_decision.release_evidence_s:.2f}s",
              f"nudge={assist_decision.nudge_angle_deg:.2f}deg",
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
      rk.keep_time()
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
