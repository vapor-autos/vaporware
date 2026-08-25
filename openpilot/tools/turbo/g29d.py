#!/usr/bin/env python3
import time

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.turbo_steer_assist import (
  DEFAULT_MAX_NUDGE_ANGLE_DEG,
  DEFAULT_NUDGE_DEADBAND_DEG,
  DEFAULT_NUDGE_GAIN_DEG,
  compute_nudge_angle_deg,
  steering_angle_to_g29_target,
)

RETRY_DELAY = 2.0
PUBLISH_INTERVAL = 0.02
ASSIST_PUBLISH_INTERVAL = 0.05
LOG_INTERVAL_FRAMES = 50

TORQUE_SIM_MAX_VELOCITY_M_S = 20.0
TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S = 8.0
TORQUE_SIM_CARSTATE_STALE_S = 0.25
TORQUE_SIM_ASSIST_STALE_S = 0.25


def _clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def _accelerator_pedal(accelerator: float) -> float:
  return (_clip(accelerator, -1.0, 1.0) + 1.0) / 2.0


def _accelerator_to_simulated_velocity_m_s(accelerator: float, max_velocity_m_s: float) -> float:
  return _accelerator_pedal(accelerator) * max(0.0, max_velocity_m_s)


def _steering_angle_to_g29_target(steering_angle_deg: float) -> float:
  return steering_angle_to_g29_target(steering_angle_deg)


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
    self.sm = messaging.SubMaster(["carOutput", "selfdriveState"]) if sm is None else sm
    self.stale_timeout_s = stale_timeout_s
    self.last_caroutput_age_s: float | None = None
    self.last_selfdrive_age_s: float | None = None
    self.last_target_angle_deg: float | None = None

  def update(self, now: float | None = None) -> tuple[float | None, str]:
    self.sm.update(0)
    now = time.monotonic() if now is None else now
    self.last_caroutput_age_s = self._age("carOutput", now)
    self.last_selfdrive_age_s = self._age("selfdriveState", now)
    self.last_target_angle_deg = None

    if not self._fresh("selfdriveState"):
      return None, "selfdriveState_stale"

    selfdrive_state = self.sm["selfdriveState"]
    if not (bool(selfdrive_state.enabled) or bool(selfdrive_state.active)):
      return None, "disengaged"

    if not self._fresh("carOutput"):
      return None, "carOutput_stale"

    angle_deg = float(self.sm["carOutput"].actuatorsOutput.steeringAngleDeg)
    self.last_target_angle_deg = angle_deg
    return _steering_angle_to_g29_target(angle_deg), "carOutput"

  def _age(self, service: str, now: float) -> float | None:
    return now - self.sm.recv_time[service] if self.sm.seen[service] else None

  def _fresh(self, service: str) -> bool:
    age = self.last_selfdrive_age_s if service == "selfdriveState" else self.last_caroutput_age_s
    return self.sm.seen[service] and self.sm.valid[service] and age is not None and age <= self.stale_timeout_s


def _dial_delta(events: list[dict]) -> int:
  return sum(int(event.get("delta", 0)) for event in events if event.get("type") == "dial")


def _button_down_events(events: list[dict]) -> set[str]:
  return {event["control"] for event in events if event.get("type") == "button_down" and "control" in event}


def _publish_state(sock, state: dict, events: list[dict]) -> None:
  buttons = state["buttons"]
  button_down = _button_down_events(events)

  msg = messaging.new_message("g29")
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
    publish_interval_s: float = ASSIST_PUBLISH_INTERVAL,
    gain_deg: float = DEFAULT_NUDGE_GAIN_DEG,
    max_nudge_angle_deg: float = DEFAULT_MAX_NUDGE_ANGLE_DEG,
    deadband_deg: float = DEFAULT_NUDGE_DEADBAND_DEG,
  ):
    self.sock = sock
    self.publish_interval_s = publish_interval_s
    self.gain_deg = gain_deg
    self.max_nudge_angle_deg = max_nudge_angle_deg
    self.deadband_deg = deadband_deg
    self.last_publish_time = 0.0
    self.last_active = False
    self.last_wheel_delta = 0.0
    self.last_nudge_angle_deg = 0.0

  def update(
    self,
    state: dict,
    target_steering: float | None,
    target_steering_angle_deg: float | None,
    now: float | None = None,
  ) -> bool:
    now = time.monotonic() if now is None else now
    if now - self.last_publish_time < self.publish_interval_s:
      return False

    wheel_steering = float(state["steering"])
    active = target_steering is not None and target_steering_angle_deg is not None
    target = 0.0 if target_steering is None else float(target_steering)
    self.last_active = active
    self.last_wheel_delta = wheel_steering - target
    self.last_nudge_angle_deg = compute_nudge_angle_deg(
      wheel_steering,
      target,
      gain_deg=self.gain_deg,
      max_nudge_angle_deg=self.max_nudge_angle_deg,
      deadband_deg=self.deadband_deg,
    ) if active else 0.0

    msg = messaging.new_message("turboSteerAssist")
    msg.turboSteerAssist.active = active
    msg.turboSteerAssist.nudgeAngleDeg = self.last_nudge_angle_deg
    msg.turboSteerAssist.wheelSteering = wheel_steering
    msg.turboSteerAssist.targetSteering = target
    msg.turboSteerAssist.targetSteeringAngleDeg = 0.0 if target_steering_angle_deg is None else float(target_steering_angle_deg)
    self.sock.send(msg.to_bytes())
    self.last_publish_time = now
    return True


def _make_torque_controller(g29):
  from g29py.advanced import SteeringTorqueConfig, SteeringTorqueController

  config = SteeringTorqueConfig(
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
    speed_source = SpeedSource()
    assist_target_source = AssistTargetSource()
    steer_assist_publisher = SteerAssistNudgePublisher(steer_assist_sock)
    g29.listen()

    print(
      " ".join((
        "g29d torque_sim enabled",
        "speed_source=carState",
        "assist_target=carOutput",
        "pedal_fallback=True",
        f"carstate_stale={TORQUE_SIM_CARSTATE_STALE_S:.2f}s",
        f"assist_stale={TORQUE_SIM_ASSIST_STALE_S:.2f}s",
        f"steer_assist_nudge_max={DEFAULT_MAX_NUDGE_ANGLE_DEG:.1f}deg",
        f"steer_assist_gain={DEFAULT_NUDGE_GAIN_DEG:.1f}deg",
        f"max_velocity={TORQUE_SIM_MAX_VELOCITY_M_S:.1f}m/s",
        f"force_response={TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S:.1f}m/s",
      )),
      flush=True,
    )

    frame = 0
    while True:
      time.sleep(PUBLISH_INTERVAL)
      state = g29.get_state()
      events = g29.get_events()

      velocity, speed_source_name = speed_source.update(state)
      target_steering, assist_target_name = assist_target_source.update()
      steer_assist_publisher.update(state, target_steering, assist_target_source.last_target_angle_deg)
      command = torque_controller.update(
        longitudinal_velocity_m_s=velocity,
        steering=state["steering"],
        target_steering=target_steering,
      )
      if frame % LOG_INTERVAL_FRAMES == 0:
        carstate_age = speed_source.last_carstate_age_s
        carstate_age_text = "none" if carstate_age is None else f"{carstate_age:.3f}s"
        caroutput_age = assist_target_source.last_caroutput_age_s
        caroutput_age_text = "none" if caroutput_age is None else f"{caroutput_age:.3f}s"
        selfdrive_age = assist_target_source.last_selfdrive_age_s
        selfdrive_age_text = "none" if selfdrive_age is None else f"{selfdrive_age:.3f}s"
        target_angle = assist_target_source.last_target_angle_deg
        target_angle_text = "none" if target_angle is None else f"{target_angle:.2f}deg"
        target_steering_text = "none" if target_steering is None else f"{target_steering:.3f}"
        print(
          " ".join((
            "g29d torque_sim",
            f"speed_source={speed_source_name}",
            f"assist_target={assist_target_name}",
            f"velocity={velocity:.2f}m/s",
            f"carstate_age={carstate_age_text}",
            f"caroutput_age={caroutput_age_text}",
            f"selfdrive_age={selfdrive_age_text}",
            f"target_angle={target_angle_text}",
            f"target_steering={target_steering_text}",
            f"nudge={steer_assist_publisher.last_nudge_angle_deg:.2f}deg",
            f"factor={command.speed_factor:.2f}",
            f"force_factor={command.force_factor:.2f}",
            f"target={command.target_position:.3f}",
            f"force={command.force:.2f}",
            f"friction={command.friction:.2f}",
          )),
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
