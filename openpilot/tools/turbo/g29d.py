#!/usr/bin/env python3
import time

import openpilot.cereal.messaging as messaging

RETRY_DELAY = 2.0
PUBLISH_INTERVAL = 0.02
LOG_INTERVAL_FRAMES = 50

TORQUE_SIM_MAX_VELOCITY_M_S = 20.0
TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S = 8.0
TORQUE_SIM_CARSTATE_STALE_S = 0.25


def _clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def _accelerator_pedal(accelerator: float) -> float:
  return (_clip(accelerator, -1.0, 1.0) + 1.0) / 2.0


def _accelerator_to_simulated_velocity_m_s(accelerator: float, max_velocity_m_s: float) -> float:
  return _accelerator_pedal(accelerator) * max(0.0, max_velocity_m_s)


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


def _make_torque_controller(g29):
  from g29py.advanced import SteeringTorqueConfig, SteeringTorqueController

  config = SteeringTorqueConfig(
    force_response_velocity_m_s=TORQUE_SIM_FORCE_RESPONSE_VELOCITY_M_S,
  )
  return SteeringTorqueController(g29, config=config)


def _run(sock) -> None:
  from g29py import G29

  g29 = None
  try:
    g29 = G29()
    g29.set_range(400)
    torque_controller = _make_torque_controller(g29)
    speed_source = SpeedSource()
    g29.listen()

    print(
      " ".join((
        "g29d torque_sim enabled",
        "speed_source=carState",
        "pedal_fallback=True",
        f"carstate_stale={TORQUE_SIM_CARSTATE_STALE_S:.2f}s",
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
      command = torque_controller.update(longitudinal_velocity_m_s=velocity, steering=state["steering"])
      if frame % LOG_INTERVAL_FRAMES == 0:
        carstate_age = speed_source.last_carstate_age_s
        carstate_age_text = "none" if carstate_age is None else f"{carstate_age:.3f}s"
        print(
          " ".join((
            "g29d torque_sim",
            f"speed_source={speed_source_name}",
            f"velocity={velocity:.2f}m/s",
            f"carstate_age={carstate_age_text}",
            f"factor={command.speed_factor:.2f}",
            f"force_factor={command.force_factor:.2f}",
            f"target={command.target_position:.3f}",
            f"force={command.force:.2f}",
            f"friction={command.friction:.2f}",
          )),
          flush=True,
        )

      _publish_state(sock, state, events)
      frame += 1
  finally:
    if g29 is not None:
      g29.force_off()
      g29.stop()


def main() -> None:
  sock = messaging.pub_sock("g29")

  while True:
    try:
      _run(sock)
    except KeyboardInterrupt:
      raise
    except Exception as e:
      print(f"g29d failed to open/read G29: {e}; retrying in {RETRY_DELAY:g}s", flush=True)
      time.sleep(RETRY_DELAY)


if __name__ == "__main__":
  main()
