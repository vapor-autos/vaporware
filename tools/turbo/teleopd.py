#!/usr/bin/env python3

import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.pandad import can_list_to_can_capnp


DBC_NAME = "turbo_rc_car"
MAIN_BUS = 1
MAX_THROTTLE_CMD = 10000
MAX_STEER_CMD = 18000
LOG_INTERVAL_FRAMES = 50


def clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def normalize_pedal(value: float) -> float:
  # g29py reports pedals in roughly [-1, 1], with released near -1.
  return (clip(value, -1.0, 1.0) + 1.0) / 2.0


def throttle_cmd(accelerator: float, reverse: float) -> int:
  accel = normalize_pedal(accelerator)
  brake = normalize_pedal(reverse)
  cmd = -brake if brake > 0.05 else accel
  return int(clip(cmd, -1.0, 1.0) * MAX_THROTTLE_CMD)


def steer_cmd(steering: float) -> int:
  return int(-clip(steering, -1.0, 1.0) * MAX_STEER_CMD)


def button_event_can_msgs(packer: CANPacker, g29) -> list[tuple[int, bytes, int]]:
  # These button fields are edge pulses generated from g29py.get_events(), not held button state.
  if g29.dpadUp:
    return [packer.make_can_msg("TOGGLE_HEADLIGHTS", MAIN_BUS, {"HEADLIGHTS_TOGGLE": 1})]
  if g29.dpadDown:
    return [packer.make_can_msg("TOGGLE_HEADLIGHTS", MAIN_BUS, {"HEADLIGHTS_TOGGLE": 0})]
  if g29.l3:
    return [packer.make_can_msg("CRUISE_ENABLE", MAIN_BUS, {"ENABLE": 1})]
  if g29.l2:
    return [packer.make_can_msg("CRUISE_ENABLE", MAIN_BUS, {"ENABLE": 0})]
  return []


def main() -> None:
  g29_sock = messaging.sub_sock("g29")
  sm = messaging.SubMaster(["carControl"])
  pm = messaging.PubMaster(["teleopSendCan"])
  packer = CANPacker(DBC_NAME)
  rk = Ratekeeper(50, print_delay_threshold=None)
  car_control_enabled = False

  while True:
    sm.update(0)
    if sm.updated["carControl"]:
      car_control_enabled = sm["carControl"].enabled

    for msg in messaging.drain_sock(g29_sock):
      if msg.which() != "g29":
        continue

      g29 = msg.g29
      can_msgs = [
        packer.make_can_msg("THROTTLE_CMD", MAIN_BUS, {"THROTTLE": throttle_cmd(g29.accelerator, g29.reverse)}),
      ]

      if not car_control_enabled:
        can_msgs.insert(0, packer.make_can_msg("STEER_CMD", MAIN_BUS, {"STEER_ANGLE": steer_cmd(g29.steering)}))

      can_msgs.extend(button_event_can_msgs(packer, g29))

      pm.send("teleopSendCan", can_list_to_can_capnp(can_msgs, msgtype="sendcan"))

      if rk.frame % LOG_INTERVAL_FRAMES == 0:
        addrs = ", ".join(f"0x{addr:x}" for addr, _, _ in can_msgs)
        print(f"teleopd g29 steer={g29.steering:.3f} accel={g29.accelerator:.3f} reverse={g29.reverse:.3f} "
              f"car_control_enabled={car_control_enabled} can=[{addrs}]", flush=True)

    rk.keep_time()


if __name__ == "__main__":
  main()
