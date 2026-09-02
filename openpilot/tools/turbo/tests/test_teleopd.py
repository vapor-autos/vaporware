from types import SimpleNamespace

from opendbc.can.packer import CANPacker

from openpilot.tools.turbo.teleopd import MAIN_BUS, button_event_can_msgs, teleop_command_can_msg


def test_l3_sends_cruise_enable():
  packer = CANPacker("turbo_rc_car")

  assert teleop_command_can_msg(packer, "cruiseEnable") == (0x205, b"\x01", MAIN_BUS)


def test_l2_sends_cruise_cancel():
  packer = CANPacker("turbo_rc_car")

  assert teleop_command_can_msg(packer, "cruiseCancel") == (0x205, b"\x00", MAIN_BUS)


def test_dpad_sends_headlight_commands():
  packer = CANPacker("turbo_rc_car")

  assert teleop_command_can_msg(packer, "headlightsOn") == (0x204, b"\x01\x00", MAIN_BUS)
  assert teleop_command_can_msg(packer, "headlightsOff") == (0x204, b"\x00\x00", MAIN_BUS)


def test_legacy_g29_button_fallback_remains_supported():
  packer = CANPacker("turbo_rc_car")
  g29 = SimpleNamespace(dpadUp=False, dpadDown=False, l2=False, l3=True)

  assert button_event_can_msgs(packer, g29) == [(0x205, b"\x01", MAIN_BUS)]
