"""Packed-button compatibility fixes for g29py 0.0.17.

Keep the existing HID reader/effect implementation. Byte 1 is a bitfield;
equality decoding loses L2 when a paddle is held. See g29py/docs/button-map.md.
"""
from g29py import G29


class TurboG29(G29):
  def apply_gamepad(self, state, val):
    for mask, name in ((0x10, "X"), (0x20, "S"), (0x40, "O"), (0x80, "T")):
      state["buttons"][name] = int(bool(val & mask))
    # The lower nibble is an eight-direction hat, not independent bits.
    hat = val & 0x0f
    for name, positions in (("up", (0, 1, 7)), ("right", (1, 2, 3)),
                            ("down", (3, 4, 5)), ("left", (5, 6, 7))):
      state["buttons"][name] = int(hat in positions)

  def apply_misc(self, state, val):
    for mask, name in ((1, "right_paddle"), (2, "left_paddle"), (4, "R2"), (8, "L2"),
                       (16, "Share"), (32, "Options"), (64, "R3"), (128, "L3")):
      state["buttons"][name] = int(bool(val & mask))
