"""Narrow compatibility fix for g29py 0.0.17's byte-1 button decoder.

Keep the existing HID reader/effect implementation. Byte 1 is a bitfield;
equality decoding loses L2 when a paddle is held. See g29py/docs/button-map.md.
"""
from g29py import G29


class TurboG29(G29):
  def apply_misc(self, state, val):
    for mask, name in ((1, "right_paddle"), (2, "left_paddle"), (4, "R2"), (8, "L2"),
                       (16, "Share"), (32, "Options"), (64, "R3"), (128, "L3")):
      state["buttons"][name] = int(bool(val & mask))
