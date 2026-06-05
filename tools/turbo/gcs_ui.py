#!/usr/bin/env python3
import os

import pyray as rl

from cereal import messaging
from msgq.visionipc import VisionStreamType
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import GuiApplication
from openpilot.system.ui.widgets import Widget


OVERLAY_BASE_W = 482
OVERLAY_BASE_H = 302
OVERLAY_MARGIN = 20
OVERLAY_MIN_SCALE = 0.5
OVERLAY_MAX_SCREEN_FRACTION = 0.65
OVERLAY_DIAL_STEP = 0.08


def _clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def _overlay_scale_bounds(rect: rl.Rectangle) -> tuple[float, float]:
  max_w = max(1, int((rect.width - OVERLAY_MARGIN * 2) * OVERLAY_MAX_SCREEN_FRACTION))
  max_h = max(1, int((rect.height - OVERLAY_MARGIN * 2) * OVERLAY_MAX_SCREEN_FRACTION))
  max_scale = min(max_w / OVERLAY_BASE_W, max_h / OVERLAY_BASE_H)
  return OVERLAY_MIN_SCALE, max(OVERLAY_MIN_SCALE, max_scale)


def _overlay_size(rect: rl.Rectangle, scale: float) -> tuple[int, int]:
  effective_scale = _clip(scale, *_overlay_scale_bounds(rect))
  return int(OVERLAY_BASE_W * effective_scale), int(OVERLAY_BASE_H * effective_scale)


def _gcs_window_size() -> tuple[int, int]:
  width = os.getenv("TURBO_GCS_WIDTH")
  height = os.getenv("TURBO_GCS_HEIGHT")
  if width is not None and height is not None:
    return int(width), int(height)

  rl.init_window(1, 1, "")
  monitor = rl.get_current_monitor()
  size = (rl.get_monitor_width(monitor), rl.get_monitor_height(monitor))
  rl.close_window()
  return size


class GcsUi(Widget):
  def __init__(self) -> None:
    super().__init__()
    self._sm = messaging.SubMaster(["g29"])
    self._wide = self._child(CameraView("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD))
    self._driver = self._child(CameraView("camerad", VisionStreamType.VISION_STREAM_DRIVER))
    self._overlay_scale = 1.0

    self._wide._set_placeholder_color(rl.Color(18, 24, 28, 255))
    self._driver._set_placeholder_color(rl.Color(34, 40, 46, 255))

  def _update_state(self) -> None:
    self._sm.update(0)
    if not self._sm.updated["g29"]:
      return

    g29 = self._sm["g29"]
    if g29.dial != 0:
      self._overlay_scale += g29.dial * OVERLAY_DIAL_STEP
      self._overlay_scale = max(self._overlay_scale, OVERLAY_MIN_SCALE)

  def _render(self, rect: rl.Rectangle) -> None:
    self._wide.render(rect)

    overlay = self._overlay_rect(rect)
    self._driver.render(overlay)

  def _overlay_rect(self, rect: rl.Rectangle) -> rl.Rectangle:
    self._overlay_scale = _clip(self._overlay_scale, *_overlay_scale_bounds(rect))
    w, h = _overlay_size(rect, self._overlay_scale)
    return rl.Rectangle(rect.x + rect.width - w - OVERLAY_MARGIN,
                        rect.y + rect.height - h - OVERLAY_MARGIN, w, h)


def main() -> None:
  gui_app = GuiApplication(*_gcs_window_size())
  gui_app.init_window("Turbo GCS")
  gui_app.push_widget(GcsUi())
  for _ in gui_app.render():
    pass


if __name__ == "__main__":
  main()
