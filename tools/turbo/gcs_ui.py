#!/usr/bin/env python3
import os

import pyray as rl

from cereal import messaging
from msgq.visionipc import VisionStreamType
from openpilot.system.ui.lib import application as ui_app
from openpilot.system.ui.widgets import Widget


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


ui_app.gui_app = ui_app.GuiApplication(*_gcs_window_size())
gui_app = ui_app.gui_app

from openpilot.selfdrive.ui.onroad.cameraview import CameraView


class GcsUi(Widget):
  def __init__(self) -> None:
    super().__init__()
    self._sm = messaging.SubMaster(["g29"])
    self._wide = self._child(CameraView("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD))
    self._driver = self._child(CameraView("camerad", VisionStreamType.VISION_STREAM_DRIVER))
    self._overlay_w = 482
    self._overlay_h = 302
    self._dial = 0

    self._wide._set_placeholder_color(rl.Color(18, 24, 28, 255))
    self._driver._set_placeholder_color(rl.Color(34, 40, 46, 255))

  def _update_state(self) -> None:
    self._sm.update(0)
    if not self._sm.updated["g29"]:
      return

    g29 = self._sm["g29"]
    self._dial = g29.dial

    if self._dial > 0:
      self._overlay_w = 482 + self._dial * 20
      self._overlay_h = 302 + self._dial * 12

  def _render(self, rect: rl.Rectangle) -> None:
    self._wide.render(rect)

    overlay = self._overlay_rect(rect)
    self._driver.render(overlay)

  def _overlay_rect(self, rect: rl.Rectangle) -> rl.Rectangle:
    margin = 20
    w = min(self._overlay_w, int(rect.width - margin * 2))
    h = min(self._overlay_h, int(rect.height - margin * 2))
    return rl.Rectangle(rect.x + rect.width - w - margin, rect.y + rect.height - h - margin, w, h)


def main() -> None:
  gui_app.init_window("Turbo GCS")
  gui_app.push_widget(GcsUi())
  for _ in gui_app.render():
    pass


if __name__ == "__main__":
  main()
