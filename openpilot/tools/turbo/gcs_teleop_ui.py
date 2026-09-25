import os

from openpilot.tools.turbo.gcs_window import monitor_geometry, patch_undecorated_window, place_window


def main() -> None:
  monitor = monitor_geometry(os.getenv("TURBO_GCS_TELEOP_UI_MONITOR", "HDMI-1-0"))
  patch_undecorated_window("TURBO_GCS_TELEOP_UI_DECORATED")

  import openpilot.system.ui.lib.application as ui_application

  # Widgets bind this shared application at import time. A separate local app
  # leaves their font/texture caches uninitialized when the intent badge draws.
  if monitor is not None:
    ui_application.gui_app = ui_application.GuiApplication(monitor.width, monitor.height)

  from openpilot.tools.turbo.gcs_ui import GcsUi

  gui_app = ui_application.gui_app
  gui_app.init_window("Turbo GCS")
  place_window("Turbo GCS", monitor)

  gui_app.push_widget(GcsUi())
  placement_frames = 20
  for _ in gui_app.render():
    if placement_frames > 0:
      place_window("Turbo GCS", monitor)
      placement_frames -= 1


if __name__ == "__main__":
  main()
