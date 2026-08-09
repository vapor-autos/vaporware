import os
import time

from msgq.visionipc import VisionStreamType

from openpilot.cereal import messaging
from openpilot.common.hardware import TICI
from openpilot.common.realtime import Priority, config_realtime_process, set_core_affinity
from openpilot.tools.turbo.gcs_window import monitor_geometry, patch_undecorated_window, place_window


GCS_STANDARD_UI_CAMERA_ENV = "TURBO_GCS_STANDARD_UI_CAMERA"

_CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def _standard_ui_camera() -> VisionStreamType:
  camera = os.getenv(GCS_STANDARD_UI_CAMERA_ENV, "wideRoad").strip()
  if camera not in _CAMERA_STREAMS:
    valid = ",".join(_CAMERA_STREAMS)
    raise ValueError(f"invalid {GCS_STANDARD_UI_CAMERA_ENV}: {camera}; expected one of {valid}")
  return _CAMERA_STREAMS[camera]


def main() -> None:
  os.environ.setdefault("BIG", "1")

  monitor = monitor_geometry(os.getenv("TURBO_GCS_DEBUG_UI_MONITOR", "eDP-1"))
  patch_undecorated_window("TURBO_GCS_DEBUG_UI_DECORATED")
  onroad_stream = _standard_ui_camera()

  import openpilot.system.ui.lib.application as ui_application

  if monitor is not None:
    ui_application.gui_app = ui_application.GuiApplication(monitor.width, monitor.height)

  from openpilot.selfdrive.ui.layouts.main import MainLayout
  from openpilot.selfdrive.ui.mici.layouts.main import MiciMainLayout
  from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
  from openpilot.selfdrive.ui.ui_state import ui_state

  gui_app = ui_application.gui_app

  cores = {5, }
  config_realtime_process(0, Priority.CTRL_HIGH)

  gui_app.init_window("UI")
  place_window("UI", monitor)

  if gui_app.big_ui():
    MainLayout(onroad_layout=AugmentedRoadView(stream_type=onroad_stream, auto_switch_stream=False))
  else:
    MiciMainLayout()

  pm = messaging.PubMaster(['uiDebug'])
  placement_frames = 20
  for should_render, frame_time, cpu_time in gui_app.render():
    if placement_frames > 0:
      place_window("UI", monitor)
      placement_frames -= 1

    extra_start = time.monotonic()
    ui_state.update()

    if should_render:
      if TICI and os.sched_getaffinity(0) != cores:
        try:
          set_core_affinity(list(cores))
        except OSError:
          pass

      extra_cpu = time.monotonic() - extra_start
      msg = messaging.new_message('uiDebug')
      msg.uiDebug.cpuTimeMillis = (cpu_time + extra_cpu) * 1000
      msg.uiDebug.frameTimeMillis = frame_time * 1000
      pm.send('uiDebug', msg)


if __name__ == "__main__":
  main()
