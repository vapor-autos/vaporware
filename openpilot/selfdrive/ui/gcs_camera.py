import os

from msgq.visionipc import VisionStreamType

from openpilot.common.params import Params


GCS_STANDARD_UI_CAMERA_ENV = "TURBO_GCS_STANDARD_UI_CAMERA"

_CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def gcs_standard_ui_camera(
  default_stream: VisionStreamType = VisionStreamType.VISION_STREAM_ROAD,
  params: Params | None = None,
) -> tuple[VisionStreamType, bool]:
  params = Params() if params is None else params
  camera = os.getenv(GCS_STANDARD_UI_CAMERA_ENV, "").strip()
  if not params.get_bool("GCS") or not camera:
    return default_stream, False

  if camera not in _CAMERA_STREAMS:
    valid = ",".join(_CAMERA_STREAMS)
    raise ValueError(f"invalid {GCS_STANDARD_UI_CAMERA_ENV}: {camera}; expected one of {valid}")

  return _CAMERA_STREAMS[camera], True
