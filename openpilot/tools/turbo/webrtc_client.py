import asyncio

import aiortc
import requests

from msgq.visionipc import VisionStreamType
from teleoprtc import WebRTCOfferBuilder, StreamingOffer


CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "driver": VisionStreamType.VISION_STREAM_DRIVER,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


class WebrtcdConnectionProvider:
  def __init__(self, host: str, port: int, cameras: list[str], enabled: bool = True):
    self.url = f"http://{host}:{port}/stream"
    self.cameras = cameras
    self.enabled = enabled

  async def __call__(self, offer: StreamingOffer) -> aiortc.RTCSessionDescription:
    body = {
      "sdp": offer.sdp,
      "init_camera": self.cameras[0],
      "enabled": self.enabled,
      "bridge_services_in": [],
      "bridge_services_out": [],
      "cameras": self.cameras,
    }

    def post_offer() -> dict:
      resp = requests.post(self.url, json=body, timeout=10)
      resp.raise_for_status()
      return resp.json()

    payload = await asyncio.to_thread(post_offer)
    return aiortc.RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])


def parse_cameras(cameras_arg: str) -> list[str]:
  cameras = [camera.strip() for camera in cameras_arg.split(",") if camera.strip()]
  if not cameras:
    raise ValueError("at least one camera is required")

  unknown = sorted(set(cameras) - set(CAMERA_STREAMS))
  if unknown:
    raise ValueError(f"unknown cameras: {','.join(unknown)}")
  return cameras


def build_offer(host: str, port: int, cameras: list[str]) -> WebRTCOfferBuilder:
  builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(host, port, cameras))
  for camera in cameras:
    builder.offer_to_receive_video_stream(camera)
  return builder
