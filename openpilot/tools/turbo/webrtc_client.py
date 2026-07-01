import asyncio
from dataclasses import asdict

import aiortc
import requests

from openpilot.system.webrtc.helpers import StreamRequestBody
from teleoprtc import WebRTCOfferBuilder, StreamingOffer


CAMERA_TYPES = ("road", "driver", "wideRoad")


class WebrtcdConnectionProvider:
  def __init__(self, host: str, port: int, cameras: list[str], enabled: bool = True):
    self.url = f"http://{host}:{port}/stream"
    self.cameras = cameras
    self.enabled = enabled

  async def __call__(self, offer: StreamingOffer) -> aiortc.RTCSessionDescription:
    body = StreamRequestBody(
      sdp=offer.sdp,
      init_camera=self.cameras[0],
      enabled=self.enabled,
      cameras=self.cameras,
    )

    def post_offer() -> dict:
      resp = requests.post(self.url, json=asdict(body), timeout=10)
      resp.raise_for_status()
      return resp.json()

    payload = await asyncio.to_thread(post_offer)
    return aiortc.RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])


def parse_cameras(cameras_arg: str) -> list[str]:
  cameras = [camera.strip() for camera in cameras_arg.split(",") if camera.strip()]
  if not cameras:
    raise ValueError("at least one camera is required")

  unknown = sorted(set(cameras) - set(CAMERA_TYPES))
  if unknown:
    raise ValueError(f"unknown cameras: {','.join(unknown)}")
  return cameras


def build_offer(host: str, port: int, cameras: list[str]) -> WebRTCOfferBuilder:
  builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(host, port, cameras))
  for camera in cameras:
    builder.offer_to_receive_video_stream(camera)
  return builder
