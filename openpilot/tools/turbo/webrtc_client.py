import asyncio
from dataclasses import asdict
import json

import aiortc
import requests

from openpilot.system.webrtc.helpers import StreamRequestBody
from teleoprtc import WebRTCOfferBuilder, StreamingOffer


CAMERA_TYPES = ("road", "driver", "wideRoad")


class WebrtcdConnectionProvider:
  def __init__(self, host: str, port: int, cameras: list[str], enabled: bool = True, feedback_services: list[str] | None = None):
    self.url = f"http://{host}:{port}/stream"
    self.cameras = cameras
    self.enabled = enabled
    self.feedback_services = [] if feedback_services is None else feedback_services

  async def __call__(self, offer: StreamingOffer) -> aiortc.RTCSessionDescription:
    body = StreamRequestBody(
      sdp=offer.sdp,
      init_camera=self.cameras[0],
      enabled=self.enabled,
      bridge_services_out=self.feedback_services,
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


def build_offer(host: str, port: int, cameras: list[str], feedback_services: list[str] | None = None) -> WebRTCOfferBuilder:
  builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(host, port, cameras, feedback_services=feedback_services))
  for camera in cameras:
    builder.offer_to_receive_video_stream(camera)
  return builder


def send_livestream_quality(stream, quality: str | None) -> None:
  if quality:
    stream.get_messaging_channel().send(json.dumps({"type": "livestreamSettings", "data": {"quality": quality}}))
