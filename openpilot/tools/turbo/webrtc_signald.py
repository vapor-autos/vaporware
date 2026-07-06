#!/usr/bin/env python3
import argparse
import asyncio
from dataclasses import asdict
import json
import os
import uuid

import aiohttp.web
import aiortc

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.tools.turbo.webrtc_client import parse_cameras
from openpilot.tools.turbo.webrtc_vipc_publisher import print_stats, publish_stream_to_vipc
from teleoprtc import StreamingOffer, WebRTCOfferBuilder


class GcsAnswerProvider:
  def __init__(self, session_id: str, cameras: list[str], enabled: bool = True):
    self.session_id = session_id
    self.cameras = cameras
    self.enabled = enabled
    self.offer_ready = asyncio.Event()
    self.answer_future: asyncio.Future[aiortc.RTCSessionDescription] = asyncio.get_running_loop().create_future()
    self.offer_body: StreamRequestBody | None = None

  async def __call__(self, offer: StreamingOffer) -> aiortc.RTCSessionDescription:
    cameras = offer.video if offer.video else self.cameras
    self.offer_body = StreamRequestBody(
      sdp=offer.sdp,
      init_camera=cameras[0],
      enabled=self.enabled,
      cameras=cameras,
    )
    self.offer_ready.set()
    return await self.answer_future

  def set_answer(self, answer: dict) -> None:
    if self.answer_future.done():
      raise ValueError("answer already received")
    self.answer_future.set_result(aiortc.RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))


class SignalingSession:
  def __init__(self, args: argparse.Namespace, cameras: list[str]):
    self.args = args
    self.cameras = cameras
    self.session_id = uuid.uuid4().hex
    self.provider = GcsAnswerProvider(self.session_id, cameras)
    builder = WebRTCOfferBuilder(self.provider)
    for camera in cameras:
      builder.offer_to_receive_video_stream(camera)
    if args.quality:
      builder.add_messaging()
    self.stream = builder.stream()
    self.task = asyncio.create_task(self.run())

  async def run(self) -> None:
    try:
      await self.stream.start()
      await self.stream.wait_for_connection()
      print(f"connected session={self.session_id} cameras={','.join(self.cameras)} server={self.args.server}", flush=True)

      if self.args.quality:
        self.stream.get_messaging_channel().send(json.dumps({"type": "livestreamSettings", "data": {"quality": self.args.quality}}))
        print(f"quality={self.args.quality}", flush=True)

      stats_task = None
      try:
        if self.args.stats:
          stats_task = asyncio.create_task(print_stats(self.stream, self.args.stats_interval))
        await publish_stream_to_vipc(
          self.stream,
          self.cameras,
          self.args.server,
          self.args.num_buffers,
          self.args.duration,
          self.args.log_interval,
        )
      finally:
        if stats_task is not None:
          stats_task.cancel()
          await asyncio.gather(stats_task, return_exceptions=True)
    except Exception as e:
      print(f"signaling session failed: {type(e).__name__}: {e}", flush=True)
    finally:
      await self.stream.stop()

  async def stop(self) -> None:
    if not self.task.done():
      self.task.cancel()
      await asyncio.gather(self.task, return_exceptions=True)
    else:
      await self.stream.stop()


class SignalingState:
  def __init__(self, args: argparse.Namespace):
    self.args = args
    self.cameras = parse_cameras(args.cameras)
    self.session: SignalingSession | None = None
    self.lock = asyncio.Lock()

  async def get_session(self) -> SignalingSession:
    async with self.lock:
      if self.session is None or self.session.task.done():
        self.session = SignalingSession(self.args, self.cameras)
      return self.session

  async def reset_session(self) -> SignalingSession:
    async with self.lock:
      if self.session is not None:
        await self.session.stop()
      self.session = SignalingSession(self.args, self.cameras)
      return self.session


async def handle_health(request: aiohttp.web.Request) -> aiohttp.web.Response:
  return aiohttp.web.json_response({"ok": True})


async def handle_offer(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  session = await state.get_session()
  await asyncio.wait_for(session.provider.offer_ready.wait(), timeout=10)
  assert session.provider.offer_body is not None
  payload = asdict(session.provider.offer_body)
  payload["session_id"] = session.session_id
  return aiohttp.web.json_response(payload)


async def handle_answer(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  session = await state.get_session()
  payload = await request.json()
  session_id = payload.get("session_id")
  if session_id is not None and session_id != session.session_id:
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "stale_session",
      "session_id": session.session_id,
    }, status=409)

  try:
    session.provider.set_answer(payload)
  except ValueError:
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "answer_already_received",
      "session_id": session.session_id,
    }, status=409)

  return aiohttp.web.json_response({"ok": True})


async def handle_reset(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  await state.reset_session()
  return aiohttp.web.json_response({"ok": True})


async def run(args: argparse.Namespace) -> None:
  app = aiohttp.web.Application()
  app["state"] = SignalingState(args)
  app.add_routes([
    aiohttp.web.get("/health", handle_health),
    aiohttp.web.get("/offer", handle_offer),
    aiohttp.web.post("/answer", handle_answer),
    aiohttp.web.post("/reset", handle_reset),
  ])

  runner = aiohttp.web.AppRunner(app)
  await runner.setup()
  site = aiohttp.web.TCPSite(runner, args.host, args.port)
  await site.start()
  print(f"turbo_webrtc_signald listening on {args.host}:{args.port} cameras={args.cameras}", flush=True)

  try:
    await asyncio.Event().wait()
  finally:
    state: SignalingState = app["state"]
    if state.session is not None:
      await state.session.stop()
    await runner.cleanup()


def main() -> None:
  parser = argparse.ArgumentParser(description="GCS signaling server for UGV-outbound WebRTC video")
  parser.add_argument("--host", default=os.getenv("GCS_SIGNALING_HOST", "0.0.0.0"), help="host to listen on")
  parser.add_argument("--port", type=int, default=int(os.getenv("GCS_SIGNALING_PORT", "8443")), help="HTTP signaling port")
  parser.add_argument("--cameras", default=os.getenv("TURBO_GCS_WEBRTC_CAMS", "wideRoad,driver,road"), help="comma-separated cameras to request")
  parser.add_argument("--server", default="camerad", help="local VisionIPC server name")
  parser.add_argument(
    "--quality",
    choices=("low", "med", "high", "auto"),
    default=os.getenv("TURBO_GCS_WEBRTC_QUALITY", "low"),
    help="livestream quality sent over WebRTC data channel",
  )
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run per session; <=0 runs until disconnected")
  parser.add_argument("--num-buffers", type=int, default=4, help="VisionIPC buffers per stream")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument("--stats", action="store_true", help="print periodic WebRTC stats")
  parser.add_argument("--stats-interval", type=float, default=2.0, help="WebRTC stats log interval in seconds")
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
