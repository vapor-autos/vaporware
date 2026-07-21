import argparse
import asyncio
from dataclasses import asdict
import faulthandler
import os
import signal
import uuid

import aiohttp.web
from libdatachannel import DataChannelInit

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.tools.turbo.teleop_metrics import default_latest_json_path, default_metrics_jsonl_path, env_bool
from openpilot.tools.turbo.webrtc_client import parse_cameras, send_livestream_quality
from openpilot.tools.turbo.webrtc_controls import CerealDataChannelSender, SyntheticDataChannelSender, parse_control_services
from openpilot.tools.turbo.webrtc_vipc_publisher import print_stats, publish_stream_to_vipc
from teleoprtc import StreamingOffer, WebRTCOfferBuilder
from teleoprtc.stream import RTCSessionDescription


class GcsAnswerProvider:
  def __init__(self, session_id: str, cameras: list[str], bridge_services_in: list[str], enabled: bool = True):
    self.session_id = session_id
    self.cameras = cameras
    self.bridge_services_in = bridge_services_in
    self.enabled = enabled
    self.offer_ready = asyncio.Event()
    self.answer_future: asyncio.Future[RTCSessionDescription] = asyncio.get_running_loop().create_future()
    self.offer_body: StreamRequestBody | None = None

  async def __call__(self, offer: StreamingOffer) -> RTCSessionDescription:
    cameras = offer.video if offer.video else self.cameras
    self.offer_body = StreamRequestBody(
      sdp=offer.sdp,
      init_camera=cameras[0] if cameras else "",
      enabled=self.enabled,
      bridge_services_in=self.bridge_services_in,
      cameras=cameras,
    )
    self.offer_ready.set()
    return await self.answer_future

  def set_answer(self, answer: dict) -> None:
    if self.answer_future.done():
      raise ValueError("answer already received")
    self.answer_future.set_result(RTCSessionDescription(sdp=answer["sdp"], type=answer["type"]))


class SignalingSession:
  def __init__(self, args: argparse.Namespace, cameras: list[str], kind: str):
    self.args = args
    self.kind = kind
    self.cameras = cameras
    self.control_services = parse_control_services(args.control_services)
    self.session_id = uuid.uuid4().hex
    bridge_services_in = self.control_services if self.kind == "controls" else []
    self.provider = GcsAnswerProvider(self.session_id, cameras, bridge_services_in, enabled=args.video_enabled)
    builder = WebRTCOfferBuilder(self.provider)
    for camera in cameras:
      builder.offer_to_receive_video_stream(camera)
    if self.kind == "video" and args.quality:
      builder.add_messaging()
    elif self.kind == "controls":
      builder.add_messaging(data_channel_init=control_data_channel_init())
    self.stream = builder.stream()
    self.task = asyncio.create_task(self.run())

  async def run(self) -> None:
    try:
      await self.stream.start()
      await self.stream.wait_for_connection()
      cameras = ",".join(self.cameras) if self.cameras else "none"
      print(f"connected session={self.session_id} kind={self.kind} cameras={cameras} server={self.args.server}", flush=True)

      if self.kind == "video":
        await self.run_video()
      else:
        await self.run_controls()
    except Exception as e:
      print(f"signaling session failed kind={self.kind}: {type(e).__name__}: {e}", flush=True)
    finally:
      await self.stream.stop()

  async def run_video(self) -> None:
    if self.args.quality:
      send_livestream_quality(self.stream, self.args.quality)
      print(f"quality={self.args.quality}", flush=True)

    stats_task = None
    try:
      if self.args.stats:
        stats_task = asyncio.create_task(print_stats(self.stream, self.args.stats_interval, self.args.stats_file, self.args.stats_latest_file))
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

  async def run_controls(self) -> None:
    controls_task = None
    synthetic_task = None
    try:
      if self.control_services:
        controls_task = asyncio.create_task(CerealDataChannelSender(
          self.control_services,
          self.stream.get_messaging_channel(),
          update_interval=self.args.control_update_interval,
          max_buffered_amount=self.args.control_max_buffered_amount,
        ).run())
        print(f"controls={','.join(self.control_services)}", flush=True)
      if self.args.synthetic_data_rate > 0:
        synthetic_task = asyncio.create_task(SyntheticDataChannelSender(
          self.stream.get_messaging_channel(),
          update_interval=1.0 / self.args.synthetic_data_rate,
          payload_bytes=self.args.synthetic_data_bytes,
        ).run())
        print(f"synthetic_data={self.args.synthetic_data_rate:.1f}hz payload={self.args.synthetic_data_bytes}B", flush=True)

      if self.args.duration > 0:
        await asyncio.sleep(self.args.duration)
      else:
        await self.stream.wait_for_disconnection()
    finally:
      if controls_task is not None:
        controls_task.cancel()
        await asyncio.gather(controls_task, return_exceptions=True)
      if synthetic_task is not None:
        synthetic_task.cancel()
        await asyncio.gather(synthetic_task, return_exceptions=True)

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
    self.sessions: dict[str, SignalingSession] = {}
    self.lock = asyncio.Lock()

  @property
  def controls_enabled(self) -> bool:
    return bool(parse_control_services(self.args.control_services)) or self.args.synthetic_data_rate > 0

  def ensure_sessions(self) -> None:
    video_session = self.sessions.get("video")
    if video_session is None or video_session.task.done():
      self.sessions["video"] = SignalingSession(self.args, self.cameras, "video")

    controls_session = self.sessions.get("controls")
    if self.controls_enabled:
      if controls_session is None or controls_session.task.done():
        self.sessions["controls"] = SignalingSession(self.args, [], "controls")
    elif controls_session is not None:
      controls_session.task.cancel()
      self.sessions.pop("controls", None)

  def pending_offer_session(self) -> SignalingSession | None:
    for kind in ("video", "controls"):
      session = self.sessions.get(kind)
      if session is None:
        continue
      if session.provider.offer_ready.is_set() and not session.provider.answer_future.done():
        return session
    return None

  async def get_offer_session(self, offer_timeout: float = 10.0) -> SignalingSession:
    deadline = asyncio.get_running_loop().time() + offer_timeout
    while True:
      async with self.lock:
        self.ensure_sessions()
        if session := self.pending_offer_session():
          return session

        wait_tasks = [
          asyncio.create_task(session.provider.offer_ready.wait())
          for session in self.sessions.values()
          if not session.provider.answer_future.done()
        ]

      remaining = deadline - asyncio.get_running_loop().time()
      if remaining <= 0:
        raise TimeoutError
      if not wait_tasks:
        await asyncio.sleep(min(0.05, remaining))
        continue

      done, pending = await asyncio.wait(wait_tasks, timeout=remaining, return_when=asyncio.FIRST_COMPLETED)
      for task in pending:
        task.cancel()
      await asyncio.gather(*pending, return_exceptions=True)
      if not done:
        raise TimeoutError

  async def set_answer(self, session_id: str | None, payload: dict) -> None:
    async with self.lock:
      if session_id is None:
        session = self.pending_offer_session()
      else:
        session = next((s for s in self.sessions.values() if s.session_id == session_id), None)
      if session is None:
        raise KeyError
      session.provider.set_answer(payload)

  async def reset_sessions(self) -> None:
    async with self.lock:
      sessions = list(self.sessions.values())
      self.sessions.clear()
    await asyncio.gather(*(session.stop() for session in sessions), return_exceptions=True)

  async def stop(self) -> None:
    await self.reset_sessions()


def control_data_channel_init() -> DataChannelInit:
  init = DataChannelInit()
  init.reliability.unordered = True
  init.reliability.max_retransmits = 0
  return init


async def handle_health(request: aiohttp.web.Request) -> aiohttp.web.Response:
  return aiohttp.web.json_response({"ok": True})


async def handle_offer(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  try:
    session = await state.get_offer_session(offer_timeout=10)
  except TimeoutError:
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "offer_unavailable",
    }, status=503)

  assert session.provider.offer_body is not None
  if session.provider.answer_future.done():
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "session_active",
      "session_id": session.session_id,
    }, status=409)

  payload = asdict(session.provider.offer_body)
  payload["session_id"] = session.session_id
  return aiohttp.web.json_response(payload)


async def handle_answer(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  payload = await request.json()
  session_id = payload.get("session_id")

  try:
    await state.set_answer(session_id, payload)
  except KeyError:
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "stale_session",
    }, status=409)
  except ValueError:
    return aiohttp.web.json_response({
      "ok": False,
      "reason": "answer_already_received",
    }, status=409)

  return aiohttp.web.json_response({"ok": True})


async def handle_reset(request: aiohttp.web.Request) -> aiohttp.web.Response:
  state: SignalingState = request.app["state"]
  await state.reset_sessions()
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
    await state.stop()
    await runner.cleanup()


def main() -> None:
  faulthandler.register(signal.SIGUSR1, all_threads=True)
  stack_dump_interval = float(os.getenv("TURBO_GCS_WEBRTC_STACK_DUMP_INTERVAL", "0"))
  if stack_dump_interval > 0:
    faulthandler.dump_traceback_later(stack_dump_interval, repeat=True)

  parser = argparse.ArgumentParser(description="GCS signaling server for UGV-outbound WebRTC video")
  parser.add_argument("--host", default=os.getenv("GCS_SIGNALING_HOST", "0.0.0.0"), help="host to listen on")
  parser.add_argument("--port", type=int, default=int(os.getenv("GCS_SIGNALING_PORT", "8443")), help="HTTP signaling port")
  parser.add_argument("--cameras", default=os.getenv("TURBO_GCS_WEBRTC_CAMS", "wideRoad,driver"), help="comma-separated cameras to request")
  parser.add_argument("--server", default="camerad", help="local VisionIPC server name")
  parser.add_argument(
    "--control-services",
    default=os.getenv("TURBO_GCS_WEBRTC_CONTROL_SERVICES", ""),
    help="comma-separated local msgq services to send to the UGV over the WebRTC data channel",
  )
  parser.add_argument(
    "--control-max-buffered-amount",
    type=int,
    default=int(os.getenv("TURBO_GCS_WEBRTC_CONTROL_MAX_BUFFERED_AMOUNT", "65536")),
    help="deprecated; retained for compatibility while libdatachannel buffered amount queries are disabled",
  )
  parser.add_argument(
    "--control-update-interval",
    type=float,
    default=float(os.getenv("TURBO_GCS_WEBRTC_CONTROL_UPDATE_INTERVAL", "0.05")),
    help="seconds between WebRTC control data-channel updates",
  )
  parser.add_argument(
    "--quality",
    choices=("low", "med", "high", "auto"),
    default=os.getenv("TURBO_GCS_WEBRTC_QUALITY", "low"),
    help="livestream quality sent over WebRTC data channel",
  )
  parser.add_argument(
    "--synthetic-data-rate",
    type=float,
    default=float(os.getenv("TURBO_GCS_WEBRTC_SYNTHETIC_DATA_RATE", "0")),
    help="send ignored synthetic JSON messages over the WebRTC data channel at this rate in Hz",
  )
  parser.add_argument(
    "--synthetic-data-bytes",
    type=int,
    default=int(os.getenv("TURBO_GCS_WEBRTC_SYNTHETIC_DATA_BYTES", "256")),
    help="approximate synthetic data-channel JSON payload size",
  )
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run per session; <=0 runs until disconnected")
  parser.add_argument(
    "--video-enabled",
    action=argparse.BooleanOptionalAction,
    default=env_bool("TURBO_GCS_WEBRTC_VIDEO_ENABLED", True),
    help="request enabled video tracks from the UGV",
  )
  parser.add_argument("--num-buffers", type=int, default=4, help="VisionIPC buffers per stream")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument(
    "--stats",
    action="store_true",
    default=env_bool("TURBO_GCS_WEBRTC_STATS"),
    help="print periodic WebRTC stats",
  )
  parser.add_argument(
    "--stats-interval",
    type=float,
    default=float(os.getenv("TURBO_GCS_WEBRTC_STATS_INTERVAL", "2.0")),
    help="WebRTC stats log interval in seconds",
  )
  parser.add_argument(
    "--stats-file",
    default=os.getenv("TURBO_GCS_WEBRTC_STATS_FILE") or default_metrics_jsonl_path("gcs_webrtc"),
    help="optional JSONL file for periodic WebRTC stats",
  )
  parser.add_argument(
    "--stats-latest-file",
    default=os.getenv("TURBO_GCS_WEBRTC_STATS_LATEST_FILE") or default_latest_json_path("gcs_webrtc"),
    help="optional JSON file for the latest WebRTC stats snapshot",
  )
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
