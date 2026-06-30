#!/usr/bin/env python3
import argparse
import asyncio
import dataclasses
import json
import os
import time

import aiortc
import requests

from teleoprtc import WebRTCOfferBuilder, StreamingOffer


def stat_dict(stat) -> dict:
  if dataclasses.is_dataclass(stat):
    return dataclasses.asdict(stat)
  return dict(getattr(stat, "__dict__", {}))


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


async def print_stats(stream, interval: float) -> None:
  while stream.is_started:
    await asyncio.sleep(interval)
    report = await stream.peer_connection.getStats()
    summary = {}
    for stat in report.values():
      if stat.type in ("inbound-rtp", "remote-outbound-rtp", "transport", "candidate-pair"):
        summary[stat.type] = stat_dict(stat)
    if summary:
      print(json.dumps({"stats": summary}, default=str, sort_keys=True), flush=True)


async def run(args: argparse.Namespace) -> None:
  cameras = args.cameras.split(",") if args.cameras else [args.camera]
  cameras = [camera.strip() for camera in cameras if camera.strip()]
  if not cameras:
    raise ValueError("at least one camera is required")

  builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(args.host, args.port, cameras))
  for camera in cameras:
    builder.offer_to_receive_video_stream(camera)
  if args.messaging:
    builder.add_messaging()

  stream = builder.stream()
  stats_task = None
  frames = 0
  start_time = 0.0
  last_log_time = 0.0
  last_log_frames = 0

  try:
    await stream.start()
    await stream.wait_for_connection()
    print(f"connected cameras={','.join(cameras)}", flush=True)
    start_time = time.monotonic()
    last_log_time = start_time

    if args.stats:
      stats_task = asyncio.create_task(print_stats(stream, args.stats_interval))

    tracks = {camera: stream.get_incoming_video_track(camera, buffered=False) for camera in cameras}
    end_time = None if args.duration <= 0 else start_time + args.duration

    while end_time is None or time.monotonic() < end_time:
      frames_by_camera = dict(zip(cameras, await asyncio.gather(*(tracks[camera].recv() for camera in cameras)), strict=True))
      frames += 1
      now = time.monotonic()
      if now - last_log_time >= args.log_interval:
        fps = (frames - last_log_frames) / (now - last_log_time)
        sizes = " ".join(f"{camera}={frame.width}x{frame.height}" for camera, frame in frames_by_camera.items())
        pts = " ".join(f"{camera}_pts={frame.pts}" for camera, frame in frames_by_camera.items())
        print(
          f"frames={frames} fps={fps:.1f} {sizes} {pts}",
          flush=True,
        )
        last_log_time = now
        last_log_frames = frames

    elapsed = time.monotonic() - start_time
    print(f"done frames={frames} avg_fps={frames / elapsed:.1f} elapsed={elapsed:.2f}s", flush=True)
  finally:
    if stats_task is not None:
      stats_task.cancel()
      try:
        await stats_task
      except asyncio.CancelledError:
        pass
    await stream.stop()


def main() -> None:
  parser = argparse.ArgumentParser(description="Receive one camera from openpilot webrtcd and print decoded frame stats")
  parser.add_argument("--host", default=os.getenv("TURBO_UGV_IP", "127.0.0.1"), help="UGV/webrtcd host")
  parser.add_argument("--port", type=int, default=5001, help="UGV/webrtcd HTTP signaling port")
  parser.add_argument("--camera", choices=("road", "wideRoad", "driver"), default="wideRoad", help="camera to request")
  parser.add_argument("--cameras", help="comma-separated cameras to request in one session")
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; <=0 runs until disconnected")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument("--messaging", action="store_true", help="create a WebRTC data channel")
  parser.add_argument("--stats", action="store_true", help="print periodic WebRTC stats")
  parser.add_argument("--stats-interval", type=float, default=2.0, help="WebRTC stats log interval in seconds")
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
