#!/usr/bin/env python3
import argparse
import asyncio
import dataclasses
import json
import os
import time
from collections.abc import Mapping

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
        summary[f"{stat.type}:{stat.id}"] = stat_dict(stat)
    if summary:
      print(json.dumps({"stats": summary}, default=str, sort_keys=True), flush=True)


async def receive_camera(camera: str, track, frame_counts: dict[str, int], frame_sizes: dict[str, str], end_time: float | None) -> None:
  while end_time is None or time.monotonic() < end_time:
    frame = await track.recv()
    frame_counts[camera] += 1
    frame_sizes[camera] = f"{frame.width}x{frame.height}"


async def print_frame_counts(
  frame_counts: Mapping[str, int],
  frame_sizes: Mapping[str, str],
  interval: float,
) -> None:
  last_counts = dict(frame_counts)
  last_time = time.monotonic()

  while True:
    await asyncio.sleep(interval)
    now = time.monotonic()
    elapsed = now - last_time
    parts = []
    for camera, count in frame_counts.items():
      fps = (count - last_counts[camera]) / elapsed
      parts.append(f"{camera}={fps:.1f}fps frames={count} size={frame_sizes.get(camera, 'unknown')}")
      last_counts[camera] = count
    print(" ".join(parts), flush=True)
    last_time = now


async def run(args: argparse.Namespace) -> None:
  cameras = args.cameras.split(",") if args.cameras else [args.camera]
  cameras = [camera.strip() for camera in cameras if camera.strip()]
  if not cameras:
    raise ValueError("at least one camera is required")

  builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(args.host, args.port, cameras))
  for camera in cameras:
    builder.offer_to_receive_video_stream(camera)
  if args.messaging or args.quality:
    builder.add_messaging()

  stream = builder.stream()
  stats_task = None
  log_task = None
  camera_tasks = []
  start_time = 0.0

  try:
    await stream.start()
    await stream.wait_for_connection()
    print(f"connected cameras={','.join(cameras)}", flush=True)
    start_time = time.monotonic()

    if args.quality:
      stream.get_messaging_channel().send(json.dumps({"type": "livestreamSettings", "data": {"quality": args.quality}}))
      print(f"quality={args.quality}", flush=True)

    if args.stats:
      stats_task = asyncio.create_task(print_stats(stream, args.stats_interval))

    tracks = {camera: stream.get_incoming_video_track(camera, buffered=False) for camera in cameras}
    end_time = None if args.duration <= 0 else start_time + args.duration
    frame_counts = {camera: 0 for camera in cameras}
    frame_sizes: dict[str, str] = {}

    if args.log_interval > 0:
      log_task = asyncio.create_task(print_frame_counts(frame_counts, frame_sizes, args.log_interval))

    camera_tasks = [
      asyncio.create_task(receive_camera(camera, tracks[camera], frame_counts, frame_sizes, end_time))
      for camera in cameras
    ]
    await asyncio.gather(*camera_tasks)

    elapsed = time.monotonic() - start_time
    summary = " ".join(f"{camera}_frames={count}" for camera, count in frame_counts.items())
    print(f"done {summary} elapsed={elapsed:.2f}s", flush=True)
  finally:
    for task in camera_tasks:
      task.cancel()
    if log_task is not None:
      log_task.cancel()
    if stats_task is not None:
      stats_task.cancel()

    pending = [task for task in [*camera_tasks, log_task, stats_task] if task is not None]
    if pending:
      await asyncio.gather(*pending, return_exceptions=True)
    await stream.stop()


def main() -> None:
  parser = argparse.ArgumentParser(description="Debug WebRTC camera receive from openpilot webrtcd")
  parser.add_argument("--host", default=os.getenv("TURBO_UGV_IP", "127.0.0.1"), help="UGV/webrtcd host")
  parser.add_argument("--port", type=int, default=5001, help="UGV/webrtcd HTTP signaling port")
  parser.add_argument("--camera", choices=("road", "wideRoad", "driver"), default="wideRoad", help="camera to request")
  parser.add_argument("--cameras", help="comma-separated cameras to request in one session")
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; <=0 runs until disconnected")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument("--messaging", action="store_true", help="create a WebRTC data channel")
  parser.add_argument("--quality", choices=("low", "med", "high", "auto"), help="set livestream quality over the data channel")
  parser.add_argument("--stats", action="store_true", help="print periodic WebRTC stats")
  parser.add_argument("--stats-interval", type=float, default=2.0, help="WebRTC stats log interval in seconds")
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
