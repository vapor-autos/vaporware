#!/usr/bin/env python3
import argparse
import asyncio
import dataclasses
import json
import os
import time

import av
import numpy as np

from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.tools.turbo.webrtc_client import build_offer, parse_cameras


CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "driver": VisionStreamType.VISION_STREAM_DRIVER,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


def frame_to_nv12(frame: av.VideoFrame) -> np.ndarray:
  yuv420 = frame.to_ndarray(format=av.video.format.VideoFormat("yuv420p")).flatten()
  uv_offset = frame.height * frame.width
  y = yuv420[:uv_offset]
  uv = yuv420[uv_offset:].reshape(2, -1).ravel("F")
  return np.hstack((y, uv))


def new_perf_stats() -> dict[str, int]:
  return {
    "convert_ns": 0,
    "convert_max_ns": 0,
    "send_wait_ns": 0,
    "send_ns": 0,
    "send_max_ns": 0,
  }


async def print_stats(stream, interval: float) -> None:
  while stream.is_started:
    await asyncio.sleep(interval)
    report = await stream.peer_connection.getStats()
    summary = {}
    for stat in report.values():
      if stat.type in ("inbound-rtp", "remote-outbound-rtp", "transport", "candidate-pair"):
        if dataclasses.is_dataclass(stat):
          stat_payload = dataclasses.asdict(stat)
        else:
          stat_payload = dict(getattr(stat, "__dict__", {}))
        summary[f"{stat.type}:{stat.id}"] = stat_payload
    if summary:
      print(json.dumps({"stats": summary}, default=str, sort_keys=True), flush=True)


async def pump_camera(
  camera: str,
  track,
  vipc_server: VisionIpcServer,
  first_frame: av.VideoFrame,
  send_lock: asyncio.Lock,
  frame_counts: dict[str, int],
  perf_stats: dict[str, dict[str, int]],
  end_time: float | None,
) -> None:
  stream_type = CAMERA_STREAMS[camera]
  frame: av.VideoFrame | None = first_frame

  while end_time is None or time.monotonic() < end_time:
    if frame is None:
      frame = await track.recv()

    frame_id = frame_counts[camera]
    timestamp = time.monotonic_ns()
    convert_start = time.monotonic_ns()
    img_yuv = frame_to_nv12(frame)
    convert_ns = time.monotonic_ns() - convert_start

    send_wait_start = time.monotonic_ns()
    async with send_lock:
      send_start = time.monotonic_ns()
      vipc_server.send(stream_type, img_yuv.data, frame_id, timestamp, timestamp)
      send_ns = time.monotonic_ns() - send_start

    stats = perf_stats[camera]
    stats["convert_ns"] += convert_ns
    stats["convert_max_ns"] = max(stats["convert_max_ns"], convert_ns)
    stats["send_wait_ns"] += send_start - send_wait_start
    stats["send_ns"] += send_ns
    stats["send_max_ns"] = max(stats["send_max_ns"], send_ns)

    frame_counts[camera] = frame_id + 1
    frame = None


async def log_frame_counts(frame_counts: dict[str, int], perf_stats: dict[str, dict[str, int]], interval: float) -> None:
  last_counts = frame_counts.copy()
  last_perf_stats = {camera: stats.copy() for camera, stats in perf_stats.items()}
  last_time = time.monotonic()
  while True:
    await asyncio.sleep(interval)
    now = time.monotonic()
    elapsed = now - last_time
    fps_parts = []
    for camera, count in frame_counts.items():
      delta_frames = count - last_counts[camera]
      fps = delta_frames / elapsed
      stats = perf_stats[camera]
      last_stats = last_perf_stats[camera]
      if delta_frames > 0:
        convert_ms = (stats["convert_ns"] - last_stats["convert_ns"]) / delta_frames / 1e6
        send_wait_ms = (stats["send_wait_ns"] - last_stats["send_wait_ns"]) / delta_frames / 1e6
        send_ms = (stats["send_ns"] - last_stats["send_ns"]) / delta_frames / 1e6
      else:
        convert_ms = 0.0
        send_wait_ms = 0.0
        send_ms = 0.0
      fps_parts.append(" ".join((
        f"{camera}={fps:.1f}fps",
        f"frames={count}",
        f"convert={convert_ms:.2f}ms",
        f"convert_max={stats['convert_max_ns'] / 1e6:.2f}ms",
        f"send_wait={send_wait_ms:.2f}ms",
        f"send={send_ms:.2f}ms",
        f"send_max={stats['send_max_ns'] / 1e6:.2f}ms",
      )))
      last_counts[camera] = count
      last_perf_stats[camera] = stats.copy()
    print(" ".join(fps_parts), flush=True)
    last_time = now


async def run(args: argparse.Namespace) -> None:
  cameras = parse_cameras(args.cameras)
  builder = build_offer(args.host, args.port, cameras)
  if args.quality:
    builder.add_messaging()

  stream = builder.stream()
  stats_task = None
  log_task = None
  camera_tasks = []

  try:
    await stream.start()
    await stream.wait_for_connection()
    print(f"connected cameras={','.join(cameras)} server={args.server}", flush=True)

    if args.quality:
      stream.get_messaging_channel().send(json.dumps({"type": "livestreamSettings", "data": {"quality": args.quality}}))
      print(f"quality={args.quality}", flush=True)

    if args.stats:
      stats_task = asyncio.create_task(print_stats(stream, args.stats_interval))

    tracks = {camera: stream.get_incoming_video_track(camera, buffered=False) for camera in cameras}
    first_frames = dict(zip(
      cameras,
      await asyncio.gather(*(tracks[camera].recv() for camera in cameras)),
      strict=True,
    ))

    vipc_server = VisionIpcServer(args.server)
    for camera, frame in first_frames.items():
      vipc_server.create_buffers(CAMERA_STREAMS[camera], args.num_buffers, frame.width, frame.height)
      print(f"vipc {camera} {frame.width}x{frame.height}", flush=True)
    vipc_server.start_listener()

    end_time = None if args.duration <= 0 else time.monotonic() + args.duration
    frame_counts = dict.fromkeys(cameras, 0)
    perf_stats = {camera: new_perf_stats() for camera in cameras}
    send_lock = asyncio.Lock()
    if args.log_interval > 0:
      log_task = asyncio.create_task(log_frame_counts(frame_counts, perf_stats, args.log_interval))

    camera_tasks = [
      asyncio.create_task(pump_camera(camera, tracks[camera], vipc_server, first_frames[camera], send_lock, frame_counts, perf_stats, end_time))
      for camera in cameras
    ]
    await asyncio.gather(*camera_tasks)
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
  parser = argparse.ArgumentParser(description="Receive WebRTC camera tracks and republish them as local VisionIPC streams")
  parser.add_argument("--host", default=os.getenv("TURBO_UGV_IP", "127.0.0.1"), help="UGV/webrtcd host")
  parser.add_argument("--port", type=int, default=5001, help="UGV/webrtcd HTTP signaling port")
  parser.add_argument("--cameras", default=os.getenv("TURBO_GCS_WEBRTC_CAMS", "wideRoad,driver,road"), help="comma-separated cameras to request")
  parser.add_argument("--server", default="camerad", help="local VisionIPC server name")
  parser.add_argument(
    "--quality",
    choices=("low", "med", "high", "auto"),
    default=os.getenv("TURBO_GCS_WEBRTC_QUALITY", "med"),
    help="livestream quality sent over WebRTC data channel",
  )
  parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; <=0 runs until disconnected")
  parser.add_argument("--num-buffers", type=int, default=4, help="VisionIPC buffers per stream")
  parser.add_argument("--log-interval", type=float, default=1.0, help="frame log interval in seconds")
  parser.add_argument("--stats", action="store_true", help="print periodic WebRTC stats")
  parser.add_argument("--stats-interval", type=float, default=2.0, help="WebRTC stats log interval in seconds")
  args = parser.parse_args()

  asyncio.run(run(args))


if __name__ == "__main__":
  main()
