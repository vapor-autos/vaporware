import asyncio
import dataclasses
import json
import time
from typing import Any

import av
import numpy as np

from msgq.visionipc import VisionIpcServer, VisionStreamType


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


def stat_dict(stat) -> dict:
  if dataclasses.is_dataclass(stat):
    return dataclasses.asdict(stat)
  return dict(getattr(stat, "__dict__", {}))


def candidate_dict(candidate) -> dict[str, Any]:
  return {
    "host": getattr(candidate, "host", None),
    "port": getattr(candidate, "port", None),
    "type": getattr(candidate, "type", None),
    "transport": getattr(candidate, "transport", None),
    "component": getattr(candidate, "component", None),
    "related_address": getattr(candidate, "related_address", None),
    "related_port": getattr(candidate, "related_port", None),
  }


def get_ice_transports(peer_connection) -> list:
  transports = getattr(peer_connection, "_RTCPeerConnection__iceTransports", None)
  if transports is None:
    return []
  return list(transports)


def ice_transport_summary(transport) -> dict[str, Any]:
  connection = getattr(transport, "_connection", None)
  summary: dict[str, Any] = {
    "role": getattr(transport, "role", None),
    "state": getattr(transport, "state", None),
    "local_candidates": [],
    "remote_candidates": [],
    "selected_pairs": [],
  }
  if connection is None:
    return summary

  summary["local_candidates"] = [candidate_dict(candidate) for candidate in getattr(connection, "local_candidates", [])]
  summary["remote_candidates"] = [candidate_dict(candidate) for candidate in getattr(connection, "remote_candidates", [])]

  nominated = getattr(connection, "_nominated", {})
  for component, pair in sorted(nominated.items()):
    local_candidate = getattr(pair, "local_candidate", None)
    remote_candidate = getattr(pair, "remote_candidate", None)
    summary["selected_pairs"].append({
      "component": component,
      "state": getattr(getattr(pair, "state", None), "name", getattr(pair, "state", None)),
      "nominated": getattr(pair, "nominated", None),
      "local": candidate_dict(local_candidate) if local_candidate is not None else None,
      "remote": candidate_dict(remote_candidate) if remote_candidate is not None else None,
    })
  return summary


def ice_summary(peer_connection) -> list[dict[str, Any]]:
  return [ice_transport_summary(transport) for transport in get_ice_transports(peer_connection)]


async def print_stats(stream, interval: float) -> None:
  last_ice_payload = None
  while stream.is_started:
    await asyncio.sleep(interval)
    report = await stream.peer_connection.getStats()
    summary = {}
    for stat in report.values():
      if stat.type in ("inbound-rtp", "remote-outbound-rtp", "transport"):
        summary[f"{stat.type}:{stat.id}"] = stat_dict(stat)
    if summary:
      print(json.dumps({"stats": summary}, default=str, sort_keys=True), flush=True)

    ice_payload = ice_summary(stream.peer_connection)
    if ice_payload and ice_payload != last_ice_payload:
      print(json.dumps({"ice": ice_payload}, default=str, sort_keys=True), flush=True)
      last_ice_payload = ice_payload


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


async def publish_stream_to_vipc(
  stream,
  cameras: list[str],
  server_name: str,
  num_buffers: int,
  duration: float,
  log_interval: float,
) -> None:
  tracks = {camera: stream.get_incoming_video_track(camera, buffered=False) for camera in cameras}
  first_frames = dict(zip(
    cameras,
    await asyncio.gather(*(tracks[camera].recv() for camera in cameras)),
    strict=True,
  ))

  vipc_server = VisionIpcServer(server_name)
  for camera, frame in first_frames.items():
    vipc_server.create_buffers(CAMERA_STREAMS[camera], num_buffers, frame.width, frame.height)
    print(f"vipc {camera} {frame.width}x{frame.height}", flush=True)
  vipc_server.start_listener()

  end_time = None if duration <= 0 else time.monotonic() + duration
  frame_counts = dict.fromkeys(cameras, 0)
  perf_stats = {camera: new_perf_stats() for camera in cameras}
  send_lock = asyncio.Lock()
  log_task = None
  camera_tasks = []

  try:
    if log_interval > 0:
      log_task = asyncio.create_task(log_frame_counts(frame_counts, perf_stats, log_interval))

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

    pending = [task for task in [*camera_tasks, log_task] if task is not None]
    if pending:
      await asyncio.gather(*pending, return_exceptions=True)
