import asyncio
import dataclasses
import time
from typing import Any
import weakref

import av
import numpy as np

from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.tools.turbo.teleop_metrics import write_metrics_payload


CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "driver": VisionStreamType.VISION_STREAM_DRIVER,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}

_RTCP_FEEDBACK_COUNTERS: weakref.WeakKeyDictionary[Any, dict[str, int]] = weakref.WeakKeyDictionary()
_RTCP_COUNTERS_INSTALLED = False


def install_rtcp_feedback_counters() -> None:
  global _RTCP_COUNTERS_INSTALLED
  if _RTCP_COUNTERS_INSTALLED:
    return

  from aiortc.rtcrtpreceiver import RTCRtpReceiver

  original_nack = RTCRtpReceiver._send_rtcp_nack
  original_pli = RTCRtpReceiver._send_rtcp_pli

  async def counted_nack(self, media_ssrc: int, lost: list[int]) -> None:
    counters = _RTCP_FEEDBACK_COUNTERS.setdefault(self, {"nack_requests": 0, "nack_packets": 0, "pli": 0})
    counters["nack_requests"] += 1
    counters["nack_packets"] += len(lost)
    await original_nack(self, media_ssrc, lost)

  async def counted_pli(self, media_ssrc: int) -> None:
    counters = _RTCP_FEEDBACK_COUNTERS.setdefault(self, {"nack_requests": 0, "nack_packets": 0, "pli": 0})
    counters["pli"] += 1
    await original_pli(self, media_ssrc)

  RTCRtpReceiver._send_rtcp_nack = counted_nack
  RTCRtpReceiver._send_rtcp_pli = counted_pli
  _RTCP_COUNTERS_INSTALLED = True


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
  # aiortc does not expose the selected ICE candidate pair through public stats in this version.
  # Keep this as best-effort diagnostics for LTE/STUN testing.
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


async def print_stats(stream, interval: float, stats_file: str | None = None, latest_file: str | None = None) -> None:
  install_rtcp_feedback_counters()
  last_ice_payload = None
  last_inbound: dict[str, dict[str, int]] = {}
  last_transport: dict[str, dict[str, int]] = {}
  last_feedback: dict[int, dict[str, int]] = {}

  while stream.is_started:
    await asyncio.sleep(interval)
    report = await stream.peer_connection.getStats()

    receivers = stream.peer_connection.getReceivers()
    receiver_by_stats_id = {f"inbound-rtp_{id(receiver)}": receiver for receiver in receivers}
    receiver_labels = {
      id(receiver): getattr(getattr(receiver, "track", None), "id", None) or f"receiver_{index}"
      for index, receiver in enumerate(receivers)
    }

    summary: dict[str, Any] = {
      "inbound": {},
      "remote_outbound": {},
      "transport": {},
    }
    for stat in report.values():
      if stat.type == "inbound-rtp":
        receiver = receiver_by_stats_id.get(stat.id)
        label = receiver_labels.get(id(receiver), stat.id) if receiver is not None else stat.id
        previous = last_inbound.get(stat.id, {})
        packets_received_delta = stat.packetsReceived - previous.get("packetsReceived", stat.packetsReceived)
        packets_lost_delta = max(0, stat.packetsLost - previous.get("packetsLost", stat.packetsLost))
        lost_total = stat.packetsReceived + stat.packetsLost
        lost_delta_total = packets_received_delta + packets_lost_delta
        payload = {
          "ssrc": stat.ssrc,
          "kind": stat.kind,
          "packets_received": stat.packetsReceived,
          "packets_received_delta": packets_received_delta,
          "packets_lost": stat.packetsLost,
          "packets_lost_delta": packets_lost_delta,
          "packet_loss_pct": (stat.packetsLost / lost_total * 100.0) if lost_total > 0 else 0.0,
          "packet_loss_delta_pct": (packets_lost_delta / lost_delta_total * 100.0) if lost_delta_total > 0 else 0.0,
          "jitter_rtp": stat.jitter,
          "jitter_ms": (stat.jitter / 90.0) if stat.kind == "video" else None,
        }
        if receiver is not None:
          counters = _RTCP_FEEDBACK_COUNTERS.get(receiver, {})
          previous_feedback = last_feedback.get(id(receiver), {})
          payload.update({
            "nack_requests": counters.get("nack_requests", 0),
            "nack_requests_delta": counters.get("nack_requests", 0) - previous_feedback.get("nack_requests", counters.get("nack_requests", 0)),
            "nack_packets": counters.get("nack_packets", 0),
            "nack_packets_delta": counters.get("nack_packets", 0) - previous_feedback.get("nack_packets", counters.get("nack_packets", 0)),
            "pli": counters.get("pli", 0),
            "pli_delta": counters.get("pli", 0) - previous_feedback.get("pli", counters.get("pli", 0)),
          })
          last_feedback[id(receiver)] = counters.copy()
        summary["inbound"][label] = payload
        last_inbound[stat.id] = {
          "packetsReceived": stat.packetsReceived,
          "packetsLost": stat.packetsLost,
        }
      elif stat.type == "remote-outbound-rtp":
        summary["remote_outbound"][stat.id] = stat_dict(stat)
      elif stat.type == "transport":
        previous = last_transport.get(stat.id, {})
        bytes_received_delta = stat.bytesReceived - previous.get("bytesReceived", stat.bytesReceived)
        bytes_sent_delta = stat.bytesSent - previous.get("bytesSent", stat.bytesSent)
        packets_received_delta = stat.packetsReceived - previous.get("packetsReceived", stat.packetsReceived)
        packets_sent_delta = stat.packetsSent - previous.get("packetsSent", stat.packetsSent)
        summary["transport"][stat.id] = {
          "bytes_received": stat.bytesReceived,
          "bytes_received_delta": bytes_received_delta,
          "rx_kbps": bytes_received_delta * 8 / interval / 1000.0,
          "bytes_sent": stat.bytesSent,
          "bytes_sent_delta": bytes_sent_delta,
          "tx_kbps": bytes_sent_delta * 8 / interval / 1000.0,
          "packets_received": stat.packetsReceived,
          "packets_received_delta": packets_received_delta,
          "packets_sent": stat.packetsSent,
          "packets_sent_delta": packets_sent_delta,
          "ice_role": stat.iceRole,
          "dtls_state": stat.dtlsState,
        }
        last_transport[stat.id] = {
          "bytesReceived": stat.bytesReceived,
          "bytesSent": stat.bytesSent,
          "packetsReceived": stat.packetsReceived,
          "packetsSent": stat.packetsSent,
        }
    if summary:
      write_metrics_payload({"stats": summary}, stats_file, latest_file)

    ice_payload = ice_summary(stream.peer_connection)
    if ice_payload and ice_payload != last_ice_payload:
      write_metrics_payload({"ice": ice_payload}, stats_file, latest_file)
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
