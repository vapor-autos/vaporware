import asyncio
import contextlib
import dataclasses
import multiprocessing as mp
import queue
import time
from typing import Any

from libdatachannel import H264RtpDepacketizer, NalUnit, RtcpReceivingSession, Track
import numpy as np

from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.tools.camerastream.ffmpeg_decoder import Decoder, FFmpegError
from openpilot.tools.turbo.teleop_metrics import write_metrics_payload


CAMERA_STREAMS = {
  "road": VisionStreamType.VISION_STREAM_ROAD,
  "driver": VisionStreamType.VISION_STREAM_DRIVER,
  "wideRoad": VisionStreamType.VISION_STREAM_WIDE_ROAD,
}


@dataclasses.dataclass(frozen=True)
class DecodedVideoFrame:
  data: np.ndarray
  width: int
  height: int


class H264FrameReceiver:
  def __init__(self, track: Track, max_pending_frames: int = 2, keyframe_retry_interval: float = 0.5):
    self._loop = asyncio.get_running_loop()
    self._track = track
    self._queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=max_pending_frames)
    self._decoder = Decoder("h264")
    self._keyframe_retry_interval = keyframe_retry_interval
    self._closed = False

    depacketizer = H264RtpDepacketizer(NalUnit.Separator.StartSequence)
    self._rtcp = RtcpReceivingSession()
    self._media_handlers = [depacketizer, self._rtcp]
    track.set_media_handler(depacketizer)
    track.chain_media_handler(self._rtcp)
    track.on_frame(self._on_frame)
    track.request_keyframe()

  def close(self) -> None:
    self._closed = True
    self._decoder.close()

  def request_keyframe(self) -> None:
    if self._track.is_open():
      self._track.request_keyframe()

  def _on_frame(self, data, _info) -> None:
    if not self._closed:
      self._loop.call_soon_threadsafe(self._enqueue, bytes(data))

  def _enqueue(self, data: bytes) -> None:
    if self._closed:
      return
    if self._queue.full():
      with contextlib.suppress(asyncio.QueueEmpty):
        self._queue.get_nowait()
    self._queue.put_nowait(data)

  async def recv(self) -> DecodedVideoFrame:
    while True:
      try:
        data = await asyncio.wait_for(self._queue.get(), timeout=self._keyframe_retry_interval)
      except TimeoutError:
        if not self._track.is_open():
          raise ConnectionError("video track closed before decoded frame was available") from None
        self.request_keyframe()
        continue
      try:
        decoded = self._decoder.decode(data)
      except FFmpegError:
        self._decoder.reset()
        self.request_keyframe()
        continue
      if decoded is not None:
        return DecodedVideoFrame(decoded, self._decoder.width, self._decoder.height)


def new_perf_stats() -> dict[str, int]:
  return {
    "decode_ns": 0,
    "decode_max_ns": 0,
    "send_wait_ns": 0,
    "send_ns": 0,
    "send_max_ns": 0,
  }


def stat_dict(stat) -> dict:
  if dataclasses.is_dataclass(stat):
    return dataclasses.asdict(stat)
  return dict(getattr(stat, "__dict__", {}))


def describe_state(value) -> str | None:
  if value is None:
    return None
  return getattr(value, "name", None) or str(value)


def call_optional(obj, name: str):
  method = getattr(obj, name, None)
  return method() if callable(method) else None


def data_channel_summary(channel) -> dict[str, Any]:
  # Avoid DataChannel.buffered_amount(); libdatachannel-py 2026.1.0.dev2 can
  # segfault when it is queried from the Python stats loop.
  return {
    "open": call_optional(channel, "is_open"),
  }


def vipc_publisher_main(
  server_name: str,
  stream_specs: list[tuple[str, int, int, int]],
  num_buffers: int,
  frame_queue,
) -> None:
  stream_types: dict[str, VisionStreamType] = {}
  vipc_server = VisionIpcServer(server_name)
  for camera, stream_type_value, width, height in stream_specs:
    stream_type = VisionStreamType(stream_type_value)
    stream_types[camera] = stream_type
    vipc_server.create_buffers(stream_type, num_buffers, width, height)
    print(f"vipc {camera} {width}x{height}", flush=True)
  vipc_server.start_listener()

  while True:
    item = frame_queue.get()
    if item is None:
      return
    camera, data, frame_id, timestamp = item
    vipc_server.send(stream_types[camera], data, frame_id, timestamp, timestamp)


def put_latest_frame(frame_queue, item) -> None:
  while True:
    try:
      frame_queue.put_nowait(item)
      return
    except queue.Full:
      with contextlib.suppress(queue.Empty):
        frame_queue.get_nowait()


def stop_vipc_publisher(process: mp.Process | None, frame_queue) -> None:
  if frame_queue is not None:
    with contextlib.suppress(Exception):
      frame_queue.put_nowait(None)
  if process is None:
    return

  process.join(timeout=1.0)
  if process.is_alive():
    process.terminate()
    process.join(timeout=2.0)
  if process.is_alive():
    process.kill()
    process.join(timeout=2.0)


async def print_stats(stream, interval: float, stats_file: str | None = None, latest_file: str | None = None) -> None:
  while stream.is_started:
    await asyncio.sleep(interval)
    summary: dict[str, Any] = {
      "connection": {
        "state": describe_state(call_optional(stream.peer_connection, "state")),
        "gathering_state": describe_state(call_optional(stream.peer_connection, "gathering_state")),
      },
      "receiver_reports": {
        camera: stat_dict(report)
        for camera, report in stream.get_receiver_report_stats().items()
      },
    }

    if stream.messaging_channel is not None:
      summary["data_channel"] = data_channel_summary(stream.messaging_channel)
    write_metrics_payload({"stats": summary}, stats_file, latest_file)


async def pump_camera(
  camera: str,
  receiver: H264FrameReceiver,
  frame_queue,
  first_frame: DecodedVideoFrame,
  frame_counts: dict[str, int],
  perf_stats: dict[str, dict[str, int]],
  end_time: float | None,
) -> None:
  frame: DecodedVideoFrame | None = first_frame

  while end_time is None or time.monotonic() < end_time:
    decode_ns = 0
    if frame is None:
      decode_start = time.monotonic_ns()
      frame = await receiver.recv()
      decode_ns = time.monotonic_ns() - decode_start

    frame_id = frame_counts[camera]
    timestamp = time.monotonic_ns()

    send_wait_start = time.monotonic_ns()
    send_start = time.monotonic_ns()
    put_latest_frame(frame_queue, (camera, frame.data.tobytes(), frame_id, timestamp))
    send_ns = time.monotonic_ns() - send_start

    stats = perf_stats[camera]
    stats["decode_ns"] += decode_ns
    stats["decode_max_ns"] = max(stats["decode_max_ns"], decode_ns)
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
        decode_ms = (stats["decode_ns"] - last_stats["decode_ns"]) / delta_frames / 1e6
        send_wait_ms = (stats["send_wait_ns"] - last_stats["send_wait_ns"]) / delta_frames / 1e6
        send_ms = (stats["send_ns"] - last_stats["send_ns"]) / delta_frames / 1e6
      else:
        decode_ms = 0.0
        send_wait_ms = 0.0
        send_ms = 0.0
      fps_parts.append(" ".join((
        f"{camera}={fps:.1f}fps",
        f"frames={count}",
        f"decode={decode_ms:.2f}ms",
        f"decode_max={stats['decode_max_ns'] / 1e6:.2f}ms",
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
  receivers = {
    camera: H264FrameReceiver(stream.get_incoming_video_track(camera))
    for camera in cameras
  }
  log_task = None
  camera_tasks = []
  vipc_process: mp.Process | None = None
  frame_queue = None

  try:
    first_frames = dict(zip(
      cameras,
      await asyncio.gather(*(receivers[camera].recv() for camera in cameras)),
      strict=True,
    ))

    stream_specs = [
      (camera, int(CAMERA_STREAMS[camera]), frame.width, frame.height)
      for camera, frame in first_frames.items()
    ]
    ctx = mp.get_context("spawn")
    frame_queue = ctx.Queue(maxsize=max(2, len(cameras) * 2))
    vipc_process = ctx.Process(target=vipc_publisher_main, args=(server_name, stream_specs, num_buffers, frame_queue))
    vipc_process.start()

    end_time = None if duration <= 0 else time.monotonic() + duration
    frame_counts = dict.fromkeys(cameras, 0)
    perf_stats = {camera: new_perf_stats() for camera in cameras}

    if log_interval > 0:
      log_task = asyncio.create_task(log_frame_counts(frame_counts, perf_stats, log_interval))

    camera_tasks = [
      asyncio.create_task(pump_camera(
        camera,
        receivers[camera],
        frame_queue,
        first_frames[camera],
        frame_counts,
        perf_stats,
        end_time,
      ))
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
    for receiver in receivers.values():
      receiver.close()
    stop_vipc_publisher(vipc_process, frame_queue)
