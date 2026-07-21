import asyncio
import contextlib
import json
import os
import queue
import threading
import time
from typing import Any

import capnp

from openpilot.cereal import messaging


def parse_control_services(services_arg: str) -> list[str]:
  return [service.strip() for service in services_arg.split(",") if service.strip()]


def cereal_to_json(msg_content: Any) -> Any:
  if isinstance(msg_content, (capnp._DynamicStructReader, capnp._DynamicStructBuilder)):
    return msg_content.to_dict()
  if isinstance(msg_content, (capnp._DynamicListReader, capnp._DynamicListBuilder)):
    return [cereal_to_json(msg) for msg in msg_content]
  if isinstance(msg_content, bytes):
    return msg_content.decode()
  return msg_content


def cereal_message_payload(service: str, sm: messaging.SubMaster) -> str:
  msg = {
    "type": service,
    "logMonoTime": sm.logMonoTime[service],
    "valid": sm.valid[service],
    "data": cereal_to_json(sm[service]),
  }
  return json.dumps(msg)


class CerealDataChannelSender:
  def __init__(
    self,
    services: list[str],
    channel,
    update_interval: float = 0.02,
    log_interval: float = 5.0,
    max_buffered_amount: int = 65536,
  ):
    self.services = services
    self.channel = channel
    self.update_interval = update_interval
    self.log_interval = log_interval
    self.max_buffered_amount = max_buffered_amount
    self.sm = messaging.SubMaster(services)
    self.sent: dict[str, int] = dict.fromkeys(services, 0)
    self.skipped: dict[str, int] = dict.fromkeys(services, 0)
    self._send_queue: queue.Queue = queue.Queue(maxsize=1)
    self._debug = os.getenv("TURBO_GCS_WEBRTC_CONTROL_DEBUG") is not None
    self._send_count = 0
    self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
    self._send_thread.start()

  def close(self) -> None:
    with contextlib.suppress(queue.Empty):
      self._send_queue.get_nowait()
    with contextlib.suppress(queue.Full):
      self._send_queue.put_nowait(None)
    self._send_thread.join(timeout=1.0)

  def _send_loop(self) -> None:
    while True:
      item = self._send_queue.get()
      if item is None:
        return
      service, payload = item
      try:
        self._send_count += 1
        send_count = self._send_count
        if self._debug:
          print(f"webrtc controls send begin service={service} count={send_count}", flush=True)
        start = time.monotonic()
        self.channel.send(payload)
        elapsed = time.monotonic() - start
        if self._debug or elapsed > 0.05:
          print(f"webrtc controls send end service={service} count={send_count} elapsed={elapsed:.3f}s", flush=True)
        self.sent[service] += 1
      except Exception as e:
        if self._debug:
          print(f"webrtc controls send failed service={service} count={send_count} error={type(e).__name__}: {e}", flush=True)
        self.skipped[service] += 1

  def queue_send(self, service: str, payload: str) -> None:
    try:
      self._send_queue.put_nowait((service, payload))
      return
    except queue.Full:
      with contextlib.suppress(queue.Empty):
        self._send_queue.get_nowait()
      self.skipped[service] += 1
    with contextlib.suppress(queue.Full):
      self._send_queue.put_nowait((service, payload))

  def buffered_amount(self) -> int:
    return 0

  def channel_open(self) -> bool:
    # Avoid polling libdatachannel state from the Python control loop. Native
    # state queries have blocked/crashed in live GCS sessions; send failures are
    # handled at the call site.
    return True

  async def run(self) -> None:
    last_log = time.monotonic()
    try:
      while True:
        self.sm.update(0)
        for service, updated in self.sm.updated.items():
          if not updated:
            continue
          if not self.channel_open():
            self.skipped[service] += 1
            continue
          self.queue_send(service, cereal_message_payload(service, self.sm))

        now = time.monotonic()
        if now - last_log >= self.log_interval:
          sent_counts = " ".join(f"{service}={count}" for service, count in self.sent.items())
          skipped_counts = " ".join(f"{service}={count}" for service, count in self.skipped.items())
          print(
            " ".join((
              f"webrtc controls sent {sent_counts}",
              f"skipped {skipped_counts}",
              f"interval={self.update_interval:.3f}s",
            )),
            flush=True,
          )
          last_log = now

        await asyncio.sleep(self.update_interval)
    finally:
      self.close()


class SyntheticDataChannelSender:
  def __init__(self, channel, update_interval: float = 0.02, payload_bytes: int = 256, log_interval: float = 5.0):
    self.channel = channel
    self.update_interval = update_interval
    self.payload_bytes = payload_bytes
    self.log_interval = log_interval
    self.sent = 0
    self.failed = 0
    self._debug = os.getenv("TURBO_GCS_WEBRTC_CONTROL_DEBUG") is not None
    self._send_queue: queue.Queue = queue.Queue(maxsize=1)
    self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
    self._send_thread.start()

  def close(self) -> None:
    with contextlib.suppress(queue.Empty):
      self._send_queue.get_nowait()
    with contextlib.suppress(queue.Full):
      self._send_queue.put_nowait(None)
    self._send_thread.join(timeout=1.0)

  def _send_loop(self) -> None:
    while True:
      payload = self._send_queue.get()
      if payload is None:
        return
      count = self.sent + self.failed + 1
      try:
        if self._debug:
          print(f"webrtc synthetic send begin count={count}", flush=True)
        start = time.monotonic()
        self.channel.send(payload)
        elapsed = time.monotonic() - start
        if self._debug or elapsed > 0.05:
          print(f"webrtc synthetic send end count={count} elapsed={elapsed:.3f}s", flush=True)
        self.sent += 1
      except Exception as e:
        self.failed += 1
        if self._debug:
          print(f"webrtc synthetic send failed count={count} error={type(e).__name__}: {e}", flush=True)

  def queue_send(self, payload: str) -> None:
    try:
      self._send_queue.put_nowait(payload)
      return
    except queue.Full:
      with contextlib.suppress(queue.Empty):
        self._send_queue.get_nowait()
    with contextlib.suppress(queue.Full):
      self._send_queue.put_nowait(payload)

  def payload(self) -> str:
    base = {
      "type": "turboSyntheticPing",
      "logMonoTime": time.monotonic_ns(),
      "valid": True,
      "data": {"sequence": self.sent + self.failed, "padding": ""},
    }
    encoded = json.dumps(base)
    if len(encoded) < self.payload_bytes:
      base["data"]["padding"] = "x" * (self.payload_bytes - len(encoded))
    return json.dumps(base)

  async def run(self) -> None:
    last_log = time.monotonic()
    try:
      while True:
        self.queue_send(self.payload())

        now = time.monotonic()
        if now - last_log >= self.log_interval:
          print(
            f"webrtc synthetic sent={self.sent} failed={self.failed} interval={self.update_interval:.3f}s payload={self.payload_bytes}B",
            flush=True,
          )
          last_log = now

        await asyncio.sleep(self.update_interval)
    finally:
      self.close()
