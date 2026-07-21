import asyncio
import json
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


def cereal_message_payload(service: str, sm: messaging.SubMaster) -> bytes:
  msg = {
    "type": service,
    "logMonoTime": sm.logMonoTime[service],
    "valid": sm.valid[service],
    "data": cereal_to_json(sm[service]),
  }
  return json.dumps(msg).encode()


class CerealDataChannelSender:
  def __init__(
    self,
    services: list[str],
    channel,
    update_interval: float = 0.01,
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
    self.max_observed_buffered_amount = 0

  def buffered_amount(self) -> int:
    # libdatachannel-py 2026.1.0.dev2 has been observed to segfault when
    # querying DataChannel.buffered_amount() during live sessions.
    return int(getattr(self.channel, "bufferedAmount", 0))

  def channel_open(self) -> bool:
    is_open = getattr(self.channel, "is_open", None)
    return bool(is_open()) if callable(is_open) else True

  async def run(self) -> None:
    last_log = time.monotonic()
    while True:
      self.sm.update(0)
      for service, updated in self.sm.updated.items():
        if not updated:
          continue
        if not self.channel_open():
          self.skipped[service] += 1
          continue
        buffered_amount = self.buffered_amount()
        self.max_observed_buffered_amount = max(self.max_observed_buffered_amount, buffered_amount)
        if self.max_buffered_amount > 0 and buffered_amount > self.max_buffered_amount:
          self.skipped[service] += 1
          continue
        self.channel.send(cereal_message_payload(service, self.sm))
        self.sent[service] += 1

      now = time.monotonic()
      if now - last_log >= self.log_interval:
        sent_counts = " ".join(f"{service}={count}" for service, count in self.sent.items())
        skipped_counts = " ".join(f"{service}={count}" for service, count in self.skipped.items())
        print(
          " ".join((
            f"webrtc controls sent {sent_counts}",
            f"skipped {skipped_counts}",
            f"buffered={self.buffered_amount()}",
            f"buffered_max={self.max_observed_buffered_amount}",
          )),
          flush=True,
        )
        self.max_observed_buffered_amount = self.buffered_amount()
        last_log = now

      await asyncio.sleep(self.update_interval)
