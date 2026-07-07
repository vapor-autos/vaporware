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
  def __init__(self, services: list[str], channel, update_interval: float = 0.01, log_interval: float = 5.0):
    self.services = services
    self.channel = channel
    self.update_interval = update_interval
    self.log_interval = log_interval
    self.sm = messaging.SubMaster(services)
    self.sent: dict[str, int] = dict.fromkeys(services, 0)

  async def run(self) -> None:
    last_log = time.monotonic()
    while True:
      self.sm.update(0)
      for service, updated in self.sm.updated.items():
        if not updated:
          continue
        self.channel.send(cereal_message_payload(service, self.sm))
        self.sent[service] += 1

      now = time.monotonic()
      if now - last_log >= self.log_interval:
        counts = " ".join(f"{service}={count}" for service, count in self.sent.items())
        print(f"webrtc controls sent {counts}", flush=True)
        last_log = now

      await asyncio.sleep(self.update_interval)
