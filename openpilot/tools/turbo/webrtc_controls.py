import asyncio
import json
import time
from typing import Any

import capnp

from openpilot.cereal import messaging


UI_SMOKE_FEEDBACK_SERVICES = [
  "deviceState",
  "pandaStates",
  "selfdriveState",
  "carState",
  "controlsState",
  "roadCameraState",
  "wideRoadCameraState",
  "liveCalibration",
]

UI_MODEL_FEEDBACK_SERVICES = UI_SMOKE_FEEDBACK_SERVICES + [
  "modelV2",
  "longitudinalPlan",
  "radarState",
  "carParams",
]

UI_FULL_FEEDBACK_SERVICES = UI_MODEL_FEEDBACK_SERVICES + [
  "driverMonitoringState",
  "driverStateV2",
  "onroadEvents",
  "liveParameters",
  "carOutput",
  "carControl",
]

FEEDBACK_SERVICE_PROFILES = {
  "torque": ["carState"],
  "ui_smoke": UI_SMOKE_FEEDBACK_SERVICES,
  "ui_model": UI_MODEL_FEEDBACK_SERVICES,
  "ui_full": UI_FULL_FEEDBACK_SERVICES,
}


def parse_services(services_arg: str) -> list[str]:
  return [service.strip() for service in services_arg.split(",") if service.strip()]


def parse_control_services(services_arg: str) -> list[str]:
  return parse_services(services_arg)


def expand_feedback_services(services_arg: str, profile_arg: str = "") -> list[str]:
  services: list[str] = []
  for profile in parse_services(profile_arg):
    if profile not in FEEDBACK_SERVICE_PROFILES:
      valid = ",".join(FEEDBACK_SERVICE_PROFILES)
      raise ValueError(f"unknown feedback profile: {profile}; expected one of {valid}")
    services.extend(FEEDBACK_SERVICE_PROFILES[profile])

  services.extend(parse_services(services_arg))
  return list(dict.fromkeys(services))


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


class CerealDataChannelReceiver:
  def __init__(self, services: list[str], pm: messaging.PubMaster | None = None):
    self.services = list(services)
    self.service_set = set(services)
    self.pm = messaging.PubMaster(self.services) if pm is None else pm
    self.received: dict[str, int] = dict.fromkeys(services, 0)
    self.ignored = 0

  def receive(self, message: bytes | str) -> bool:
    payload = json.loads(message)
    if not isinstance(payload, dict):
      self.ignored += 1
      return False

    service = payload.get("type")
    if service not in self.service_set:
      self.ignored += 1
      return False

    msg_data = payload.get("data")
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(
      service,
      size=size,
      valid=bool(payload.get("valid", False)),
      logMonoTime=int(payload.get("logMonoTime", time.monotonic() * 1e9)),
    )
    setattr(msg, service, msg_data)
    self.pm.send(service, msg)
    self.received[service] += 1
    return True


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
    return int(getattr(self.channel, "bufferedAmount", 0))

  async def run(self) -> None:
    last_log = time.monotonic()
    while True:
      self.sm.update(0)
      for service, updated in self.sm.updated.items():
        if not updated:
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
