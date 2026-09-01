import asyncio
from collections.abc import Callable
import json
import socket
import struct
import time
from typing import Any
import zlib

import capnp

from openpilot.cereal import messaging


FEEDBACK_DATA_CHANNEL_LABEL = "feedback"
FEEDBACK_PACKET_MAGIC = b"TFB1"
FEEDBACK_PACKET_PAYLOAD_SIZE = 1000
FEEDBACK_REASSEMBLY_TIMEOUT_S = 2.0
FEEDBACK_MAX_PENDING_MESSAGES = 64
_FEEDBACK_PACKET_HEADER = struct.Struct("!4sIHH")
UDP_CONTROL_SERVICES = frozenset(("g29", "turboSteerAssist"))
TELEOP_COMMAND_SERVICE = "turboTeleopCommand"
G29_EDGE_FIELDS = ("dpadUp", "dpadDown", "l2", "l3")


def create_feedback_data_channel(peer_connection, message_handler):
  channel = peer_connection.createDataChannel(
    FEEDBACK_DATA_CHANNEL_LABEL,
    ordered=False,
    maxRetransmits=0,
  )
  channel.on("message", message_handler)
  return channel


def encode_feedback_packets(payload: bytes, message_id: int) -> list[bytes]:
  compressed = zlib.compress(payload, level=1)
  chunks = [
    compressed[offset:offset + FEEDBACK_PACKET_PAYLOAD_SIZE]
    for offset in range(0, len(compressed), FEEDBACK_PACKET_PAYLOAD_SIZE)
  ] or [b""]
  return [
    _FEEDBACK_PACKET_HEADER.pack(FEEDBACK_PACKET_MAGIC, message_id & 0xFFFFFFFF, index, len(chunks)) + chunk
    for index, chunk in enumerate(chunks)
  ]


class FeedbackPacketReassembler:
  def __init__(
    self,
    timeout_s: float = FEEDBACK_REASSEMBLY_TIMEOUT_S,
    max_pending_messages: int = FEEDBACK_MAX_PENDING_MESSAGES,
  ):
    self.timeout_s = timeout_s
    self.max_pending_messages = max_pending_messages
    self.pending: dict[int, tuple[float, int, dict[int, bytes]]] = {}

  def add(self, packet: bytes, now: float | None = None) -> bytes | None:
    if len(packet) < _FEEDBACK_PACKET_HEADER.size:
      raise ValueError("feedback packet is shorter than its header")

    magic, message_id, index, count = _FEEDBACK_PACKET_HEADER.unpack_from(packet)
    if magic != FEEDBACK_PACKET_MAGIC or count == 0 or index >= count:
      raise ValueError("invalid feedback packet header")

    now = time.monotonic() if now is None else now
    self._expire(now)
    pending = self.pending.get(message_id)
    if pending is None or pending[1] != count:
      if len(self.pending) >= self.max_pending_messages:
        oldest_id = min(self.pending, key=lambda pending_id: self.pending[pending_id][0])
        del self.pending[oldest_id]
      pending = (now, count, {})
      self.pending[message_id] = pending

    pending[2][index] = packet[_FEEDBACK_PACKET_HEADER.size:]
    if len(pending[2]) != count:
      return None

    compressed = b"".join(pending[2][chunk_index] for chunk_index in range(count))
    del self.pending[message_id]
    return zlib.decompress(compressed)

  def _expire(self, now: float) -> None:
    expired = [message_id for message_id, pending in self.pending.items() if now - pending[0] > self.timeout_s]
    for message_id in expired:
      del self.pending[message_id]


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
  "carParams",
  "liveParameters",
  "onroadEvents",
]

UI_FULL_FEEDBACK_SERVICES = UI_MODEL_FEEDBACK_SERVICES + [
  "longitudinalPlan",
  "radarState",
  "driverMonitoringState",
  "driverStateV2",
  "carOutput",
  "carControl",
]

STEER_ASSIST_FEEDBACK_SERVICES = [
  "carState",
  "selfdriveState",
  "controlsState",
  "carOutput",
]

FEEDBACK_SERVICE_PROFILES = {
  "torque": ["carState"],
  "steer_assist": STEER_ASSIST_FEEDBACK_SERVICES,
  "ui_smoke": UI_SMOKE_FEEDBACK_SERVICES,
  "ui_model": UI_MODEL_FEEDBACK_SERVICES,
  "ui_full": UI_FULL_FEEDBACK_SERVICES,
}

def parse_services(services_arg: str) -> list[str]:
  return [service.strip() for service in services_arg.split(",") if service.strip()]


def parse_control_services(services_arg: str) -> list[str]:
  services = parse_services(services_arg)
  if "g29" in services and "turboSteerAssist" not in services:
    services.append("turboSteerAssist")
  return services


def split_control_services(services: list[str], udp_enabled: bool) -> tuple[list[str], list[str]]:
  if not udp_enabled:
    return [], list(services)

  udp_services = [service for service in services if udp_enabled and service in UDP_CONTROL_SERVICES]
  reliable_services = [service for service in services if service not in udp_services]
  if "g29" in services and TELEOP_COMMAND_SERVICE not in reliable_services:
    reliable_services.append(TELEOP_COMMAND_SERVICE)
  return udp_services, reliable_services


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


def model_v2_ui_projection(model: dict[str, Any]) -> dict[str, Any]:
  # The GCS debug UI only needs renderer fields; omit large model/debug fields
  # to keep the reliable ordered LTE data channel from backing up.
  def as_dict(data: Any) -> dict[str, Any]:
    return data if isinstance(data, dict) else {}

  def xyz(data: Any) -> dict[str, Any]:
    data = as_dict(data)
    return {axis: data.get(axis, []) for axis in ("x", "y", "z")}

  meta = as_dict(model.get("meta", {}))
  disengage_predictions = as_dict(meta.get("disengagePredictions", {}))
  acceleration = as_dict(model.get("acceleration", {}))

  return {
    "position": xyz(model.get("position", {})),
    "laneLines": [xyz(lane_line) for lane_line in model.get("laneLines", [])],
    "roadEdges": [xyz(road_edge) for road_edge in model.get("roadEdges", [])],
    "laneLineProbs": model.get("laneLineProbs", []),
    "roadEdgeStds": model.get("roadEdgeStds", []),
    "acceleration": {"x": acceleration.get("x", [])},
    "meta": {
      "disengagePredictions": {
        "brakeDisengageProbs": disengage_predictions.get("brakeDisengageProbs", []),
        "steerOverrideProbs": disengage_predictions.get("steerOverrideProbs", []),
      },
    },
  }


def project_feedback_message(service: str, msg_content: Any) -> Any:
  msg_dict = cereal_to_json(msg_content)
  if service == "modelV2" and isinstance(msg_dict, dict):
    return model_v2_ui_projection(msg_dict)
  return msg_dict


def cereal_message_payload(service: str, sm: messaging.SubMaster) -> bytes:
  msg = {
    "type": service,
    "logMonoTime": sm.logMonoTime[service],
    "valid": sm.valid[service],
    "data": project_feedback_message(service, sm[service]),
  }
  return json.dumps(msg).encode()


def udp_control_message_payload(service: str, sm: messaging.SubMaster) -> bytes:
  msg_data = project_feedback_message(service, sm[service])
  if service == "g29":
    msg_data = {**msg_data, **dict.fromkeys(G29_EDGE_FIELDS, False)}
  msg = {
    "type": service,
    "logMonoTime": sm.logMonoTime[service],
    "valid": sm.valid[service],
    "data": msg_data,
  }
  return json.dumps(msg).encode()


class CerealDataChannelReceiver:
  def __init__(self, services: list[str], pm: messaging.PubMaster | None = None):
    self.services = list(services)
    self.service_set = set(services)
    self.pm = messaging.PubMaster(self.services) if pm is None else pm
    self.received: dict[str, int] = dict.fromkeys(services, 0)
    self.ignored = 0
    self.out_of_order = 0
    self.last_log_mono_time: dict[str, int] = dict.fromkeys(services, 0)
    self.reassembler = FeedbackPacketReassembler()

  def receive(self, message: bytes | str) -> bool:
    if isinstance(message, bytes) and message.startswith(FEEDBACK_PACKET_MAGIC):
      try:
        assembled = self.reassembler.add(message)
      except (ValueError, zlib.error):
        self.ignored += 1
        return False
      if assembled is None:
        return True
      message = assembled

    payload = json.loads(message)
    if not isinstance(payload, dict):
      self.ignored += 1
      return False

    service = payload.get("type")
    if service not in self.service_set:
      self.ignored += 1
      return False

    log_mono_time = int(payload.get("logMonoTime", time.monotonic() * 1e9))
    if log_mono_time <= self.last_log_mono_time[service]:
      self.ignored += 1
      self.out_of_order += 1
      return False

    msg_data = payload.get("data")
    size = None
    if not isinstance(msg_data, dict):
      size = len(msg_data)

    msg = messaging.new_message(
      service,
      size=size,
      valid=bool(payload.get("valid", False)),
      logMonoTime=log_mono_time,
    )
    setattr(msg, service, msg_data)
    self.pm.send(service, msg)
    self.last_log_mono_time[service] = log_mono_time
    self.received[service] += 1
    return True


class CerealDatagramProtocol(asyncio.DatagramProtocol):
  def __init__(self, receiver: Callable[[], CerealDataChannelReceiver | None]):
    self.receiver = receiver
    self.received_packets = 0
    self.received_bytes = 0
    self.ignored_packets = 0

  def datagram_received(self, data: bytes, addr) -> None:
    receiver = self.receiver()
    if receiver is None:
      self.ignored_packets += 1
      return
    try:
      accepted = receiver.receive(data)
    except (TypeError, ValueError):
      accepted = False
    if accepted:
      self.received_packets += 1
      self.received_bytes += len(data)
    else:
      self.ignored_packets += 1


class UdpCerealChannel:
  label = "control-udp"
  bufferedAmount = 0

  def __init__(self, endpoint: tuple[str, int]):
    self.endpoint = endpoint
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.socket.connect(endpoint)

  def send(self, payload: bytes) -> None:
    self.socket.send(payload)

  def close(self) -> None:
    self.socket.close()


class CerealDataChannelSender:
  def __init__(
    self,
    services: list[str],
    channel,
    update_interval: float = 0.01,
    log_interval: float = 5.0,
    max_buffered_amount: int = 65536,
    log_label: str = "webrtc controls",
    payload_builder: Callable[[str, messaging.SubMaster], bytes] = cereal_message_payload,
  ):
    self.services = services
    self.channel = channel
    self.update_interval = update_interval
    self.log_interval = log_interval
    self.max_buffered_amount = max_buffered_amount
    self.log_label = log_label
    self.payload_builder = payload_builder
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
        self.channel.send(self.payload_builder(service, self.sm))
        self.sent[service] += 1

      now = time.monotonic()
      if now - last_log >= self.log_interval:
        sent_counts = " ".join(f"{service}={count}" for service, count in self.sent.items())
        skipped_counts = " ".join(f"{service}={count}" for service, count in self.skipped.items())
        print(
          " ".join((
            f"{self.log_label} sent {sent_counts}",
            f"skipped {skipped_counts}",
            f"buffered={self.buffered_amount()}",
            f"buffered_max={self.max_observed_buffered_amount}",
          )),
          flush=True,
        )
        self.max_observed_buffered_amount = self.buffered_amount()
        last_log = now

      await asyncio.sleep(self.update_interval)
