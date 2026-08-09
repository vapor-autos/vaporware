import json

import pytest

from openpilot.tools.turbo.webrtc_controls import CerealDataChannelReceiver, expand_feedback_services


class AiortcChannel:
  bufferedAmount = 123


class FakePubMaster:
  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append((service, msg))


def test_cereal_data_channel_sender_reads_aiortc_buffered_amount():
  from openpilot.tools.turbo.webrtc_controls import CerealDataChannelSender

  sender = CerealDataChannelSender(["g29"], AiortcChannel())

  assert sender.buffered_amount() == 123


def test_expand_feedback_services_accepts_explicit_services():
  assert expand_feedback_services("carState, deviceState") == ["carState", "deviceState"]


def test_expand_feedback_services_accepts_ui_smoke_profile():
  assert expand_feedback_services("", "ui_smoke") == [
    "deviceState",
    "pandaStates",
    "selfdriveState",
    "carState",
    "controlsState",
    "roadCameraState",
    "wideRoadCameraState",
    "liveCalibration",
  ]


def test_expand_feedback_services_combines_and_deduplicates():
  assert expand_feedback_services("carState,modelV2", "torque,ui_smoke") == [
    "carState",
    "deviceState",
    "pandaStates",
    "selfdriveState",
    "controlsState",
    "roadCameraState",
    "wideRoadCameraState",
    "liveCalibration",
    "modelV2",
  ]


def test_expand_feedback_services_rejects_unknown_profile():
  with pytest.raises(ValueError, match="unknown feedback profile"):
    expand_feedback_services("", "unknown")


def test_cereal_data_channel_receiver_publishes_allowlisted_car_state():
  pm = FakePubMaster()
  receiver = CerealDataChannelReceiver(["carState"], pm=pm)
  payload = {
    "type": "carState",
    "logMonoTime": 123,
    "valid": True,
    "data": {
      "vEgo": 4.25,
      "vEgoRaw": 4.5,
      "standstill": False,
    },
  }

  assert receiver.receive(json.dumps(payload).encode())

  assert len(pm.sent) == 1
  service, msg = pm.sent[0]
  assert service == "carState"
  assert msg.valid
  assert msg.logMonoTime == 123
  assert msg.carState.vEgo == 4.25
  assert msg.carState.vEgoRaw == 4.5
  assert not msg.carState.standstill
  assert receiver.received["carState"] == 1


def test_cereal_data_channel_receiver_ignores_non_allowlisted_service():
  pm = FakePubMaster()
  receiver = CerealDataChannelReceiver(["carState"], pm=pm)
  payload = {
    "type": "deviceState",
    "logMonoTime": 123,
    "valid": True,
    "data": {},
  }

  assert not receiver.receive(json.dumps(payload))

  assert pm.sent == []
  assert receiver.ignored == 1
