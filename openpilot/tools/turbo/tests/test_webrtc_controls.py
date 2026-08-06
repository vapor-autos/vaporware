import json

from openpilot.tools.turbo.webrtc_controls import CerealDataChannelReceiver, data_channel_buffered_amount


class AiortcChannel:
  bufferedAmount = 123


class LibdatachannelChannel:
  def buffered_amount(self):
    return 456


class FakePubMaster:
  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append((service, msg))


def test_data_channel_buffered_amount_accepts_aiortc_property():
  assert data_channel_buffered_amount(AiortcChannel()) == 123


def test_data_channel_buffered_amount_accepts_libdatachannel_method():
  assert data_channel_buffered_amount(LibdatachannelChannel()) == 456


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
