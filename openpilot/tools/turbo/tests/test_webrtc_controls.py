import json

import pytest

from openpilot.tools.turbo.webrtc_controls import CerealDataChannelReceiver, expand_feedback_services, model_v2_ui_projection


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


def test_expand_feedback_services_accepts_steer_assist_profile():
  assert expand_feedback_services("", "steer_assist") == [
    "carState",
    "selfdriveState",
    "carOutput",
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


def test_expand_feedback_services_ui_model_keeps_lte_default_small():
  assert expand_feedback_services("", "ui_model") == [
    "deviceState",
    "pandaStates",
    "selfdriveState",
    "carState",
    "controlsState",
    "roadCameraState",
    "wideRoadCameraState",
    "liveCalibration",
    "modelV2",
    "carParams",
    "liveParameters",
    "onroadEvents",
  ]


def test_expand_feedback_services_ui_full_keeps_optional_ui_services():
  services = expand_feedback_services("", "ui_full")

  assert "radarState" in services
  assert "longitudinalPlan" in services
  assert "driverMonitoringState" in services
  assert "carControl" in services


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


def test_model_v2_ui_projection_keeps_only_renderer_fields():
  model = {
    "position": {"x": [0.0], "y": [1.0], "z": [2.0], "t": [0.0]},
    "orientation": {"x": [3.0]},
    "velocity": {"x": [4.0]},
    "laneLines": [{"x": [1.0], "y": [2.0], "z": [3.0], "t": [4.0]}],
    "laneLineProbs": [0.9],
    "laneLineStds": [0.1],
    "roadEdges": [{"x": [5.0], "y": [6.0], "z": [7.0], "t": [8.0]}],
    "roadEdgeStds": [0.2],
    "acceleration": {"x": [0.1], "y": [0.2], "z": [0.3]},
    "leadsV3": [{"prob": 1.0}],
    "rawPredictions": "heavy",
    "meta": {
      "desireState": [0.0],
      "disengagePredictions": {
        "brakeDisengageProbs": [0.01],
        "steerOverrideProbs": [0.02],
        "gasDisengageProbs": [0.03],
      },
    },
  }

  projected = model_v2_ui_projection(model)

  assert projected == {
    "position": {"x": [0.0], "y": [1.0], "z": [2.0]},
    "laneLines": [{"x": [1.0], "y": [2.0], "z": [3.0]}],
    "roadEdges": [{"x": [5.0], "y": [6.0], "z": [7.0]}],
    "laneLineProbs": [0.9],
    "roadEdgeStds": [0.2],
    "acceleration": {"x": [0.1]},
    "meta": {
      "disengagePredictions": {
        "brakeDisengageProbs": [0.01],
        "steerOverrideProbs": [0.02],
      },
    },
  }


def test_cereal_data_channel_receiver_accepts_slim_model_v2():
  pm = FakePubMaster()
  receiver = CerealDataChannelReceiver(["modelV2"], pm=pm)
  payload = {
    "type": "modelV2",
    "logMonoTime": 123,
    "valid": True,
    "data": {
      "position": {"x": [0.0], "y": [1.0], "z": [2.0]},
      "laneLines": [{"x": [1.0], "y": [2.0], "z": [3.0]}],
      "roadEdges": [{"x": [4.0], "y": [5.0], "z": [6.0]}],
      "laneLineProbs": [0.7],
      "roadEdgeStds": [0.2],
      "acceleration": {"x": [0.1]},
      "meta": {
        "disengagePredictions": {
          "brakeDisengageProbs": [0.01],
          "steerOverrideProbs": [0.02],
        },
      },
    },
  }

  assert receiver.receive(json.dumps(payload).encode())

  service, msg = pm.sent[0]
  assert service == "modelV2"
  assert list(msg.modelV2.position.x) == [0.0]
  assert list(msg.modelV2.laneLines[0].y) == [2.0]
  assert list(msg.modelV2.acceleration.x) == pytest.approx([0.1])
  assert list(msg.modelV2.meta.disengagePredictions.brakeDisengageProbs) == pytest.approx([0.01])
  assert list(msg.modelV2.orientation.x) == []
  assert msg.modelV2.rawPredictions == b""
