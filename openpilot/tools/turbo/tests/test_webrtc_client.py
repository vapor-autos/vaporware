import asyncio

from openpilot.tools.turbo.webrtc_client import WebrtcdConnectionProvider
from teleoprtc import StreamingOffer


class FakeResponse:
  def raise_for_status(self):
    pass

  def json(self):
    return {"sdp": "answer-sdp", "type": "answer"}


def test_webrtcd_connection_provider_requests_feedback_services(monkeypatch):
  captured = {}

  def fake_post(url, json, timeout):
    captured["url"] = url
    captured["json"] = json
    captured["timeout"] = timeout
    return FakeResponse()

  monkeypatch.setattr("openpilot.tools.turbo.webrtc_client.requests.post", fake_post)

  provider = WebrtcdConnectionProvider("ugv.local", 5001, ["wideRoad"], feedback_services=["carState"])
  answer = asyncio.run(provider(StreamingOffer(sdp="offer-sdp", video=["wideRoad"])))

  assert answer.sdp == "answer-sdp"
  assert answer.type == "answer"
  assert captured["url"] == "http://ugv.local:5001/stream"
  assert captured["timeout"] == 10
  assert captured["json"]["sdp"] == "offer-sdp"
  assert captured["json"]["init_camera"] == "wideRoad"
  assert captured["json"]["cameras"] == ["wideRoad"]
  assert captured["json"]["bridge_services_in"] == []
  assert captured["json"]["bridge_services_out"] == ["carState"]
