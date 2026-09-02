import asyncio

from openpilot.tools.turbo.webrtc_uplink import fetch_offer


def test_fetch_offer_extracts_udp_feedback_endpoint(mocker):
  response = mocker.Mock()
  response.status_code = 200
  response.json.return_value = {
    "sdp": "offer-sdp",
    "init_camera": "wideRoad",
    "enabled": True,
    "bridge_services_in": ["g29"],
    "bridge_services_out": ["carState"],
    "cameras": ["wideRoad", "driver"],
    "session_id": "session-1",
    "feedback_udp_host": "100.99.187.99",
    "feedback_udp_port": 8444,
    "control_udp_services": ["g29", "turboSteerAssist"],
  }
  get = mocker.patch("openpilot.tools.turbo.webrtc_uplink.requests.get", return_value=response)

  offer = asyncio.run(fetch_offer("http://gcs:8443", timeout=1.0))

  assert offer.session_id == "session-1"
  assert offer.feedback_udp_endpoint == ("100.99.187.99", 8444)
  assert offer.control_udp_services == ["g29", "turboSteerAssist"]
  assert offer.body.cameras == ["wideRoad", "driver"]
  get.assert_called_once_with("http://gcs:8443/offer", params={"control_udp_port": 8445}, timeout=1.0)
