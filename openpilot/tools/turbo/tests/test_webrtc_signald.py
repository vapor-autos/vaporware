import asyncio

from openpilot.tools.turbo.webrtc_controls import CerealDatagramProtocol
from openpilot.tools.turbo.webrtc_signald import GcsAnswerProvider
from teleoprtc import StreamingOffer


async def _run_provider_once():
  provider = GcsAnswerProvider(
    session_id="session-1",
    cameras=["wideRoad"],
    bridge_services_in=["g29"],
    bridge_services_out=["carState"],
  )

  task = asyncio.create_task(provider(StreamingOffer(sdp="offer-sdp", video=["wideRoad"])))
  await asyncio.wait_for(provider.offer_ready.wait(), timeout=1.0)
  assert provider.offer_body is not None
  offer_body = provider.offer_body
  provider.set_answer({"sdp": "answer-sdp", "type": "answer"})
  answer = await task
  return offer_body, answer


def test_gcs_answer_provider_requests_feedback_services():
  offer_body, answer = asyncio.run(_run_provider_once())

  assert answer.sdp == "answer-sdp"
  assert answer.type == "answer"
  assert offer_body.sdp == "offer-sdp"
  assert offer_body.init_camera == "wideRoad"
  assert offer_body.cameras == ["wideRoad"]
  assert offer_body.bridge_services_in == ["g29"]
  assert offer_body.bridge_services_out == ["carState"]


def test_feedback_datagram_protocol_dispatches_to_current_receiver(mocker):
  receiver = mocker.Mock()
  receiver.receive.return_value = True
  protocol = CerealDatagramProtocol(lambda: receiver)

  protocol.datagram_received(b"feedback", ("100.67.29.97", 12345))

  receiver.receive.assert_called_once_with(b"feedback")
  assert protocol.received_packets == 1
  assert protocol.received_bytes == 8
  assert protocol.ignored_packets == 0


def test_feedback_datagram_protocol_ignores_packets_without_session():
  protocol = CerealDatagramProtocol(lambda: None)

  protocol.datagram_received(b"feedback", ("100.67.29.97", 12345))

  assert protocol.received_packets == 0
  assert protocol.ignored_packets == 1
