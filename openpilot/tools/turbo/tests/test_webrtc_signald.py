import asyncio

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
