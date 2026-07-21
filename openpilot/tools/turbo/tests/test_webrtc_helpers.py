import asyncio
import importlib
from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from openpilot.tools.turbo.webrtc_client import WebrtcdConnectionProvider
from openpilot.tools.turbo.webrtc_controls import CerealDataChannelSender
from openpilot.tools.turbo import webrtc_signald
from openpilot.tools.turbo.webrtc_signald import GcsAnswerProvider
from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.tools.turbo import webrtc_vipc_publisher
from teleoprtc import StreamingOffer
from teleoprtc.stream import RTCSessionDescription


def test_manager_webrtc_helpers_import_without_aiortc():
  for module in (
    "openpilot.tools.turbo.webrtc_vipc",
    "openpilot.tools.turbo.webrtc_signald",
    "openpilot.tools.turbo.webrtc_uplink",
  ):
    importlib.import_module(module)


def test_webrtcd_connection_provider_returns_teleoprtc_description(monkeypatch):
  captured = {}

  class Response:
    def raise_for_status(self):
      pass

    def json(self):
      return {"sdp": "answer-sdp", "type": "answer"}

  def post(url, json, timeout):
    captured.update({"url": url, "json": json, "timeout": timeout})
    return Response()

  monkeypatch.setattr("openpilot.tools.turbo.webrtc_client.requests.post", post)

  provider = WebrtcdConnectionProvider("192.0.2.1", 5001, ["wideRoad", "driver"])
  answer = asyncio.run(provider(StreamingOffer(sdp="offer-sdp", video=["wideRoad", "driver"])))

  assert answer == RTCSessionDescription(sdp="answer-sdp", type="answer")
  assert captured["url"] == "http://192.0.2.1:5001/stream"
  assert captured["json"]["init_camera"] == "wideRoad"
  assert captured["json"]["cameras"] == ["wideRoad", "driver"]
  assert captured["timeout"] == 10


def test_gcs_answer_provider_returns_teleoprtc_description():
  async def run_test():
    provider = GcsAnswerProvider("session", ["wideRoad"], ["teleopSendCan"])
    task = asyncio.create_task(provider(StreamingOffer(sdp="offer-sdp", video=["driver"])))

    await asyncio.wait_for(provider.offer_ready.wait(), timeout=1)
    assert provider.offer_body is not None
    assert provider.offer_body.init_camera == "driver"
    assert provider.offer_body.cameras == ["driver"]
    assert provider.offer_body.bridge_services_in == ["teleopSendCan"]

    provider.set_answer({"sdp": "answer-sdp", "type": "answer"})
    assert await task == RTCSessionDescription(sdp="answer-sdp", type="answer")

  asyncio.run(run_test())


def test_gcs_answer_provider_supports_data_only_offer():
  async def run_test():
    provider = GcsAnswerProvider("session", [], ["g29"])
    task = asyncio.create_task(provider(StreamingOffer(sdp="offer-sdp", video=[])))

    await asyncio.wait_for(provider.offer_ready.wait(), timeout=1)
    assert provider.offer_body is not None
    assert provider.offer_body.init_camera == ""
    assert provider.offer_body.cameras == []
    assert provider.offer_body.bridge_services_in == ["g29"]

    provider.set_answer({"sdp": "answer-sdp", "type": "answer"})
    assert await task == RTCSessionDescription(sdp="answer-sdp", type="answer")

  asyncio.run(run_test())


def test_signaling_state_serves_video_then_controls_offer(monkeypatch):
  async def run_test():
    class FakeProvider:
      def __init__(self, kind):
        self.offer_ready = asyncio.Event()
        self.offer_ready.set()
        self.answer_future = asyncio.get_running_loop().create_future()
        self.offer_body = StreamRequestBody(
          sdp=f"{kind}-offer",
          init_camera="wideRoad" if kind == "video" else "",
          enabled=True,
          bridge_services_in=[] if kind == "video" else ["g29"],
          cameras=["wideRoad"] if kind == "video" else [],
        )

      def set_answer(self, answer):
        self.answer_future.set_result(answer)

    class FakeSession:
      def __init__(self, _args, _cameras, kind):
        self.kind = kind
        self.session_id = kind
        self.provider = FakeProvider(kind)
        self.task = asyncio.get_running_loop().create_future()

      async def stop(self):
        self.task.cancel()

    args = SimpleNamespace(
      cameras="wideRoad",
      control_services="g29",
      synthetic_data_rate=0,
    )
    monkeypatch.setattr(webrtc_signald, "SignalingSession", FakeSession)

    state = webrtc_signald.SignalingState(args)
    video = await state.get_offer_session(offer_timeout=1)
    assert video.kind == "video"
    await state.set_answer("video", {"sdp": "video-answer", "type": "answer"})

    controls = await state.get_offer_session(offer_timeout=1)
    assert controls.kind == "controls"
    assert controls.provider.offer_body.bridge_services_in == ["g29"]

    await state.stop()

  asyncio.run(run_test())


def test_data_channel_sender_avoids_libdatachannel_state_queries():
  class Channel:
    def buffered_amount(self):
      raise AssertionError("libdatachannel buffered_amount should not be queried")

    def is_open(self):
      raise AssertionError("libdatachannel is_open should not be queried")

  sender = CerealDataChannelSender(["customReservedRawData0"], Channel())

  assert sender.buffered_amount() == 0
  assert sender.channel_open()


def test_h264_frame_receiver_decodes_queued_frames(monkeypatch):
  async def run_test():
    class FakeDecoder:
      def __init__(self, codec_name):
        assert codec_name == "h264"
        self.width = 640
        self.height = 480
        self.closed = False

      def decode(self, data):
        assert data == b"frame"
        return np.zeros(self.width * self.height * 3 // 2, dtype=np.uint8)

      def reset(self):
        pass

      def close(self):
        self.closed = True

    class FakeTrack:
      def __init__(self):
        self.keyframe_requests = 0

      def set_media_handler(self, _handler):
        pass

      def chain_media_handler(self, _handler):
        pass

      def on_frame(self, callback):
        self.callback = callback

      def request_keyframe(self):
        self.keyframe_requests += 1

      def is_open(self):
        return True

    monkeypatch.setattr(webrtc_vipc_publisher, "Decoder", FakeDecoder)

    track = FakeTrack()
    receiver = webrtc_vipc_publisher.H264FrameReceiver(cast(Any, track))
    track.callback(b"frame", None)

    frame = await asyncio.wait_for(receiver.recv(), timeout=1)
    assert frame.width == 640
    assert frame.height == 480
    assert frame.data.shape == (640 * 480 * 3 // 2,)
    assert track.keyframe_requests == 1

    receiver.close()

  asyncio.run(run_test())
