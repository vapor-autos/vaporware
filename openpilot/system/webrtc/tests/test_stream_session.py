import asyncio
from dataclasses import asdict
import json
import time

import capnp
from openpilot.cereal import messaging, log
from teleoprtc.tracks import VIDEO_CLOCK_RATE

from openpilot.system.webrtc.helpers import StreamRequestBody
from openpilot.system.webrtc.webrtcd import CerealOutgoingMessageProxy, CerealIncomingMessageProxy, ServerState, handle_get_stream
from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack


class TestStreamSession:
  def setup_method(self):
    self.loop = asyncio.new_event_loop()

  def teardown_method(self):
    self.loop.stop()
    self.loop.close()

  def test_outgoing_proxy(self, mocker):
    test_msg = log.Event.new_message()
    test_msg.logMonoTime = 123
    test_msg.valid = True
    test_msg.customReservedRawData0 = b"test"
    expected_dict = {"type": "customReservedRawData0", "logMonoTime": 123, "valid": True, "data": "test"}
    expected_json = json.dumps(expected_dict).encode()

    channel = mocker.Mock()
    channel.is_open.return_value = True
    proxy = CerealOutgoingMessageProxy(["customReservedRawData0"])
    def mocked_update(t):
      proxy.sm.update_msgs(0, [test_msg])

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    proxy.add_channel(channel)

    proxy.update()

    channel.send.assert_called_once_with(expected_json)

  def test_incoming_proxy(self, mocker):
    tested_msgs = [
      {"type": "customReservedRawData0", "data": "test"}, # primitive
      {"type": "can", "data": [{"address": 0, "dat": "", "src": 0}]}, # list
      {"type": "testJoystick", "data": {"axes": [0, 0], "buttons": [False]}}, # dict
    ]

    mocked_pubmaster = mocker.MagicMock(spec=messaging.PubMaster)

    proxy = CerealIncomingMessageProxy(mocked_pubmaster)

    for msg in tested_msgs:
      proxy.send(json.dumps(msg).encode())

      deadline = time.monotonic() + 1.0
      while mocked_pubmaster.send.call_count == 0 and time.monotonic() < deadline:
        time.sleep(0.01)
      mocked_pubmaster.send.assert_called_once()
      mt, md = mocked_pubmaster.send.call_args.args
      msg_type = msg["type"]
      assert isinstance(msg_type, str)
      assert mt == msg_type
      assert isinstance(md, capnp._DynamicStructBuilder)
      assert hasattr(md, msg_type)

      mocked_pubmaster.reset_mock()

    proxy.close()

  def test_livestream_track(self, mocker):
    fake_msg = messaging.new_message("livestreamDriverEncodeData")

    config = {"receive.return_value": fake_msg.to_bytes()}
    mocker.patch("msgq.SubSocket", spec=True, **config)
    track = LiveStreamVideoStreamTrack("driver")

    assert track.id.startswith("driver")

    for i in range(5):
      packet = self.loop.run_until_complete(track.recv())
      if i == 0:
        start_ns = time.monotonic_ns()
        start_pts = packet.pts
      assert abs(i + packet.pts - (start_pts + (((time.monotonic_ns() - start_ns) * VIDEO_CLOCK_RATE) // 1_000_000_000))) < 450 #5ms
      assert bytes(packet) == b""


def test_data_only_stream_does_not_replace_video_stream(mocker):
  async def run_test():
    class FakeChannel:
      def send(self, _payload):
        pass

    class FakeStream:
      def get_messaging_channel(self):
        return FakeChannel()

    class FakeAnswer:
      sdp = "answer-sdp"
      type = "answer"

    class FakeSession:
      next_id = 0

      def __init__(self, body, _debug_mode=False):
        self.identifier = f"session-{FakeSession.next_id}"
        FakeSession.next_id += 1
        self.has_video = bool(body.cameras or body.init_camera)
        self.enabled = body.enabled
        self.stream = FakeStream()
        self.run_task = None

      async def get_answer(self):
        return FakeAnswer()

      def start(self):
        self.run_task = asyncio.create_task(asyncio.Event().wait())

      async def stop(self):
        if self.run_task is not None:
          self.run_task.cancel()
          await asyncio.gather(self.run_task, return_exceptions=True)

    mocker.patch("openpilot.system.webrtc.webrtcd.StreamSession", FakeSession)

    state = ServerState(debug=False)
    video_body = StreamRequestBody(
      sdp="video-offer",
      init_camera="wideRoad",
      enabled=True,
      cameras=["wideRoad"],
    )
    data_body = StreamRequestBody(
      sdp="data-offer",
      init_camera="",
      enabled=True,
      bridge_services_in=["g29"],
      cameras=[],
    )

    await handle_get_stream(state, json.dumps(asdict(video_body)).encode())
    await handle_get_stream(state, json.dumps(asdict(data_body)).encode())

    assert len(state.streams) == 2
    assert sorted(session.has_video for session in state.streams.values()) == [False, True]

    await asyncio.gather(*(session.stop() for session in state.streams.values()))

  asyncio.run(run_test())
