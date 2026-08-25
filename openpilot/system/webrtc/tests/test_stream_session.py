import asyncio
import json
import time
# for aiortc and its dependencies
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning) # TODO: remove this when google-crc32c publish a python3.12 wheel

from aiortc import RTCDataChannel
from aiortc.mediastreams import VIDEO_CLOCK_RATE, VIDEO_TIME_BASE
import capnp
from openpilot.cereal import messaging, log

from openpilot.system.webrtc.webrtcd import CerealOutgoingMessageProxy, CerealIncomingMessageProxy
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

    channel = mocker.Mock(spec=RTCDataChannel)
    proxy = CerealOutgoingMessageProxy(["customReservedRawData0"])
    def mocked_update(t):
      proxy.sm.update_msgs(0, [test_msg])

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    proxy.add_channel(channel)

    proxy.update()

    channel.send.assert_called_once_with(expected_json)

  def test_outgoing_proxy_rate_limits_feedback(self, mocker):
    car_state_msg = messaging.new_message("carState")
    car_state_msg.logMonoTime = 123
    car_state_msg.valid = True
    car_state_msg.carState.vEgo = 1.5

    state_msg = messaging.new_message("selfdriveState")
    state_msg.logMonoTime = 456
    state_msg.valid = True
    state_msg.selfdriveState.enabled = True

    channel = mocker.Mock(spec=RTCDataChannel)
    proxy = CerealOutgoingMessageProxy(["selfdriveState", "carState"])

    def mocked_update(t):
      proxy.sm.update_msgs(0, [state_msg, car_state_msg])

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    mocker.patch("openpilot.system.webrtc.webrtcd.time.monotonic", side_effect=[100.0, 100.02, 100.06, 100.12])
    proxy.add_channel(channel)

    proxy.update()
    proxy.update()
    proxy.update()
    proxy.update()

    sent_types = [json.loads(call.args[0])["type"] for call in channel.send.call_args_list]
    assert sent_types == ["carState", "selfdriveState", "carState", "selfdriveState", "carState", "selfdriveState"]

  def test_outgoing_proxy_prioritizes_critical_feedback(self):
    proxy = CerealOutgoingMessageProxy(["modelV2", "controlsState", "deviceState", "selfdriveState", "carOutput", "carState"])

    assert proxy.services == ["carState", "selfdriveState", "carOutput", "controlsState", "modelV2", "deviceState"]

  def test_outgoing_proxy_keeps_pending_message_until_rate_limit_opens(self, mocker):
    car_state_msg = messaging.new_message("carState")
    car_state_msg.logMonoTime = 123
    car_state_msg.valid = True
    car_state_msg.carState.vEgo = 1.5

    channel = mocker.Mock(spec=RTCDataChannel)
    proxy = CerealOutgoingMessageProxy(["carState"])
    update_count = 0

    def mocked_update(t):
      nonlocal update_count
      update_count += 1
      if update_count <= 2:
        proxy.sm.update_msgs(0, [car_state_msg])
      else:
        proxy.sm.updated["carState"] = False

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    mocker.patch("openpilot.system.webrtc.webrtcd.time.monotonic", side_effect=[100.0, 100.049, 100.051])
    proxy.add_channel(channel)

    proxy.update()
    proxy.update()
    proxy.update()

    assert channel.send.call_count == 2

  def test_outgoing_proxy_keeps_pending_message_until_data_channel_drains(self, mocker):
    car_state_msg = messaging.new_message("carState")
    car_state_msg.logMonoTime = 123
    car_state_msg.valid = True
    car_state_msg.carState.vEgo = 1.5

    channel = mocker.Mock(spec=RTCDataChannel)
    channel.bufferedAmount = 20
    proxy = CerealOutgoingMessageProxy(["carState"], max_buffered_amount=10)
    update_count = 0

    def mocked_update(t):
      nonlocal update_count
      update_count += 1
      if update_count == 1:
        proxy.sm.update_msgs(0, [car_state_msg])
      else:
        proxy.sm.updated["carState"] = False

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    proxy.add_channel(channel)

    proxy.update()

    channel.send.assert_not_called()
    assert proxy.pending_send["carState"]
    assert proxy.skipped["carState"] == 1

    channel.bufferedAmount = 0
    proxy.update()

    channel.send.assert_called_once()
    assert not proxy.pending_send["carState"]

  def test_outgoing_proxy_feedback_stats_logging_is_opt_in(self, mocker):
    proxy = CerealOutgoingMessageProxy(["carState"])
    log_stats = mocker.patch.object(proxy, "log_stats")

    assert proxy.maybe_log_stats(now=106.0, last_log=100.0) == 100.0
    log_stats.assert_not_called()

  def test_outgoing_proxy_logs_feedback_stats_when_enabled(self, mocker):
    proxy = CerealOutgoingMessageProxy(["carState"], log_stats=True)
    log_stats = mocker.patch.object(proxy, "log_stats")

    assert proxy.maybe_log_stats(now=106.0, last_log=100.0) == 106.0
    log_stats.assert_called_once()

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

      mocked_pubmaster.send.assert_called_once()
      mt, md = mocked_pubmaster.send.call_args.args
      assert mt == msg["type"]
      assert isinstance(md, capnp._DynamicStructBuilder)
      assert hasattr(md, msg["type"])

      mocked_pubmaster.reset_mock()

  def test_incoming_proxy_preserves_message_metadata(self, mocker):
    mocked_pubmaster = mocker.MagicMock(spec=messaging.PubMaster)
    proxy = CerealIncomingMessageProxy(mocked_pubmaster)
    msg = {
      "type": "turboSteerAssist",
      "logMonoTime": 123,
      "valid": True,
      "data": {
        "active": True,
        "nudgeAngleDeg": 2.5,
      },
    }

    proxy.send(json.dumps(msg).encode())

    service, forwarded = mocked_pubmaster.send.call_args.args
    assert service == "turboSteerAssist"
    assert forwarded.valid
    assert forwarded.logMonoTime == 123
    assert forwarded.turboSteerAssist.active
    assert forwarded.turboSteerAssist.nudgeAngleDeg == 2.5

  def test_livestream_track(self, mocker):
    fake_msg = messaging.new_message("livestreamDriverEncodeData")

    config = {"receive.return_value": fake_msg.to_bytes()}
    mocker.patch("msgq.SubSocket", spec=True, **config)
    track = LiveStreamVideoStreamTrack("driver")

    assert track.id.startswith("driver")
    assert track.codec_preference() == "H264"

    for i in range(5):
      packet = self.loop.run_until_complete(track.recv())
      assert packet.time_base == VIDEO_TIME_BASE
      if i == 0:
        start_ns = time.monotonic_ns()
        start_pts = packet.pts
      assert abs(i + packet.pts - (start_pts + (((time.monotonic_ns() - start_ns) * VIDEO_CLOCK_RATE) // 1_000_000_000))) < 450 #5ms
      assert packet.size == 0
