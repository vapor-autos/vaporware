# Controls WebRTC Gameplan

Date: 2026-07-07

## Goal

Get GCS controls working over LTE without relying on LAN-only `GCS_IP` reachability.

The preferred direction is to move the `g29` control stream onto the same WebRTC session used for video, using a WebRTC data channel. Keep the existing `teleopd` and CAN output path unchanged on the UGV.

## Current Controls Path

Current managed process wiring lives in `openpilot/system/manager/process_config.py`.

GCS side:

```text
g29d
  -> publishes msgq service: g29 @ 50 Hz

turbo_gcs_control_bridge
  -> runs ./bridge with no args
  -> exports local msgq services over ZMQ
```

UGV side:

```text
turbo_ugv_g29_bridge
  -> runs ./bridge <GCS_IP> g29
  -> imports g29 from the GCS ZMQ bridge into local msgq

teleopd
  -> subscribes local g29
  -> converts wheel/pedal/buttons into CAN messages
  -> publishes teleopSendCan
```

Final CAN path:

```text
teleopSendCan -> card/pandad path -> CAN bus
```

This works on LAN because the UGV can reach `GCS_IP` directly. It does not work for LTE if `GCS_IP` is a private LAN address like `192.168.x.x`.

## Existing WebRTC Data Channel Support

`openpilot/system/webrtc/helpers.py::StreamRequestBody` already has:

```python
bridge_services_in: list[str]
bridge_services_out: list[str]
```

In `openpilot/system/webrtc/webrtcd.py`:

- `bridge_services_in` creates a `CerealIncomingMessageProxy`
- incoming data-channel messages are routed through `message_handler`
- matching services are republished into local msgq through a dynamic `PubMaster`

For controls:

```text
bridge_services_in=["g29"]
```

means:

```text
GCS data channel -> UGV webrtcd -> local UGV msgq g29 -> teleopd
```

This is the direction we want.

## Options

### Option A: Port-Forward Current ZMQ Bridge

Use the existing bridge path and expose the GCS bridge over the router.

Pros:

- closest to the current LAN architecture
- likely minimal changes if the bridge ports are understood

Cons:

- exposes raw msgq/ZMQ bridge transport
- may require multiple forwarded ports depending on service sockets
- separate connectivity path from WebRTC video
- separate reconnect/liveness behavior from video
- not ideal for LTE/NAT security or debugging

Conclusion: keep as an emergency fallback, not the main plan.

### Option B: Tailscale Current ZMQ Bridge

Set `GCS_IP` to the GCS Tailscale IP and keep the current bridge.

Pros:

- minimal code changes
- good development/debug fallback
- avoids public bridge exposure

Cons:

- latency depends on Tailscale direct versus DERP relay
- controls still have a separate transport from video
- does not prove the production LTE/WebRTC architecture

Conclusion: useful fallback, but not the primary LTE controls solution.

### Option C: WebRTC Data Channel For `g29`

Send `g29` over the existing WebRTC data channel in the video session.

Pros:

- one signaling/session lifecycle for video and controls
- one ICE-selected path through LTE/NAT
- no extra port-forwarded ZMQ surface
- reuses existing `webrtcd` cereal bridge support
- keeps UGV `teleopd` unchanged

Cons:

- need GCS-side sender for local `g29` over data channel
- need to verify data-channel latency/reliability behavior under LTE loss
- control stream currently shares the video PeerConnection/session

Conclusion: preferred direction.

## Recommended Architecture

```text
GCS:
  g29d publishes local msgq g29
  turbo_webrtc_signald creates WebRTC offer
  signald requests bridge_services_in=["g29"]
  signald starts a data-channel sender for local g29

UGV:
  turbo_webrtc_uplink fetches offer from GCS
  StreamSession creates CerealIncomingMessageProxy for g29
  incoming data-channel g29 is republished into local msgq
  teleopd consumes local g29 and publishes teleopSendCan
```

Video and controls share:

```text
one PeerConnection
one ICE selected candidate pair
one DTLS/SRTP/SCTP session
```

The control data channel should send only the `g29` service at first. Do not bridge arbitrary services for the first LTE controls test.

## Rebase Follow-Up: Libdatachannel Isolation Results

Status: the shared-video-session data-channel plan above is not viable as-is after the upstream `libdatachannel` rebase.

Observed after updating the GCS/UGV stack to the rebased WebRTC path:

- Two-camera video with no control data channel is stable.
- Full G29-over-data-channel with video connects, then video and controls stop after a short period.
- Stack dumps show the GCS sender thread blocked inside `channel.send(...)` with no matching send completion log.
- Changing G29 payloads from bytes to JSON strings did not fix it.
- Using unreliable/unordered `DataChannelInit` did not fix it.
- Polling native `buffered_amount()` is unsafe in this binding path; it segfaulted the GCS signald process in live testing.
- Moving UGV receive-side msgq publishing off the libdatachannel callback helped but did not eliminate the failure.

Synthetic data-channel tests isolated the failure away from G29/cereal:

```text
two cameras, 50 Hz, 256 B synthetic: wedges at begin=260/end=259
two cameras, 10 Hz, 256 B synthetic: wedges at begin=80/end=79
two cameras, 10 Hz, 64 B synthetic: wedges at begin=64/end=63
two cameras, 10 Hz, 64 B synthetic, unreliable/unordered: wedges at begin=50/end=49
one camera, 10 Hz, 64 B synthetic: wedges later at begin=256/end=255
video disabled, 10 Hz, 64 B synthetic: survived bounded run at begin=727/end=727 with health green
one camera, 1 Hz, 64 B synthetic: survived bounded run at begin=135/end=135 with health green
```

Interpretation:

- This is not a G29 schema/serialization problem.
- This is not caused by the local H264 decode/VIPC publisher.
- This is not fixed by unreliable/unordered data-channel negotiation.
- The failure correlates with data-channel send cadence while RTP video is active in the same `libdatachannel` `PeerConnection`.
- More active video streams make the failure happen sooner.
- The old aiortc stack hid this because media and data-channel scheduling/backpressure lived in Python asyncio and aiortc's buffering model. The rebased stack uses native synchronous `libdatachannel` calls; under shared RTP/SCTP pressure, `DataChannel.send()` can block indefinitely.

Updated fix direction:

Keep WebRTC data channels, but do not put high-frequency controls on the same `PeerConnection` as active camera RTP.

Preferred next architecture:

```text
GCS video session:
  turbo_webrtc_signald requests wideRoad,driver video
  no high-rate control bridge on this PeerConnection

GCS controls session:
  second WebRTC data-channel-only PeerConnection
  sends g29 over data channel

UGV:
  webrtcd supports data-channel-only StreamRequestBody with bridge_services_in=["g29"]
  CerealIncomingMessageProxy republishes g29 into local msgq
  teleopd remains unchanged
```

Implementation plan:

1. Add data-only stream support. Complete locally.
   - Allow `StreamRequestBody.cameras=[]` and `init_camera=""` for data-channel-only requests.
   - In `StreamSession`, skip `LiveStreamVideoStreamTrack` creation when no cameras are requested.
   - Keep bitrate/stats/video cleanup tolerant of zero video tracks.
   - `webrtcd` now keeps active video sessions when a data-only stream is added, and only replaces existing sessions of the same video/data kind.

2. Teach GCS signald to create a separate control session. Complete locally.
   - Build the current video session without `bridge_services_in`.
   - Build a second `WebRTCOfferBuilder` with only messaging enabled and `DataChannelInit` unreliable/unordered.
   - Send the second offer through the same HTTP signaling path and let `turbo_webrtc_uplink` post it to UGV `webrtcd`.
   - Start `CerealDataChannelSender(["g29"], control_stream.get_messaging_channel())` only on the control stream.
   - `turbo_webrtc_signald` now serves separate pending offers for `kind=video` and `kind=controls`.
   - `turbo_webrtc_uplink` now posts an answer, starts that session, and continues polling for more offers instead of waiting forever on the first session.

3. Keep the receive callback nonblocking. Complete locally.
   - Retain the UGV `CerealIncomingMessageProxy` worker queue.
   - Do JSON parsing/msgq publish off the libdatachannel receive callback path.

4. Validate in stages.
   - Synthetic 10 Hz and 20 Hz over data-only PeerConnection while two-camera video runs on the video PeerConnection.
   - Real `g29` controls over data-only PeerConnection while two-camera video runs.
   - Confirm UGV local `g29` updates, `teleopd` consumes it, and `teleopSendCan` publishes.
   - Full stack steering/video smoke on LAN before LTE.

Local verification after implementation:

```text
ruff check openpilot/tools/turbo/webrtc_signald.py openpilot/tools/turbo/webrtc_uplink.py openpilot/system/webrtc/webrtcd.py openpilot/tools/turbo/tests/test_webrtc_helpers.py openpilot/system/webrtc/tests/test_stream_session.py
pytest openpilot/tools/turbo/tests/test_webrtc_helpers.py openpilot/system/webrtc/tests/test_stream_session.py
```

Result:

```text
ruff: passed
pytest: 11 passed
```

## Implementation Plan

Short staged diff plan:

1. Add a reusable WebRTC data-channel sender for local cereal/msgq services.
   - First target: `g29` only.
   - No process config changes.
   - Test with lint/compile and a fake channel.

2. Wire the sender into `turbo_webrtc_signald`.
   - Add `TURBO_GCS_WEBRTC_CONTROL_SERVICES`, initially defaulting to empty or `g29` only after validation.
   - Include requested services in `StreamRequestBody.bridge_services_in`.
   - Start sender after data channel opens.
   - Test on LAN without changing UGV process config.

3. Prove UGV receives `g29` over WebRTC.
   - Run GCS signald and UGV uplink on LAN.
   - Confirm local UGV `g29` updates.
   - Confirm `teleopd` logs and `teleopSendCan` publishes.

4. Disable duplicate ZMQ `g29` bridge in WebRTC-controls mode.
   - Gate `turbo_ugv_g29_bridge` off when UGV is using outbound WebRTC signaling, or add an explicit UGV env flag if needed.
   - Test managed process lists.

5. Test full LTE path.
   - GCS port-forwarded signaling.
   - UGV LTE boot.
   - Video plus `g29` data channel.
   - Verify no public ZMQ bridge ports are required.

6. Add stale-control protection if LTE buffering shows up.
   - Watch data-channel `bufferedAmount`.
   - Drop stale `g29` samples instead of queueing old controls.
   - Re-test LTE control feel.

1. Add a small reusable GCS data-channel sender.

   Candidate helper:

   ```text
   openpilot/tools/turbo/webrtc_controls.py
   ```

   Responsibilities:

   - subscribe to local msgq service `g29`
   - convert capnp message to JSON in the same shape `CerealIncomingMessageProxy` already expects
   - send over the WebRTC messaging channel
   - log send rate and stale/disconnect errors

2. Update GCS signaling offer body.

   In `webrtc_signald.py`, include:

   ```python
   bridge_services_in=["g29"]
   ```

   in the `StreamRequestBody` returned to the UGV.

3. Start the GCS `g29` data-channel sender after connection.

   In `SignalingSession.run()`:

   - wait for WebRTC connection
   - get messaging channel
   - send livestream quality
   - start `g29` sender task
   - keep publishing video to VIPC as today
   - cancel sender task on disconnect

4. Keep UGV `teleopd` unchanged.

   `teleopd` should not care whether local `g29` came from ZMQ bridge or WebRTC data channel.

5. Avoid duplicate control sources during LTE tests.

   Options:

   - unset `GCS_IP` on UGV for WebRTC controls tests so `turbo_ugv_g29_bridge` does not run
   - or gate `turbo_ugv_g29_bridge` off when `GCS_SIGNALING_URL` is set

   Prefer explicit process-config gating after LAN validation.

6. LAN validation.

   Test on LAN before LTE:

   ```text
   GCS signald + g29d running
   UGV uplink connects
   UGV local msgq receives g29
   teleopd logs steering/accel/reverse values
   teleopSendCan is published
   ```

7. LTE validation.

   Test with:

   ```text
   UGV LTE boot
   GCS port-forwarded signaling URL
   WebRTC video + g29 data channel
   no ZMQ g29 bridge
   ```

   Verify:

   - `g29` receive rate on UGV near 50 Hz
   - `teleopd` logs sane values
   - control latency feels acceptable
   - video stays connected
   - data-channel reconnect follows video reconnect

## Open Questions

- Should data-channel controls use reliable ordered mode or unreliable/unordered mode?

  Current teleoprtc data-channel path likely uses default reliable/ordered behavior. That is acceptable for first validation because `g29` is small and 50 Hz, but stale controls are undesirable under packet loss. Later, consider an unordered or max-lifetime data channel if teleoprtc exposes it cleanly.

- Should `g29` be rate-limited or stale-dropped on the GCS side?

  For first pass, send updated `g29` at source rate. If data-channel buffering grows under LTE loss, add stale dropping based on `bufferedAmount` or latest-sample sending.

- Should controls be separate from video PeerConnection?

  Not initially. A single PeerConnection is simpler and shares the same ICE-selected LTE path. Reconsider only if video congestion harms data-channel latency.

## Success Criteria

- UGV receives `g29` over WebRTC data channel with no `GCS_IP` LAN dependency.
- `teleopd` and `teleopSendCan` work unchanged.
- Video and controls reconnect together through the existing GCS signaling path.
- No public ZMQ bridge ports are required for LTE controls.

## Progress

### 2026-07-07 First WebRTC Controls Test

Branch:

```text
webrtc-controls
```

Implemented:

- `openpilot/tools/turbo/webrtc_controls.py`
  - serializes local cereal/msgq services to the JSON shape accepted by `webrtcd.CerealIncomingMessageProxy`
  - sends selected services over an open WebRTC data channel
- `webrtc_signald.py`
  - adds `TURBO_GCS_WEBRTC_CONTROL_SERVICES`
  - includes selected services in `StreamRequestBody.bridge_services_in`
  - starts the sender after WebRTC connection
- `process_config.py`
  - disables `turbo_ugv_g29_bridge` when `GCS_SIGNALING_URL` is set

Validation:

- local `g29` JSON roundtrip through `CerealIncomingMessageProxy` passed
- focused `py_compile` and `ruff` passed
- `FAST=1 scripts/lint/lint.sh` passed
- GCS/UGV managed test connected video plus controls:
  - GCS log showed `controls=g29`
  - GCS sent about 50 Hz: `webrtc controls sent g29=...`
  - UGV local msgq observed over 5 seconds:

```text
g29: 247 messages
teleopSendCan: 247 messages
```

Important test config:

- GCS `.env` added `TURBO_GCS_WEBRTC_CONTROL_SERVICES=g29`
- UGV `.env` had `GCS_IP` removed for the test, so the old ZMQ `g29` bridge was not running

Next:

- reboot UGV after pulling the process-config gate if testing with `GCS_IP` present
- run a full LTE boot test with `GCS_IP` restored to confirm the new gate prevents duplicate ZMQ controls
- add data-channel buffering/stale-control protection if LTE control latency shows queued samples

### 2026-07-07 Boot Gate And Longer Controls Test

Branch:

```text
webrtc-controls
```

Commits:

```text
0b67313d8 Disable ZMQ g29 bridge in WebRTC uplink mode
80b32d7b7 Guard WebRTC control sends on channel backpressure
```

UGV boot test:

- restored `GCS_IP=192.168.50.17` in UGV `.env`
- rebooted UGV and let openpilot start from normal boot path
- verified `GCS_SIGNALING_URL` and `GCS_IP` were both present
- verified from boot:

```text
webrtcd running
webrtc_uplink running
teleopd running
turbo_ugv_g29_bridge not running
```

This confirms the process-config gate disables the old ZMQ `g29` bridge when outbound WebRTC signaling is active.

Longer GCS/UGV test:

- launched stock GCS `launch_openpilot.sh`
- GCS connected video and controls:

```text
connected cameras=wideRoad,driver,road
quality=low
controls=g29
```

- GCS data-channel sender logged about 50 Hz:

```text
webrtc controls sent g29=245
webrtc controls sent g29=492
webrtc controls sent g29=740
```

- UGV received over a 30 second sample:

```text
g29: 1482 messages, 49.4 Hz
teleopSendCan: 1483 messages, 49.5 Hz
last_g29 steering=-0.080 accel=-0.859 reverse=-1.000
```

Interpretation:

- WebRTC data-channel controls worked with the old ZMQ bridge disabled.
- `teleopd` converted received `g29` into `teleopSendCan` at the expected rate.
- There was no evidence of dropped or queued controls in this sample.

Backpressure guard added:

- control sender now logs current and max observed data-channel `bufferedAmount`
- skips sending controls while `bufferedAmount` is above `TURBO_GCS_WEBRTC_CONTROL_MAX_BUFFERED_AMOUNT`
- default threshold is `65536` bytes
- set `TURBO_GCS_WEBRTC_CONTROL_MAX_BUFFERED_AMOUNT=0` to disable skipping

Next:

- run another GCS/UGV test after the guard commit and watch:

```text
buffered=...
buffered_max=...
skipped g29=...
```

- if `buffered_max` stays near zero and `skipped g29=0`, keep the default reliable/ordered data channel for now
- if buffering/skips appear during LTE driving, evaluate unordered/partial-reliability data-channel options

## Managed bridge removal

Branch:

```text
webrtc-controls
```

Commit:

```text
d398ccc74 Remove managed Turbo ZMQ bridges
```

Change:

- removed managed `turbo_gcs_control_bridge`
- removed managed `turbo_ugv_camera_bridge`
- removed managed `turbo_ugv_g29_bridge`
- left the bridge tools/binaries in-tree for manual debugging or rollback

UGV boot validation:

- pulled latest branch on UGV
- rebooted UGV and let stock boot launch openpilot
- verified running processes included:

```text
manager.py
./encoderd --stream
openpilot.system.webrtc.webrtcd
openpilot.tools.turbo.webrtc_uplink
openpilot.tools.turbo.teleopd
./camerad
```

- verified no managed Turbo bridge processes were running

GCS validation:

- launched stock GCS `launch_openpilot.sh`
- verified managed processes included WebRTC signaling/UI pieces, but no `turbo_gcs_control_bridge`
- connected all three video streams:

```text
wideRoad
driver
road
```

- controls still arrived on UGV:

```text
g29: 494 messages, 49.5 Hz
teleopSendCan: 494 messages, 49.5 Hz
```

Backpressure result:

```text
skipped g29=0
buffered_max=283
```

Interpretation:

- WebRTC video and WebRTC data-channel controls are now the managed/default path for this branch.
- The old ZMQ bridges are no longer started by process manager, which removes the duplicate/fallback streaming path from normal launches.
