# Camera Video Streaming Gameplan

Date: 2026-06-30

## Current Setup

Turbo mode is controlled by persistent params:

- `GCS`: enabled on the PC/GCS side by `openpilot/tools/turbo/up.py`
- `UGV`: enabled on the device side by the same script when running on TICI

The active process wiring lives in `openpilot/system/manager/process_config.py`.

### UGV Side

When `UGV` is true on-device:

- `camerad` runs through the normal `driverview` predicate while started.
- `stream_encoderd` also runs for `notCar`, even without `IsLiveStreaming`.
- `webrtcd` also runs for `notCar`.
- `turbo_ugv_camera_bridge` runs `openpilot/cereal/messaging/bridge` with no args, which exports local msgq services to ZMQ.
- `turbo_ugv_g29_bridge` imports `g29` from the GCS when `GCS_IP` is set.
- `teleopd` converts `g29` input into `teleopSendCan`.

Important point: the UGV already has the openpilot WebRTC livestream stack available in this mode. It is not only an Athena/offroad path.

### GCS Side

When `GCS` is true on PC:

- `turbo_gcs_control_bridge` runs `bridge` with no args and exports local GCS msgq services, including `g29`.
- `turbo_gcs_bridge` runs `bridge <TURBO_UGV_IP> <camera-service-list>` and imports encoded camera services from the UGV.
- `turbo_camerastream` runs `compressed_vipc.py 127.0.0.1 --cams <cams> --silent`.
- `gcs_ui` renders local VisionIPC streams from server name `camerad`.

`TURBO_GCS_CAMS` defaults to `1,2`, meaning driver plus wide road. The process config maps:

- `0` -> `roadEncodeData`
- `1` -> `driverEncodeData`
- `2` -> `wideRoadEncodeData`

`compressed_vipc.py` subscribes to those bridged encoded services, decodes HEVC, and republishes raw YUV frames into a local VisionIPC server named `camerad`. The current `gcs_ui.py` then renders:

- full-screen `VISION_STREAM_WIDE_ROAD`
- driver camera overlay from `VISION_STREAM_DRIVER`

This is a LAN msgq/ZMQ bridge plus local decode pipeline.

## Bridge Behavior

`openpilot/cereal/messaging/bridge.cc` has two modes:

- No args: export all local msgq endpoints to ZMQ.
- `bridge <ip> <whitelist>`: subscribe to ZMQ services at `<ip>` and republish matching services into local msgq.

The whitelist check is substring based against the single whitelist string. This is why process config passes comma-joined service names such as `driverEncodeData,wideRoadEncodeData`.

## Existing WebRTC Path

The WebRTC server is `openpilot/system/webrtc/webrtcd.py`.

It imports `teleoprtc.builder.WebRTCAnswerBuilder`, so teleoprtc is already part of the packaged openpilot environment. Root `pyproject.toml` includes `aiortc`, `av`, and packages `teleoprtc`.

The video track is `openpilot/system/webrtc/device/video.py::LiveStreamVideoStreamTrack`.

That track reads H264 livestream services:

- `driver` -> `livestreamDriverEncodeData`
- `wideRoad` -> `livestreamWideRoadEncodeData`
- `road` -> `livestreamRoadEncodeData`

This is different from the current Turbo bridge path, which imports normal encode services:

- `driverEncodeData`
- `wideRoadEncodeData`
- `roadEncodeData`

The livestream encoders are configured in `openpilot/system/loggerd/loggerd.h` as H264, low GOP (`gop_size = 5`), non-recording, and bitrate-controlled through `LivestreamEncoderBitrate`.

### WebRTC Transport Details

WebRTC does use RTP concepts for media, but it is not plain RTP/UDP:

- media is carried as SRTP, not raw RTP
- SRTP keys are negotiated through DTLS
- connection establishment uses ICE candidates and connectivity checks
- data channels use SCTP over DTLS
- signaling is still application-owned; in this tree it is the HTTP `POST /stream` endpoint on `webrtcd`

In the current `teleoprtc` code, `WebRTCBaseStream` constructs `aiortc.RTCPeerConnection()` without an explicit `RTCConfiguration`. With aiortc 1.14.0, that means `RTCConfiguration(iceServers=None)`: no STUN or TURN servers are configured by this code path.

`webrtcd.py` also patches `aioice.ice.get_host_addresses` so ICE advertises only the default-route interface when it can identify one. This is good for avoiding bad multi-interface candidates, but it matters for Tailscale: a normal Tailscale `100.x` address is often not the default route. If the GCS connects over Tailscale, we may need to adjust that patch to include the Tailscale address or make candidate interface selection configurable.

Practical implication:

- on the same LAN, current host-candidate ICE should work
- across arbitrary NAT without Tailscale, current code likely needs STUN/TURN configuration
- across Tailscale, current code may work only if the advertised ICE candidate is reachable from the GCS; otherwise patch candidate gathering first

Tailscale is still useful with WebRTC because it can remove the need for public STUN/TURN in the first deployment, but it is not automatically guaranteed by the current ICE candidate patch.

## teleoprtc Repo Findings

Local repo: `teleoprtc_repo`

Current commit:

```text
22df577 data channel double counting (#10)
```

teleoprtc provides:

- `WebRTCOfferBuilder`: client side; creates offers and requests incoming camera tracks.
- `WebRTCAnswerBuilder`: server side; answers an offer and attaches outgoing camera tracks.
- `TiciVideoStreamTrack`: track base with camera-aware IDs like `driver:<track-id>`.
- Data channel support for cereal-style JSON messages.

The examples are useful conceptually, but `examples/face_detection/face_detection.py` appears stale for this fork: it posts `cameras`, while this `webrtcd` expects `StreamRequestBody` fields `sdp`, `init_camera`, `enabled`, `bridge_services_in`, and `bridge_services_out`.

## Main Integration Gap

The current GCS UI needs simultaneous wide plus driver video.

The current `webrtcd` session creates exactly one `LiveStreamVideoStreamTrack` using `body.init_camera`. It supports camera switching over the data channel via `livestreamCameraSwitch`, but not simultaneous multi-camera output.

teleoprtc itself can request and identify multiple camera tracks, but the server wrapper in this openpilot tree is currently single-track.

Therefore, direct WebRTC integration needs one of these changes:

1. Extend `webrtcd` request schema to accept multiple cameras and add one video track per camera.
2. Use one WebRTC track and switch cameras, which is not enough for the current GCS layout.
3. Keep the existing bridge/VIPC path for the GCS and use WebRTC only for remote/browser streaming.

## Recommended Direction

Use WebRTC as the long-term camera transport for GCS, but do it in phases. Keep the current ZMQ/VIPC camera path as a known-good fallback while WebRTC is being proven.

Near-term codec choice should be H264, not H265/HEVC. The existing WebRTC path is already wired to the `livestream*EncodeData` H264 services, aiortc has working H264 packetization/depacketization, and WebRTC recovery behavior depends on keyframe/PLI/NACK flows that are already shaped around that path.

Keep H265 through the existing ZMQ plus `compressed_vipc.py` path as the LAN/high-quality fallback for now. Revisit H265-over-WebRTC only if H264 cannot hit the LTE quality/bitrate target.

## Transport Strategy

### WebRTC Versus Plain RTP

WebRTC media is not raw RTP/UDP. It is H264 packetized as RTP, protected as SRTP, negotiated over DTLS, and connected through ICE. It also gives us RTCP feedback:

- NACK: receiver asks sender to retransmit specific missing RTP packets
- PLI/FIR: receiver asks sender for a fresh keyframe when decode state is broken
- receiver reports: loss/jitter/RTT stats
- REMB-style bandwidth feedback in aiortc

The current openpilot WebRTC stack also has a simple adaptive bitrate controller that samples WebRTC stats and writes `LivestreamEncoderBitrate`; `encoderd` reads that param and updates the livestream encoder bitrate.

Pulling out the RTP stack would mean rebuilding or replacing loss detection, retransmission, PLI/keyframe recovery, bitrate feedback, and session setup. That may be worthwhile later for a very lean transport, but LTE is exactly where these feedback paths matter.

Working decision: do not pull out the RTP stack yet. Use WebRTC first and measure.

### Tailscale Role

Tailscale should be treated as a connectivity layer, not as the media protocol.

Desired path:

```text
GCS -> WebRTC -> Tailscale direct WireGuard UDP -> UGV
```

Undesired fallback:

```text
GCS -> WebRTC -> Tailscale DERP relay -> UGV
```

Tailscale direct is useful because it avoids public app exposure and can avoid needing STUN/TURN for the first deployment. It does not guarantee minimum latency unless the path is direct. DERP relay must be detected and treated as degraded or unusable for teleop.

Current caveat: `webrtcd.py` patches ICE candidate gathering to prefer the default-route IP. Tailscale is often not the default route, so WebRTC may advertise a LAN/Wi-Fi IP instead of the Tailscale `100.x` address. Add a configurable ICE host candidate override before relying on WebRTC over Tailscale:

```text
WEBRTC_ICE_HOST_IP=100.x.y.z
```

or:

```text
WEBRTC_ICE_INTERFACE=tailscale0
```

### Router Port Forward Role

Router port forwarding is useful as a direct no-relay benchmark when the UGV is behind a router we control.

It removes DERP/TURN relay variables, but it does not by itself guarantee low latency. It also does not solve carrier NAT if the UGV is directly on LTE behind CGNAT.

For WebRTC, forwarding only `webrtcd` HTTP port `5001` is not enough for fully deterministic direct media. The media UDP ports are selected by ICE. If we want router-port-forward WebRTC as a stable mode, we should investigate constraining aiortc/ICE UDP port ranges or exposing enough UDP range to make media candidate selection reliable.

### STUN/TURN Role

Current code has no explicit STUN/TURN config:

```python
aiortc.RTCPeerConnection()
```

with aiortc 1.14.0 defaults to:

```python
RTCConfiguration(iceServers=None)
```

So current ICE is basically host-candidate direct connectivity. It can work on LAN, routed VPN/Tailscale, or public direct IP. It will not reliably traverse arbitrary NATs. LTE/carrier NAT may require TURN if direct UDP hole punching fails.

Add STUN/TURN only after LAN and Tailscale-direct tests are understood.

## Test Matrix

All tests should log:

- connection mode selected
- RTT
- packet loss
- jitter
- NACK count
- PLI/keyframe request count
- selected bitrate rung
- actual encoded bitrate
- decode/render FPS
- end-to-end glass-to-glass latency when possible

### 1. LAN Direct Baseline

Purpose: prove WebRTC code and UI path without NAT complexity.

Path:

```text
GCS -> UGV LAN IP:5001 -> WebRTC host candidates -> direct LAN UDP
```

Expected:

- no STUN/TURN
- no Tailscale
- lowest practical latency baseline
- use H264 livestream services

Pass criteria:

- stable wideRoad stream
- then stable wideRoad plus driver after multi-camera support
- latency comparable enough to existing ZMQ/VIPC path to continue

### 2. Existing ZMQ/VIPC Baseline

Purpose: keep a known-good comparison.

Path:

```text
UGV normal HEVC encode -> msgq/ZMQ bridge -> GCS compressed_vipc.py -> local VIPC -> gcs_ui
```

Expected:

- simultaneous cameras already work
- H265/HEVC quality baseline
- no WebRTC recovery/adaptation behavior

Pass criteria:

- record CPU, latency, and visual quality versus WebRTC H264

### 3. Router Port Forward Direct

Purpose: direct public-IP no-relay benchmark when available.

Path:

```text
GCS -> public router IP -> forwarded webrtcd/media ports -> UGV
```

Work needed:

- verify whether aiortc UDP media ports can be constrained
- forward `5001` plus media UDP range if needed
- document firewall rules

Expected:

- no DERP
- no TURN
- direct internet path if the UGV router is controlled

Pass criteria:

- confirm selected ICE candidate pair uses the public forwarded path
- compare latency to LAN and Tailscale direct

### 4. Tailscale Direct

Purpose: practical remote connectivity without public exposure.

Path:

```text
GCS -> UGV Tailscale IP:5001 -> WebRTC host candidate over Tailscale direct
```

Work needed:

- add `WEBRTC_ICE_HOST_IP` or `WEBRTC_ICE_INTERFACE`
- use `tailscale ping <peer>` to confirm direct path
- gate teleop readiness on direct path or measured latency/jitter

Expected:

- direct WireGuard UDP
- no STUN/TURN required
- extra tunnel encapsulation, but likely acceptable if LTE path is stable

Pass criteria:

- `tailscale ping` reports direct, not DERP
- video stable at target bitrate
- latency acceptable for teleop

### 5. Tailscale DERP

Purpose: characterize degraded fallback, not preferred production path.

Path:

```text
GCS -> DERP relay -> UGV
```

Expected:

- higher latency
- possible congestion
- still useful to know failure mode

Pass criteria:

- decide whether to block teleop video, warn operator, or force low bitrate

### 6. WebRTC STUN Direct

Purpose: test direct WebRTC NAT traversal without Tailscale.

Work needed:

- add `RTCConfiguration` injection to teleoprtc/openpilot wrappers
- configure STUN server
- expose signaling path securely

Expected:

- may work on permissive NAT
- may fail on symmetric NAT or carrier CGNAT

Pass criteria:

- confirm direct candidate pair and compare to Tailscale direct

### 7. WebRTC TURN Relay

Purpose: guaranteed WebRTC connectivity fallback when direct fails.

Work needed:

- run or rent TURN server near expected operating region
- add TURN credentials/config
- measure relay latency and bandwidth cost

Expected:

- most reliable across LTE/carrier NAT
- may add latency similar in concept to Tailscale DERP

Pass criteria:

- determine if TURN is acceptable fallback for non-teleop viewing, low-bitrate teleop, or not acceptable

### Phase 1: Validate Direct WebRTC Video

Build a small GCS-side test client using `teleoprtc.WebRTCOfferBuilder`.

Behavior:

- Connect directly to `http://<TURBO_UGV_IP>:5001/stream`.
- Request one camera, probably `wideRoad` first.
- POST body should match current `StreamRequestBody`:

```json
{
  "sdp": "<offer-sdp>",
  "init_camera": "wideRoad",
  "enabled": true,
  "bridge_services_in": [],
  "bridge_services_out": ["carState", "deviceState"]
}
```

Goal:

- Confirm UGV `webrtcd` works in Turbo/UGV mode without Athena.
- Measure latency and stability against the existing `compressed_vipc.py` path.
- Confirm H264 packet path works on the GCS machine.
- Confirm NACK/PLI/keyframe behavior with induced packet loss if possible.

### Phase 2: Add Multi-Camera WebRTC Support

Extend the server request model with either:

- `cameras: list[str]`, keeping `init_camera` for backwards compatibility; or
- make `init_camera` a single default and add optional `extra_cameras`.

Server implementation:

- In `StreamSession.__init__`, create `LiveStreamVideoStreamTrack(camera, enabled)` for every requested camera.
- Call `builder.add_video_stream(camera, track)` for each.
- Keep camera switching only for the primary track, or disable switching when multiple fixed tracks are active.

Client implementation:

- `WebRTCOfferBuilder.offer_to_receive_video_stream("wideRoad")`
- `WebRTCOfferBuilder.offer_to_receive_video_stream("driver")`
- Receive both tracks by camera ID.

### Phase 3: Integrate With GCS UI

Add a new GCS video source abstraction instead of hardcoding `CameraView("camerad", ...)` everywhere.

Candidate sources:

- Existing source: VisionIPC via `compressed_vipc.py`
- New source: WebRTC incoming tracks via teleoprtc

Keep the current bridge/VIPC path as fallback behind an env or param:

- `TURBO_GCS_VIDEO_BACKEND=vipc`
- `TURBO_GCS_VIDEO_BACKEND=webrtc`

For the first WebRTC UI pass, avoid re-encoding back into VisionIPC unless needed. Decode incoming aiortc frames in the GCS process and upload them to the raylib texture path used by the UI. If that becomes invasive, a temporary WebRTC-to-VIPC adapter can mimic `compressed_vipc.py`, but that adds an extra decode/copy stage and partly defeats the transport cleanup.

### Phase 4: Move Control/Messaging Onto WebRTC Data Channels

After video is stable, evaluate moving `g29` and telemetry off the ZMQ bridges.

Current:

- GCS exports `g29` by no-arg bridge.
- UGV imports `g29` by `bridge <GCS_IP> g29`.

WebRTC option:

- Send `g29` messages over the teleoprtc data channel as JSON cereal messages.
- Add `g29` to `bridge_services_in` on the UGV-side `webrtcd` session.
- Use existing `CerealIncomingMessageProxy` to publish into local msgq.

This could remove the reverse bridge requirement and make NAT/firewall behavior cleaner.

### Phase 5: LTE Hardening

After LAN and direct-path tests pass, tune specifically for LTE:

- add more bitrate ladder rungs, for example `500k`, `800k`, `1.2M`, `1.8M`, `2.5M`, `3.5M`
- make bitrate thresholds configurable
- log WebRTC stats and openpilot encoder bitrate decisions
- verify PLI actually causes `LivestreamRequestKeyframe=True` for pre-encoded livestream tracks; patch if needed
- add operator-visible degraded-link state
- block or warn on Tailscale DERP/TURN when latency exceeds teleop threshold

## Open Questions

- Is the target GCS always on the same LAN as the UGV, or should this work across NAT/cellular?
- Do we need simultaneous `wideRoad` plus `driver` forever, or is camera switching acceptable in some modes?
- What latency budget is acceptable compared with the existing ZMQ plus `compressed_vipc.py` path?
- Should the WebRTC GCS client live inside `openpilot/tools/turbo/gcs_ui.py`, or as a separate daemon that feeds a cleaner video abstraction?
- Do we want browser compatibility, native GCS only, or both?

## Near-Term Tasks

1. Add a standalone `tools/turbo/webrtc_video_test.py` client that connects to UGV `webrtcd` and displays one camera.
2. Confirm the `webrtcd` lifecycle on UGV with `UGV=true`: `camerad`, `stream_encoderd`, and `webrtcd` should all be running while started.
3. Measure single-camera latency and CPU/GPU load against `compressed_vipc.py`.
4. Extend `StreamRequestBody` and `webrtcd.StreamSession` for multiple fixed cameras.
5. Add a GCS video backend switch and integrate WebRTC frames into `gcs_ui`.
6. Add configurable ICE host candidate selection for Tailscale/direct-interface tests.
7. Build the connection-mode test matrix: LAN, ZMQ/VIPC, router port forward, Tailscale direct, Tailscale DERP, STUN, TURN.
8. Only after video is solid, prototype `g29` over WebRTC data channel and remove one bridge from the test setup.

## Working Recommendation

Do not delete the current `compressed_vipc.py` bridge path yet. It is simple, already supports simultaneous cameras, and matches the current GCS UI.

The right next step is to prove direct UGV `webrtcd` with a one-camera teleoprtc GCS client, then extend the server to support multiple outgoing camera tracks. Once that works, the GCS UI can move from local VIPC camera views to WebRTC-backed camera views without losing the existing LAN fallback.

## Concrete Next Branch Plan

Branch: `webrtc-vid`

Current local working assumption:

- first tests are Wi-Fi LAN only
- no STUN/TURN
- no Tailscale
- no router port forwarding
- use H264 livestream encoder services
- use the existing openpilot `webrtcd` shape first
- GCS creates the WebRTC offer
- UGV `webrtcd` answers and sends video

### Important Architecture Note

The existing openpilot WebRTC daemon is device-server oriented:

```text
GCS/client creates offer -> UGV webrtcd answers -> UGV sends video
```

This already matches the minimum LAN test. Do this first.

The possible later Turbo shape is network-client oriented:

```text
UGV connects/posts to GCS -> GCS answers -> UGV sends video
```

That later inversion is not just a process-config flip. `teleoprtc.WebRTCOfferBuilder` currently supports requesting incoming video, while `WebRTCAnswerBuilder` supports attaching outgoing video tracks. For UGV-as-offerer with outgoing video, we would need one minimal extension:

- allow `WebRTCOfferBuilder` / `WebRTCOfferStream` to add outgoing video producer tracks; or
- build the first UGV sender directly with `aiortc.RTCPeerConnection` and reuse `LiveStreamVideoStreamTrack`

Do not extend teleoprtc until the existing direction proves the H264 video path and GCS decode/render path.

### Minimal Code Changes

#### 1. Add GCS WebRTC video test client

New file:

- `openpilot/tools/turbo/webrtc_video_test.py`

Behavior:

- runs on GCS
- creates a WebRTC offer with `teleoprtc.WebRTCOfferBuilder`
- requests one incoming camera, default `wideRoad`
- POSTs to `http://<UGV_IP>:5001/stream`
- uses current `StreamRequestBody` fields:
  - `sdp`
  - `init_camera`
  - `enabled`
  - `bridge_services_in`
  - `bridge_services_out`
- receives decoded frames from `stream.get_incoming_video_track(camera)`
- logs FPS, frame dimensions, and basic connection state

Expected result:

```python
builder = WebRTCOfferBuilder(WebrtcdConnectionProvider(host))
builder.offer_to_receive_video_stream("wideRoad")
stream = builder.stream()
```

#### 2. Local same-machine smoke test

Before UGV testing:

- run `openpilot.system.webrtc.webrtcd --debug` locally on a spare port
- run `webrtc_video_test.py --host 127.0.0.1 --port <port> --camera wideRoad`
- verify signaling, connection setup, and frame receive loop

This does not validate the H264 encoder path; it only validates the client and signaling flow.

#### 3. UGV LAN test

On UGV:

- branch `webrtc-vid`
- make sure `UGV=true`
- make sure started state brings up `camerad`, `stream_encoderd`, and `webrtcd`
- connect from GCS to `http://<UGV_LAN_IP>:5001/stream`
- request `wideRoad`

Expected result:

- GCS receives real H264 livestream frames from `livestreamWideRoadEncodeData`
- current ZMQ/VIPC path remains untouched

#### 4. Add multi-camera support

Only after single camera works:

- extend `StreamRequestBody` or add a Turbo-specific request field for multiple cameras
- update `webrtcd.StreamSession` to add one `LiveStreamVideoStreamTrack` per requested camera
- update `webrtc_video_test.py` to request `wideRoad` and `driver`

#### 5. GCS UI integration

After multi-camera works:

- integrate incoming WebRTC frames into `gcs_ui`
- keep existing `CameraView("camerad", ...)` path behind `TURBO_GCS_VIDEO_BACKEND=vipc`
- add `TURBO_GCS_VIDEO_BACKEND=webrtc`

#### 6. Process config, disabled by default first

File:

- `openpilot/system/manager/process_config.py`

Add only after manual scripts work:

- GCS process: WebRTC-backed GCS UI/client enabled by `GCS` and `TURBO_WEBRTC=1`
- UGV already has `webrtcd` through existing `notCar`/UGV path; avoid adding a new UGV sender until the later inverted-connectivity design is needed

#### 7. Minimal test order

1. LAN single camera, no process manager:
   - start or verify UGV `webrtcd`
   - run GCS `webrtc_video_test.py --camera wideRoad`
   - verify frames arrive
2. LAN dual camera:
   - extend server for multiple tracks
   - verify camera IDs and frame rates
3. WebRTC stats:
   - log RTT/loss/jitter/NACK/PLI where available
4. GCS UI integration:
   - either render frames directly from incoming tracks
   - or temporarily create a WebRTC-to-VIPC adapter if direct raylib integration takes longer
5. Process manager integration behind `TURBO_WEBRTC=1`

### Non-Goals For First Branch

- no STUN/TURN
- no Tailscale candidate override
- no router port-forward media range work
- no H265-over-WebRTC
- no removal of `compressed_vipc.py`
- no moving `g29` over WebRTC data channel yet
- no UGV-initiated outbound WebRTC yet

### First Success Definition

On Wi-Fi LAN:

- GCS test client connects to UGV `webrtcd`
- UGV answers the offer and sends H264 `wideRoad` from `livestreamWideRoadEncodeData`
- GCS receives decoded frames with stable FPS
- no ZMQ camera bridge or `compressed_vipc.py` required for that test
- existing Turbo ZMQ/VIPC path still works unchanged
