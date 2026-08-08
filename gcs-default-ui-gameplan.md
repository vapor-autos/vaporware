# GCS Default UI Gameplan

Goal: run the standard openpilot onroad UI on the GCS bottom screen while keeping the Turbo teleop UI on the top screen. The bottom UI should show the standard onroad overlays such as HUD speed, alerts, model path, lane lines, leads, and driver-monitoring state.

Bandwidth constraint: avoid sending the narrow `road` camera unless we prove we need it. Prefer reusing the existing `wideRoad` video stream for both Turbo UI and the standard UI bottom screen.

## Current State

The repo already has two GCS UI processes in `openpilot/system/manager/process_config.py`:

- `gcs_ui`: `openpilot.tools.turbo.gcs_teleop_ui`
- `gcs_debug_ui`: `openpilot.tools.turbo.gcs_debug_ui`

These are GCS-specific wrappers around the existing UIs. Keep this split local to Turbo/GCS code; do not change core `openpilot/system/ui/lib/application.py` or generic manager `process.py` for monitor placement.

The current local `.env` should not block the standard UI:

```text
BLOCK=manage_athenad
```

The local GCS display layout is:

```text
HDMI-1-0  1920x1080+0+0     Turbo teleop UI
eDP-1     1920x1080+0+1080  standard debug UI
```

GNOME/Mutter snaps normal raylib/GLFW full-size windows back onto the top display when they are mapped normally. The working approach is:

1. Use a GCS wrapper to create the raylib window hidden and undecorated.
2. Find the target monitor by name with raylib monitor discovery.
3. Before showing the window, set the X11 client to `override_redirect`.
4. Move/resize it directly to the target monitor rectangle.
5. Clear `FLAG_WINDOW_HIDDEN` and render normally.

This was validated locally with:

```text
Turbo GCS  1920x1080+0+0
UI         1920x1080+0+1080
```

The Turbo WebRTC path already supports UGV-to-GCS msgq feedback over the data channel:

- GCS requests services with `bridge_services_out` in `StreamRequestBody`.
- UGV `webrtcd` creates `CerealOutgoingMessageProxy(body.bridge_services_out)`.
- GCS receives those messages with `CerealDataChannelReceiver` and republishes them into local msgq.

This is how `carState` now reaches `g29d`.

## Camera Transport

The standard onroad UI does not receive camera pixels through msgq. It uses VisionIPC:

- `openpilot/selfdrive/ui/onroad/cameraview.py`
- `VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, conflate=True)`

The standard big UI defaults to `road`:

- `AugmentedRoadView` defaults to `VisionStreamType.VISION_STREAM_ROAD`.
- `_switch_stream_if_needed()` targets `ROAD_CAM` unless `experimentalMode` is true and `wideRoad` is available.
- In experimental mode, it switches to `wideRoad` below 10 m/s and back to `road` above 15 m/s.

So simply requesting only `wideRoad` is not enough. If the UI still starts with `ROAD_CAM`, it will try to connect to a local `road` VisionIPC stream that does not exist and will show the placeholder.

Today `.env` requests:

```text
TURBO_GCS_WEBRTC_CAMS=wideRoad,driver
```

The earlier bandwidth-expensive option was:

```text
TURBO_GCS_WEBRTC_CAMS=road,wideRoad,driver
```

That lets:

- Turbo top UI keep using `wideRoad` plus `driver`.
- Standard bottom UI use the default narrow `road` stream.
- Standard UI still switch to wide in experimental low-speed mode if `wideRoad` is available.

For the bandwidth-first path, keep:

```text
TURBO_GCS_WEBRTC_CAMS=wideRoad,driver
```

and add a GCS-specific standard UI override so the bottom standard UI starts on `wideRoad` and does not switch back to `road`.

Possible implementation shapes:

1. Add an env var such as `TURBO_GCS_STANDARD_UI_CAMERA=wideRoad`.
2. When `GCS=True`, instantiate the standard onroad `AugmentedRoadView` with `VisionStreamType.VISION_STREAM_WIDE_ROAD`.
3. In GCS wide-only mode, bypass `_switch_stream_if_needed()` or make it keep `WIDE_CAM`.

This should be a local GCS/UI behavior change, not a change to normal in-car openpilot behavior.

### Is Wide Road Cropped?

Yes, in the current standard big UI it is effectively cropped/zoomed when rendered through `AugmentedRoadView`.

In `openpilot/selfdrive/ui/onroad/augmented_road_view.py`:

```python
intrinsic = device_camera.ecam.intrinsics if is_wide_camera else device_camera.fcam.intrinsics
zoom = 2.0 if is_wide_camera else 1.1
```

`CameraView` then uses that matrix to draw the frame. When the transformed frame is larger than the viewport, the scissor/clipping region crops it. So `wideRoad` is not shown as the full raw fisheye/wide frame in the standard big UI; it is rectified/zoomed for the augmented road view.

That may be acceptable for the bottom standard UI because the goal is normal onroad context with overlays, not full teleop situational awareness. The Turbo top UI still shows `wideRoad` directly with its own `CameraView`, so it remains the main teleop view.

## Main UI Msgq Subscriptions

The canonical standard UI service list is in `openpilot/selfdrive/ui/ui_state.py`.

`ui_state.sm` subscribes to:

```text
modelV2
controlsState
onroadEvents
liveCalibration
radarState
deviceState
pandaStates
carParams
driverMonitoringState
carState
driverStateV2
roadCameraState
wideRoadCameraState
managerState
selfdriveState
longitudinalPlan
gpsLocationExternal
carOutput
carControl
liveParameters
testJoystick
rawAudioData
```

Service metadata from `openpilot/cereal/services.py`:

```text
modelV2                  20 Hz
controlsState           100 Hz
onroadEvents              1 Hz
liveCalibration           4 Hz
radarState               20 Hz
deviceState               2 Hz
pandaStates              10 Hz
carParams              0.02 Hz
driverMonitoringState    20 Hz
carState                100 Hz
driverStateV2            20 Hz
roadCameraState          20 Hz
wideRoadCameraState      20 Hz
managerState              2 Hz
selfdriveState          100 Hz
longitudinalPlan         20 Hz
gpsLocationExternal      10 Hz
carOutput               100 Hz
carControl              100 Hz
liveParameters           20 Hz
testJoystick              0 Hz
rawAudioData             20 Hz
```

Do not blindly bridge all of these at full rate at first. Several are high-rate, and `modelV2` can be large once converted to JSON.

## What The Onroad UI Actually Needs

### Required To Enter Onroad

`UIState` only considers the UI onroad when:

```python
ui_state.started = deviceState.started and ignition
```

Ignition is derived from `pandaStates`.

Minimum services to make the standard UI switch into onroad mode:

```text
deviceState
pandaStates
selfdriveState
carState
```

### Required For Camera View And Calibration

The camera pixels come from VisionIPC, but camera geometry and calibration come from msgq:

```text
roadCameraState
wideRoadCameraState
liveCalibration
deviceState
```

`deviceState.deviceType` and `roadCameraState.sensor` select the camera intrinsics. `liveCalibration` sets the model overlay transform.

Even in wide-only mode, keep bridging both `roadCameraState` and `wideRoadCameraState` initially. The current calibration/camera selection path still reads `roadCameraState.sensor` for the device camera config, and wide calibration is carried in `liveCalibration.wideFromDeviceEuler`.

### Required For HUD

`openpilot/selfdrive/ui/onroad/hud_renderer.py` uses:

```text
carState
controlsState
selfdriveState
```

`carState` provides current speed and cluster speed. `controlsState` provides cruise/set speed. `selfdriveState` drives status and the experimental button.

### Required For Alerts

`openpilot/selfdrive/ui/onroad/alert_renderer.py` uses:

```text
selfdriveState
```

This includes alert text, size, status, and enabled state.

### Required For Model Path And Leads

`openpilot/selfdrive/ui/onroad/model_renderer.py` uses:

```text
modelV2
liveCalibration
selfdriveState
carParams
radarState
longitudinalPlan
```

`modelV2` draws path/lane/road-edge overlays. `radarState` draws lead vehicles. `longitudinalPlan.allowThrottle` colors the path.

### Required For Driver Monitoring Overlay

`openpilot/selfdrive/ui/onroad/driver_state.py` uses:

```text
driverMonitoringState
driverStateV2
selfdriveState
```

### Nice-To-Have For Full Fidelity

These are useful for mici widgets or deeper standard UI fidelity, but should come after the first bottom-screen pass:

```text
onroadEvents
liveParameters
carOutput
carControl
managerState
gpsLocationExternal
```

### Skip Initially

Do not bridge these initially:

```text
rawAudioData
testJoystick
```

`rawAudioData` is unrelated to the bottom onroad UI and could waste data-channel bandwidth. `testJoystick` is not part of UGV-to-GCS display feedback.

## Proposed Service Profiles

### Profile 1: Onroad Smoke

Purpose: make `gcs_debug_ui` transition onroad and show real speed/status, without model overlays.

```text
deviceState,pandaStates,selfdriveState,carState,controlsState,roadCameraState,wideRoadCameraState,liveCalibration
```

Expected result:

- Standard UI leaves home/offroad when the UGV is onroad.
- Camera appears if either `road` is requested, or the GCS standard UI is forced to start on `wideRoad`.
- HUD speed and alert state update.
- Model overlay may be absent or stale because `modelV2` is not included yet.

### Profile 2: Model Overlay

Add:

```text
modelV2,longitudinalPlan,radarState,carParams
```

Expected result:

- Path/lane/road-edge overlay appears.
- Lead indicator appears when radar has a lead and longitudinal control is active.
- Path color reflects throttle allowance.

### Profile 3: Driver Monitoring And Full Onroad

Add:

```text
driverMonitoringState,driverStateV2,onroadEvents,liveParameters,carOutput,carControl
```

Expected result:

- Driver monitoring icon/pose overlay works.
- Mici/torque/debug widgets have the services they expect.

## Data Channel Concerns

The current `webrtcd` outgoing proxy sends JSON for every updated service every 10 ms:

```python
self.sm.update(0)
for service, updated in self.sm.updated.items():
  if updated:
    channel.send(json.dumps(...).encode())
```

That is okay for `carState`, but full UI feedback will add:

- multiple 100 Hz services
- several 20 Hz services
- large `modelV2` messages
- JSON conversion overhead
- no sender-side buffered amount limit in `webrtcd` outgoing bridge

Before making full UI feedback the default, add one of these:

1. A named feedback profile with an explicit allowlist.
2. Per-service rate caps, for example:
   - `carState`, `selfdriveState`, `controlsState`: 20 Hz for UI
   - `modelV2`, `radarState`, `longitudinalPlan`, driver state: native 20 Hz
   - `cameraState`: 10 or 20 Hz
   - `deviceState`, `managerState`, `onroadEvents`: native low rate
3. Data-channel buffered amount checks on UGV outgoing feedback, like the GCS control sender already has.

Longer term, consider binary cereal payload forwarding instead of JSON for large services like `modelV2`. The current JSON approach is convenient and already working, so use it for the first smoke tests.

## Step-By-Step Plan

### Step 0: Develop Locally First

We can implement most of this on the GCS without the UGV powered on:

- GCS-only standard UI camera override.
- Feedback service profile parsing.
- WebRTC offer/request plumbing that expands a profile into `bridge_services_out`.
- Unit tests for the profile expansion and request body.

The UGV is only required for live validation:

- Confirm `webrtcd` accepts the expanded `bridge_services_out` list.
- Confirm data-channel load is acceptable.
- Confirm real `wideRoad` VisionIPC and real UI msgq services render correctly.

Create a branch:

```bash
git checkout -b gcs-standard-ui
```

Initial local implementation target:

```text
TURBO_GCS_WEBRTC_CAMS=wideRoad,driver
TURBO_GCS_STANDARD_UI_CAMERA=wideRoad
TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_smoke
```

`ui_smoke` should expand to:

```text
deviceState,pandaStates,selfdriveState,carState,controlsState,roadCameraState,wideRoadCameraState,liveCalibration
```

Keep `TURBO_GCS_WEBRTC_FEEDBACK_SERVICES` available as an explicit override/addition for ad hoc tests.

### Step 1: Add Wide-Only Standard UI Override

Implement the GCS-only UI camera override:

- Add an env var such as `TURBO_GCS_STANDARD_UI_CAMERA=wideRoad`.
- When `GCS=True`, construct the standard onroad `AugmentedRoadView` with `VisionStreamType.VISION_STREAM_WIDE_ROAD`.
- Lock the standard UI to that stream so `_switch_stream_if_needed()` does not switch back to missing `ROAD_CAM`.
- Keep normal in-car behavior unchanged when `GCS=False`.

Test locally without the UGV by instantiating the layout/helper and verifying:

- default is `ROAD_CAM`
- `GCS=True` plus `TURBO_GCS_STANDARD_UI_CAMERA=wideRoad` selects `WIDE_CAM`
- locked mode skips stream switching

### Step 2: Add Feedback Profiles

Add named feedback profiles for WebRTC GCS clients:

```text
torque: carState
ui_smoke: deviceState,pandaStates,selfdriveState,carState,controlsState,roadCameraState,wideRoadCameraState,liveCalibration
ui_model: ui_smoke + modelV2,longitudinalPlan,radarState,carParams
ui_full: ui_model + driverMonitoringState,driverStateV2,onroadEvents,liveParameters,carOutput,carControl
```

Both `webrtc_vipc.py` and `webrtc_signald.py` should accept:

```text
--feedback-profile
```

with env:

```text
TURBO_GCS_WEBRTC_FEEDBACK_PROFILE
```

The existing `--feedback-services` should still work. If both are set, combine and de-duplicate while preserving order.

Test locally:

- `ui_smoke` expands to the expected list.
- `ui_model` includes smoke services plus model services.
- explicit `--feedback-services` combines cleanly with a profile.
- WebRTC offer body includes the expanded list as `bridge_services_out`.

### Step 3: Run Standard UI Locally Without Manager

Start from the smallest manual test:

```bash
TURBO_GCS_WEBRTC_CAMS=wideRoad,driver \
TURBO_GCS_STANDARD_UI_CAMERA=wideRoad \
TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_smoke \
uv run python -m openpilot.tools.turbo.webrtc_signald
```

Then, in another terminal:

```bash
uv run python -m openpilot.selfdrive.ui.ui
```

Expected: camera view should connect to local `camerad` `wideRoad`; UI should go onroad when remote `deviceState.started` and `pandaStates` ignition are true.

### Step 4: Add Model Overlay Services

Use:

```text
TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_model
```

Expected: model overlay and leads appear. Watch WebRTC stats and data-channel behavior.

### Step 5: Add Driver Monitoring Services

Use:

```text
TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_full
```

Expected: driver monitoring overlay appears when `driverStateV2` is fresh and alerts are not covering it.

### Step 6: Add Manager Integration

Once manual launch works, update GCS config:

- Remove `gcs_debug_ui` from `BLOCK`.
- Keep `TURBO_GCS_WEBRTC_CAMS=wideRoad,driver`.
- Set `TURBO_GCS_STANDARD_UI_CAMERA=wideRoad`.
- Set `TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_smoke`, then graduate to `ui_model` or `ui_full`.

### Step 7: Add Transport Guardrails

Before making the full profile default:

- Add UGV outgoing bridge buffered amount checks.
- Add per-service counters and periodic logs for sent/skipped feedback.
- Add per-service rate caps if the raw profile creates data-channel pressure.
- Keep `carState` feedback as the minimal default for torque.
- Make full UI feedback opt-in with profile/env until bandwidth is proven stable.

### Step 6: Persist The Screen Layout

The top and bottom screen split is probably outside msgq transport:

- Top: Turbo UI (`gcs_ui`)
- Bottom: standard UI (`gcs_debug_ui`)

Decide whether this is handled by:

- separate `DISPLAY` values per process,
- compositor/window placement,
- explicit geometry env vars,
- or a small GCS launcher that starts each UI on the intended monitor.

Do this after the bottom UI works manually.

## Initial Recommendation

Start with the bandwidth-first camera set:

```text
TURBO_GCS_WEBRTC_CAMS=wideRoad,driver
```

Then add the standard UI GCS override to force `wideRoad` as its onroad camera.

For msgq feedback, start with:

```text
TURBO_GCS_WEBRTC_FEEDBACK_SERVICES=deviceState,pandaStates,selfdriveState,carState,controlsState,roadCameraState,wideRoadCameraState,liveCalibration
```

Then add:

```text
modelV2,longitudinalPlan,radarState,carParams
```

only after the bottom UI enters onroad and shows camera/HUD correctly.

This keeps the first test small and avoids adding narrow camera bandwidth. If the standard UI remains blank after this, the likely issue is that it is still trying to connect to `ROAD_CAM`; fix that with the GCS wide-only UI override before adding more msgq services.

## GCS Dual-Screen Launch Plan

### Current Findings

`./launch_openpilot.sh` just execs `./launch_chffrplus.sh`, which sources `.env` and starts manager.

The process config already defines both GCS UI processes:

- `gcs_ui`: `openpilot.tools.turbo.gcs_ui`
- `gcs_debug_ui`: `openpilot.selfdrive.ui.ui`

The immediate reason the standard debug UI did not appear is local config:

```text
BLOCK=manage_athenad,gcs_debug_ui
```

Manager reads `BLOCK` and excludes those process names from startup. Removing `gcs_debug_ui` from `BLOCK` should allow the process to start.

The current GCS X layout is one `DISPLAY=:1` screen with two stacked 1080p monitors:

```text
HDMI-1-0  1920x1080+0+0     external/top
eDP-1     1920x1080+0+1080  laptop/bottom
```

So the two UI windows should be assigned by monitor name on one X display:

- Turbo teleop UI: `OPENPILOT_UI_MONITOR=HDMI-1-0`
- Standard debug UI: `OPENPILOT_UI_MONITOR=eDP-1`

The standard UI also needs `BIG=1` if we want the full openpilot UI instead of the small PC/MICI layout.

### Implementation Chunks

1. Unblock the debug UI locally.
   - Change `.env` from `BLOCK=manage_athenad,gcs_debug_ui` to `BLOCK=manage_athenad`.
   - Keep this as a local config change unless we decide to commit a sample config.

2. Add monitor-based raylib window placement.
   - In `openpilot/system/ui/lib/application.py`, read optional env vars:
     - `OPENPILOT_UI_MONITOR`
     - `OPENPILOT_UI_DECORATED`
   - Use raylib monitor APIs to find monitor name, position, width, and height.
   - Create a borderless window sized to the selected monitor and position it at the monitor origin.
   - Fall back to existing UI sizing if `OPENPILOT_UI_MONITOR` is unset or invalid.

3. Avoid touching generic manager process infrastructure.
   - Do not modify `openpilot/system/manager/process.py`.
   - Add tiny GCS-specific wrapper modules that set env in `main()` before importing the real UI module.
   - Point `gcs_ui` and `gcs_debug_ui` at those wrappers from `process_config.py`.
   - Keep the standard `ui` process from preimporting `openpilot.selfdrive.ui.ui` on Turbo GCS configs so `BIG=1` can take effect in the debug wrapper.

4. Wire the two GCS UI processes.
   - `gcs_ui` wrapper defaults:
     - `OPENPILOT_UI_MONITOR=HDMI-1-0`
     - `OPENPILOT_UI_DECORATED=0`
   - `gcs_debug_ui` env:
     - `BIG=1`
     - `OPENPILOT_UI_MONITOR=eDP-1`
     - `OPENPILOT_UI_DECORATED=0`
     - `TURBO_GCS_STANDARD_UI_CAMERA=wideRoad`

5. Keep feedback/video settings explicit for the first full-stack test.
   - `TURBO_GCS_WEBRTC_CAMS=wideRoad,driver`
   - `TURBO_GCS_WEBRTC_FEEDBACK_PROFILE=ui_model`
   - `TURBO_GCS_WEBRTC_CONTROL_SERVICES=g29`

6. Test in small steps.
   - First: remove `gcs_debug_ui` from `BLOCK`, launch, and confirm both process names appear in manager output.
   - Second: run both UI modules manually with geometry env and verify top/bottom placement.
   - Third: enable manager process env overrides and launch through `./launch_openpilot.sh`.
   - Fourth: confirm WebRTC video publishes `wideRoad`/`driver`, standard UI stays on wide road camera, and model overlay services are valid/alive.

### Expected End State

Running `./launch_openpilot.sh` on the GCS should start:

- External monitor/top: fullscreen Turbo teleop UI.
- Laptop/bottom: fullscreen standard openpilot UI, locked to `wideRoad` for the onroad camera.
- One GCS WebRTC session receiving `wideRoad,driver` video and `ui_model` feedback services over the data channel.
