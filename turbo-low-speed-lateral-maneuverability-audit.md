# Turbo Low-Speed Lateral Maneuverability Audit

Date: 2026-08-21

Update: speed feedback and steering sign have since been fixed. The remaining first test is relaxing the Turbo-specific curvature cap.

## Scope

This audits why Turbo can look like the model wants an aggressive low-speed turn while the actual steering/control output is not very maneuverable. I inspected the current source tree and existing local Turbo notes. I did not find local `rlog`/`qlog` files in `/home/yeezy/vaporware`, `/home/yeezy/.comma`, `/home/yeezy/.openpilot`, or `tools/turbo/captures`, so this is a code and prior-notes audit, not a fresh route-statistics audit.

## Short Answer

The most likely primary limiter is not torque. Turbo is configured as an angle-control platform:

- `opendbc_repo/opendbc/car/turbo/interface.py` sets `ret.steerControlType = angle`.
- `opendbc_repo/opendbc/car/turbo/carcontroller.py` sends `STEER_CMD.STEER_ANGLE` from `actuators.steeringAngleDeg`.
- Panda Turbo safety currently allows the whitelisted TX messages without enforcing torque or angle limits.

The highest-confidence issue is the generic openpilot curvature cap:

- `openpilot/selfdrive/controls/lib/drive_helpers.py` caps desired curvature to `MAX_CURVATURE = 0.2 1/m`.
- With Turbo constants `wheelbase=0.3302 m`, `steerRatio=18`, and `STEER_ANGLE_MAX=180 deg`, the declared openpilot steering-command range can represent about `0.53 1/m` curvature, or about a `1.9 m` turn radius.
- This `180 deg` value is not a claim that the physical steering servo has 180 degrees of useful steering travel. It is the software's artificial `steeringAngleDeg` command scale, sent as `STEER_CMD.STEER_ANGLE = steeringAngleDeg * 100`.
- The generic `0.2 1/m` cap is a `5.0 m` turn radius and converts to only about `68 deg` of steering command on Turbo.
- That means self-drive may use only about 38 percent of the declared steering range at low speed, even if the model/UI path looks tighter.

So the current stack can definitely be limiting low-speed maneuverability before any torque limit would be visible.

## Control Path

The path is:

1. `modeld` publishes `modelV2.action.desiredCurvature`.
   - If the model has an `action` output, `modeld` computes desired curvature as `model_output['action'][0,0] / max(1.0, v_ego)^2`.
   - Below `1.0 m/s`, desired curvature does not grow further as speed falls.
2. `controlsd` chooses that curvature, unless `lateralManeuverPlan` is valid.
3. `controlsd` clips it through `clip_curvature()`.
   - This applies lateral jerk/accel limits and the hard `MAX_CURVATURE = 0.2 1/m` cap.
4. `LatControlAngle` converts clipped curvature to `actuators.steeringAngleDeg` through `VehicleModel.get_steer_from_curvature()`.
5. Turbo `CarController` applies `CarControllerParams.ANGLE_LIMITS`.
   - Max angle: `180 deg`.
   - Rate-up breakpoints: `[0, 5, 25] m/s -> [40, 24, 3.2] deg per 10 ms control frame`.
   - Rate-down breakpoints: `[0, 5, 25] m/s -> [80, 32, 4.8] deg per 10 ms control frame`.
   - At low speed, these rate limits are very loose, so they are probably not the low-speed limiter.
6. Turbo sends CAN `STEER_CMD` every `STEER_STEP = 2` frames, so 50 Hz.
7. `carOutput.actuatorsOutput.steeringAngleDeg` records the Python-limited command, not necessarily what the physical steering achieved.
8. `carState.steeringAngleDeg` reads `STEER_16 / 100.0`, which should be treated as the actuator feedback signal.

## Findings

### 1. Generic curvature cap is too low for an RC-scale platform

Evidence:

- `openpilot/selfdrive/controls/lib/drive_helpers.py`:
  - `MAX_CURVATURE = 0.2`
  - `clip_curvature()` clamps all desired curvature to `[-0.2, 0.2]`.
- Turbo software geometry:
  - `wheelbase = 0.3302`
  - `steerRatio = 18`
  - max openpilot steering command `180 deg`

Numerics:

- Commanded angle for `0.2 1/m`: `degrees(0.2 * 18 * 0.3302) = 68.1 deg`.
- Declared max openpilot command: `180 deg`.
- Kinematic curvature at `180 deg`: about `0.53 1/m`.
- Radius at `0.2 1/m`: `5.0 m`.
- Radius at `0.53 1/m`: about `1.9 m`.

Impact:

This directly matches the symptom: the visual/model path can look like a tight low-speed turn, but `controlsd` will not command the curvature needed for a small RC turning radius.

Important caveat:

The real physical limit depends on the full command chain:

- openpilot virtual steering command, in `steeringAngleDeg`
- gaming-wheel/full-scale command range, currently treated as `+/-180 deg`
- servo PWM or ECU command range
- steering rack/linkage travel
- actual front-wheel road angle

So `STEER_16 = 18000` should not be read as "servo physically rotated 180 degrees." If the intended calibration is `+/-180 deg` virtual steering command to full servo/rack travel, and that full rack travel produces about `+/-10 deg` road-wheel angle, then `steerRatio = 18` is internally consistent.

Under that interpretation, the curvature cap finding still matters: `MAX_CURVATURE = 0.2 1/m` maps to about `68 deg` virtual steering command, which is only about 38 percent of the available `+/-180 deg` command/rack range.

Recommended fix:

Make max curvature platform-dependent. For Turbo, start with a conservative cap around `0.45-0.50 1/m`, then validate on logs and real steering geometry. Do not globally raise `MAX_CURVATURE` for passenger cars.

### 2. Model action is also low-speed normalized at `max(1.0, v_ego)^2`

Evidence:

- `openpilot/selfdrive/modeld/modeld.py`:
  - `desired_curvature = model_output['action'][0,0] / (max(1.0, v_ego))**2`
  - If `v_ego <= 0.3`, it holds previous lateral curvature.

Impact:

Below `1 m/s`, the model action path stops asking for more curvature as speed falls. That is probably reasonable for passenger cars, but it can be under-aggressive for an RC platform where low-speed tight maneuvers are expected.

How to prove:

On a low-speed run, compare:

- `modelV2.position.y` / visual path shape
- `modelV2.action.desiredCurvature`
- `controlsState.desiredCurvature`

If `modelV2.action.desiredCurvature` is already small before `controlsd`, the model/action transform is limiting. If model action is large but `controlsState.desiredCurvature` is capped at `0.2`, `clip_curvature()` is limiting.

### 3. Saturation/limit reporting is mostly blind at low speed

Evidence:

- `LatControlAngle.sat_check_min_speed = 5.0`.
- Generic `LatControl._check_saturation()` only accumulates saturation above that speed.
- Turbo is not in `LatControlAngle.use_steer_limited_by_safety` brands.
- `controlsd` computes `steer_limited_by_safety` by comparing requested `carControl` angle to `carOutput` angle, but `LatControlAngle` does not use that path for Turbo saturation.

Impact:

At low speed, the stack can be visibly under-steering without surfacing `steerSaturated` or an obvious limit flag. Not seeing torque limits hit is expected, and not seeing steering saturation does not prove the stack is unrestricted.

Recommended fix:

For Turbo diagnostics, add explicit low-speed debug logging or a Turbo-specific saturation rule based on:

- `abs(carControl.actuators.steeringAngleDeg - carOutput.actuatorsOutput.steeringAngleDeg)`
- `abs(carOutput.actuatorsOutput.steeringAngleDeg - carState.steeringAngleDeg)`
- `abs(modelV2.action.desiredCurvature - controlsState.desiredCurvature)`

### 4. Prior route evidence showed steering command and feedback sign disagreement

Evidence from `turbo-lateral-selfdrive-bugs.md`, route `000000e9--b1b08f03a9`, segments `7-8`:

- With command and feedback magnitudes above `5 deg`, nearly all samples had opposite signs.
- Example:
  - `carControl.actuators.steeringAngleDeg = +7.49`
  - `carOutput.actuatorsOutput.steeringAngleDeg = +7.49`
  - decoded `STEER_CMD = +7.48 deg`
  - `carState.steeringAngleDeg = -7.48`

Current source:

- `carcontroller.py` sends `int(steering_angle_deg * 100.0)`.
- `carstate.py` reads feedback as `STEER_16 / 100.0`.
- There is no sign correction in either path.

Impact:

If this still reproduces, it is a critical interface bug. Even if low-speed magnitude limits are fixed, a sign mismatch makes closed-loop curvature and params learning wrong. It also makes the UI/model interpretation misleading.

Recommended fix:

Before tuning limits, run a static steering sign test and settle the convention:

- positive `STEER_CMD` should produce positive `STEER_16`
- positive `carState.steeringAngleDeg` should produce the curvature sign assumed by `controlsd`

Apply exactly one sign correction, either in the command path or feedback path, based on the physical convention.

### 5. Turbo is not marked `notCar`, so payload mass is wrong

Evidence:

- `CarInterfaceBase.get_params()` adds `STD_CARGO_KG = 136` unless `ret.notCar` is true.
- Turbo's `_get_params()` does not set `ret.notCar = True`.
- Turbo specs mass is `4.082 kg`, but effective CP mass becomes about `140 kg`.

Impact:

This is wrong for a robotics/RC platform. It affects derived inertia and tire stiffness used by paramsd and dynamic vehicle model behavior. It may not dominate the pure low-speed kinematic conversion, but it is still a bad foundation for learning and diagnostics.

Caveat:

Do not blindly set `ret.notCar = True` without adjusting manager predicates. `openpilot/system/manager/process_config.py` currently runs `controlsd` under `iscar`, while `notcar` starts `joystickd`. A clean fix needs a Turbo-specific process predicate or separate handling of no-payload mass without flipping all notCar behavior.

### 6. Speed feedback spikes can poison control and diagnostics

Evidence from prior local notes:

- Raw `SPEED_16` samples jumped from near zero to impossible values.
- `carState.vEgo` followed `SPEED_16 / 100.0` directly.
- Prior route saw max `carState.vEgo = 46.32 m/s`.
- `carstate.py` currently does no filtering or plausibility rejection.

Impact:

Bad speed affects:

- model action scaling by `max(1.0, v_ego)^2`
- `clip_curvature()` lateral accel/jerk limits
- `VehicleModel` steering conversion
- locationd validity
- speed-too-high events

At false high speeds, desired curvature and steering authority collapse. This can look like poor low-speed maneuverability if the vehicle is physically slow but `vEgo` briefly reports fast.

Recommended fix:

Add a Turbo speed deglitcher at the source if possible. If not, add a conservative temporary filter in `opendbc_repo/opendbc/car/turbo/carstate.py` that rejects physically impossible jumps from near-zero speed while preserving real acceleration.

### 7. Delay and parameter learning are not tuned for Turbo low-speed operation

Evidence:

- `paramsd` updates vehicle params only when `vEgo > 1.0 m/s` and `abs(steeringAngleDeg) < 45`.
- `lagd` requires `vEgo > 50 mph` by default, so it will never learn meaningful Turbo lateral delay.
- `liveDelay` therefore falls back to `CP.steerActuatorDelay + 0.2`, which is `0.3 s` for Turbo.
- `modeld` receives `lat_action_t = liveDelay.lateralDelay + frame_delay + action_delay`, roughly `0.375 s` when unlearned.

Impact:

Turbo low-speed steering is effectively using static delay and geometry assumptions. If the servo/controller latency differs materially from `0.3-0.375 s`, the model action can be aimed at the wrong horizon.

Recommended fix:

Add Turbo-specific lag learning thresholds or bypass lagd with an empirically measured constant. For low-speed testing, log actual command-to-feedback delay from `STEER_CMD` or `carOutput` to `STEER_16`.

## What To Log Next

For one controlled low-speed run, log these at minimum:

- `modelV2.action.desiredCurvature`
- `modelV2.position.x`, `modelV2.position.y`
- `controlsState.desiredCurvature`
- `controlsState.curvature`
- `carControl.latActive`
- `carControl.actuators.steeringAngleDeg`
- `carOutput.actuatorsOutput.steeringAngleDeg`
- decoded `STEER_CMD.STEER_ANGLE`
- `carState.steeringAngleDeg`
- `carState.vEgo`
- `liveParameters.steerRatio`
- `liveParameters.angleOffsetDeg`
- `liveDelay.lateralDelay`
- `livePose.angularVelocityDevice.z` or calibrated yaw rate

Decision table:

| Observation | Likely cause |
| --- | --- |
| `modelV2.action.desiredCurvature` is low while visual path is tight | model action transform / low-speed model behavior |
| `modelV2.action.desiredCurvature > 0.2`, but `controlsState.desiredCurvature == 0.2` | `clip_curvature()` cap |
| `carControl` angle is high, but `carOutput` is lower | Turbo `ANGLE_LIMITS` rate/max clamp |
| `carOutput` angle is high, but `carState` angle is lower or delayed | servo/ECU actuator limit or feedback lag |
| `carOutput` and `carState` have opposite signs | Turbo sign convention bug |
| `vEgo` spikes high while physically slow | speed feedback/filtering bug |

## Recommended Order Of Operations

1. Verify speed feedback and steering sign are still fixed on the test build.
2. Add a short offline log audit script for the signal comparisons above.
3. Make `MAX_CURVATURE` platform-dependent and raise Turbo's low-speed cap.
4. Add Turbo-specific low-speed saturation diagnostics so under-actuation is visible below `5 m/s`.
5. Fix speed deglitching before trusting any low-speed maneuverability result.
6. Revisit Turbo mass/notCar handling without accidentally disabling `controlsd`.
7. Measure actual steering command-to-feedback delay and either tune `steerActuatorDelay` or make `lagd` Turbo-aware.

## Current Best Diagnosis

The current behavior is most likely a controls-stack/interface limitation, not a pure model problem:

- The model may ask for a tight path, but control curvature is globally capped to a passenger-car-like `0.2 1/m`.
- Turbo angle control then sends at most the clipped angle output; torque limits are not involved.
- Low-speed saturation is not reliably reported.
- Prior evidence suggests a possible steering sign mismatch, which must be resolved before tuning.
- Speed spikes and stale/incorrect learned parameters can further distort the result.

The first code change I would test is a Turbo-specific curvature cap, but only after a quick steering sign sanity check and confirmation that full-scale `+/-180 deg` virtual command really maps to the intended full servo/rack travel.
