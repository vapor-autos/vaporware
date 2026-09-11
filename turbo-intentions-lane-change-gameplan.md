# Turbo intentions and lane-change gameplan

Date: 2026-09-11. Research baseline plus implementation notes. Sections 1–13 record the original review/proposal; section 14 records the implementation and supersedes proposed details where noted. No UGV deployment, firmware change, parameter change, or hardware test was performed during implementation.

Reviewed Vaporware `steer-assist-phase-2` at `eb63babee584359861f5333ed70c8717c8401700`, checked-out opendbc at `dd04a72e3957627e9202ecc85d2a687f8e1ba090`, and installed `g29py==0.0.17`. Local g29py source is in `/home/yeezy/g29-py`, at `0b8568c63b7165ee9f00b393a93a531dae52034a`. Existing unrelated worktree changes were left alone.

## Recommendation

Use **paddles for explicit lane-change intent, and keep wheel movement for steering override**. The UGV should validate the request and feed the existing model desire mechanism locally. Do not make the GCS run the maneuver state machine or send model tensors.

- **First-test decision: one left/right paddle press requests one lane change in that direction.** No operator arming cycle, second pull, long hold, or steering nudge. The press itself authorizes the request; UGV validation and acknowledgment still apply.
- Intent requests use the existing reliable WebRTC/SCTP channel, with request identity, acknowledgment, and bounded lifetime. Wheel/pedal state and steering override stay on their current latest-state UDP path.
- Keep the current haptic target tracking and 200 ms release/200 ms fade settings initially. Never disable override detection just because a lane change is active.
- Add an independent directional/status indicator. Keep orange reserved for an actually applied steering override, including its release fade.
- Limit version one to `laneChangeLeft` and `laneChangeRight`. Intersection turns, route selection, and arbitrary “intentions” are separate work.

There are three immediate prerequisites: repair G29 combined-button decoding, separate Turbo blinkers from headlights, and introduce a Turbo-only speed gate suitable for a validated closed-course experiment. None requires changing ECU speed firmware or inventing a new CAN message.

This single-press decision supersedes the earlier two-pull proposal. It simplifies the interaction state machine and isolates the first test from nudge-detection tuning. It does not remove freshness, one-shot, health, or override checks.

## 1. What “intentions” means in this model

The relevant API is called **desire**. It is a categorical input conditioning the model's behavior, not a steering angle, a natural-language instruction, or a guarantee that a maneuver will succeed.

The local [Desire enum](openpilot/cereal/log.capnp) contains:

| Value | Meaning | Current production helper emits it? |
| --- | --- | --- |
| 0 | none | Yes; the model's desire input explicitly clears this slot |
| 1 / 2 | turnLeft / turnRight | No |
| 3 / 4 | laneChangeLeft / laneChangeRight | Yes |
| 5 / 6 | keepLeft / keepRight | No |

[ModelConstants](openpilot/selfdrive/modeld/constants.py) uses an eight-element desire vector, despite only seven named schema values. The extra slot is not a documented extra user command.

In [modeld.py](openpilot/selfdrive/modeld/modeld.py):

1. `DesireHelper` derives a desire from vehicle state.
2. The loop constructs a one-hot vector from `DH.desire`.
3. `ModelState.run()` converts that level into a **rising-edge pulse**, using `prev_desire`. Holding the desire does not continuously retrigger it.
4. The model produces its path/action and `meta.desireState` probabilities. Those output probabilities are not the operator's requested direction and are not a blind-spot clearance score.
5. After inference, the helper consumes the left/right lane-change probabilities and updates the state published in `modelV2.meta`. The next evaluation uses the resulting desire.

The relevant upstream implementation also documents the pulse behavior: [comma modeld](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/modeld/modeld.py). Local code, not a moving upstream branch, is the implementation baseline for this plan.

**Important consequence:** returning the input to `none` is not a demonstrated “abort and return to the original lane” command. The model has temporal state and has already received the pulse. Likewise, the existence of `turnLeft` in an enum does not establish that this model version supports reliable operator-requested intersection turns. Do not expose those values without a separate model/simulation investigation.

### Pulse-delivery edge case to fix before adding requests

The local `ModelState.run()` updates `prev_desire` **before** its `prepare_only` early return. A first desire pulse on a dropped-frame/prepare-only iteration can therefore be consumed without a policy evaluation seeing it. This is a code-inspection finding, not a reproduced live failure.

An implementation must retain the pending pulse across skipped evaluations and associate “started” acknowledgment with an actual evaluation that consumed it. Add a regression test for this ordering; do not acknowledge success merely because the network message arrived or the helper entered `laneChangeStarting`.

## 2. Stock lane-change behavior

The local [desire_helper.py](openpilot/selfdrive/controls/lib/desire_helper.py) is shared across brands. Its current behavior is:

```text
off
  -- lateral active + one new blinker + speed >= 20 mph --> preLaneChange
preLaneChange
  -- driver steering torque in selected direction + no indicated blind spot --> laneChangeStarting
laneChangeStarting
  -- model lane-change probability < 0.02 after at least 0.5 s --> off or preLaneChange
```

- The speed minimum is **20 mph = 8.9408 m/s**, not `CP.minSteerSpeed`.
- Left confirmation requires `steeringPressed` and positive `steeringTorque`; right requires negative torque.
- Both blinkers count as neither direction, using XOR.
- Removing the blinker or dropping below the speed minimum cancels **preLaneChange**. The starting branch does not use those conditions as an immediate abort.
- A starting maneuver times out after the helper's 10-second limit. These timers advance with model updates; they are not an independent wall-clock watchdog.
- If one blinker is still on when the helper finishes, it returns to preLaneChange. A remote one-shot request must not silently become a repeated-lane-change latch.
- `laneChangeFinishing` remains in the schema/UI, but this checked-out helper does not enter it. Do not design around an older four-state lane-line-fading implementation.
- In this checkout, a blinker first enabled below the speed threshold does not newly arm just because the speed later crosses it. A fresh edge is needed.

The current [upstream helper](https://github.com/commaai/openpilot/blob/master/openpilot/selfdrive/controls/lib/desire_helper.py) uses the same basic gates. There is a minor moving-master difference in how the previous-blinker state accounts for lateral engagement; preserve and test the checked-out behavior for non-Turbo cars rather than importing unrelated upstream changes.

## 3. Comparison across all checked-out car interfaces

Scope: all 16 brand/non-car `carstate.py` files present in this opendbc checkout, their interface configuration, and blinker-output references in the controllers. This is not a claim about every historical fork or every possible OEM configuration.

The common pattern is that each interface normalizes OEM CAN signals into `CarState`; **the model-layer helper owns lane-change intent**, not a separate lane-change algorithm in each car interface. Steering torque thresholds vary by platform and should not be copied into G29 angle units.

| Interface | Blinker input | Driver confirmation input | Blind-spot input in this code |
| --- | --- | --- | --- |
| [Chrysler](opendbc_repo/opendbc/car/chrysler/carstate.py) | Stalk smoothing or direct lever state, depending on platform | Column/driver torque threshold | Conditional BSM |
| [Ford](opendbc_repo/opendbc/car/ford/carstate.py) | Turn-light switch | Filtered column-torque threshold | Conditional side-detect CAN |
| [GM](opendbc_repo/opendbc/car/gm/carstate.py) | BCM turn-signal state | Driver-applied torque threshold | Conditional BCM BSM |
| [Honda](opendbc_repo/opendbc/car/honda/carstate.py) | Stalk smoothing | Platform-specific torque threshold | Conditional BSM |
| [Hyundai](opendbc_repo/opendbc/car/hyundai/carstate.py) | Lamp smoothing; classic CAN and CAN-FD variants | Filtered column-torque threshold | Conditional classic/CAN-FD BSM |
| [Mazda](opendbc_repo/opendbc/car/mazda/carstate.py) | Lamp smoothing | Steering-torque threshold | BSM |
| [Nissan](opendbc_repo/opendbc/car/nissan/carstate.py) | Light states | Averaged driver torque, then threshold | Not populated here |
| [PSA](opendbc_repo/opendbc/car/psa/carstate.py) | BSI turn-signal state | Filtered driver-torque threshold | Not populated here |
| [Rivian](opendbc_repo/opendbc/car/rivian/carstate.py) | Indicator states | Filtered torsion-bar torque threshold | BSM assignments commented out |
| [Subaru](opendbc_repo/opendbc/car/subaru/carstate.py) | Lamp smoothing | Platform-specific torque threshold | Conditional BSM |
| [Tesla](opendbc_repo/opendbc/car/tesla/carstate.py) | UI blinker states | Sign-normalized, filtered torsion-bar torque | DAS rear blind-spot states |
| [Toyota](opendbc_repo/opendbc/car/toyota/carstate.py) | Turn-signal state | Driver-torque threshold | Conditional adjacent/approaching BSM |
| [Volkswagen](opendbc_repo/opendbc/car/volkswagen/carstate.py) | Direct states or stalk smoothing across PQ/MQB/MEB/MLB paths | Sign-normalized torque; some paths filter it | Conditional side-assist/BSM |
| [Body](opendbc_repo/opendbc/car/body/carstate.py) | Not populated | Not populated | Not populated |
| [Mock](opendbc_repo/opendbc/car/mock/carstate.py) | Default state | Default state | Default state |
| [Turbo](opendbc_repo/opendbc/car/turbo/carstate.py) | **Not populated** | **Neither torque nor steeringPressed populated** | **Not populated** |

[CarStateBase](opendbc_repo/opendbc/car/interfaces.py) supplies lamp/stalk smoothing and steering-pressed filtering. These prevent flashing lamps and noisy torque signals from directly becoming unstable model inputs.

Input blinkers and output blinkers are separate concepts. In the checked-out controller scan, Hyundai CAN-FD has an optional real blinker actuation path using SPAS messages. Turbo instead maps `CC.leftBlinker` to headlights on and `CC.rightBlinker` to headlights off. Most other controllers do not drive their turn signals from these fields.

### Why copying the car path literally is wrong for Turbo

Turbo has no physical driver-torque sensor and no stalk/BSM inputs. G29 wheel displacement or commanded force is not measured column torque. Publishing fake `steeringTorque`/`steeringPressed` merely to satisfy the helper would also affect stock override events and other consumers. Stock `steerOverride` is not the same mechanism as our `turboSteerAssistState` overlay; it does not simply mean “disengage.”

Keep `CarState` factual. Supply explicit, validated intent to the helper at a separate Turbo-only input boundary. No second `carState` publisher and no synthetic CAN blinker/torque messages.

## 4. G29 findings and button proposal

### Existing hardware/software support

The G29 has physical paddle shifters: [Logitech hardware description](https://www.logitechg.com/en-us/shop/p/driving-force-racing-wheel). Our installed [g29py 0.0.17](https://pypi.org/project/g29py/0.0.17/) reads the wheel through HID, not through evdev button numbers. It already exposes `left_paddle` and `right_paddle` in `state["buttons"]`, plus `button_down`/`button_up` events. Its documented Linux configuration is PS3 mode, USB `046d:c24f`.

No new input reader, G Hub remapping, or guessed `L1`/`R1` mapping is needed. Use the existing `g29d` reader and names. The current Cereal `G29` struct and `_publish_state()` do not transmit paddle fields; a semantic intent service avoids making raw paddle events the network API.

The project's [captured button map](https://github.com/vapor-autos/g29py/blob/master/docs/button-map.md) confirms byte 1: right paddle `0x01`, left paddle `0x02`, L2 `0x08`, and both paddles `0x03`.

### Prerequisite: combined-button decoder bug

Installed `g29py/g29.py:apply_misc()` uses exact-value `elif` comparisons. It consequently loses combined presses instead of decoding independent bits. A local, hardware-free test of the installed decoder produced:

| Byte 1 | Input | Decoded pressed buttons |
| --- | --- | --- |
| `0x01` | Right paddle | right_paddle |
| `0x02` | Left paddle | left_paddle |
| `0x03` | Both paddles | **none** |
| `0x08` | L2 | L2 |
| `0x0a` | Left paddle + L2 | **none** |
| `0x06` | Left paddle + R2 | **none** |

Both-paddle input is documented by existing hardware captures. The other combinations above were synthetic decoder checks; confirm their packets during a stationary hardware test too.

This matters beyond missing a request: a combined press can hide L2 cancel, and transitions into/out of an unrecognized combination can generate false button-up/down edges. That could accidentally create a new lane-change request if left unfixed.

Fix byte-1 bit decoding in `/home/yeezy/g29-py`, test all combinations of its known bits and transition sequences, release/pin a corrected dependency, then verify on the real wheel while stationary. Review the other packed button bytes separately; do not blindly apply a bitmask interpretation to the D-pad's hat value. No library changes have been made in this research turn.

Also fix `_publish_teleop_command()` arbitration: today it selects the first matching control in a mapping ordered headlights, enable, cancel. **L2 cancel must win simultaneous inputs**, including L3 + L2 and paddle combinations.

### Selected first-test mapping

| Input | Proposed behavior |
| --- | --- |
| Left/right paddle from idle | Request exactly one lane change in that direction; execute only if UGV gates pass |
| Hold a paddle | No repeat; release is required before a later fresh request |
| Any direction paddle while a request is pending or executing | Report busy; do not queue, replace, or reverse the maneuver |
| Both paddles detected together from idle | Reject ambiguous direction; require both released before another request |
| Both paddles after a request was sent | No special cancel gesture in version one; use L2. Do not claim a previously sent request was retracted |
| L2 | Existing global disengage/cancel, plus clear pending intent; highest priority |
| L3 | Existing engage; must never create or resurrect an old intent |
| R2 | Preserve the operator-contact logging marker |
| D-pad up/down | Preserve headlights |
| Dial | Preserve GCS overlay sizing |
| Steering wheel | Existing override/release behavior |

There is no armed selection or three-second arm timeout. A new action instead has the bounded context/deadline described below. No pending request survives disengagement, disconnect, process restart, or feature disable. A rejected request requires release and a new press; it cannot start later because conditions improve. Reconnect/engage while a paddle is already held must not count as a new press.

Use debounced press edges from the existing reader, preserve event order, and suppress additional requests from an ambiguous/backlogged batch. Check current paddle levels as well as edges so simultaneous opposing inputs do not choose a direction by dictionary order. Do not infer that a later second paddle can retract a request already sent. Normal debounce is not an operator hold-to-confirm cycle.

The first powered test therefore exercises only: **press paddle -> UGV accepts/rejects -> one model pulse -> maneuver status**. Nudge confirmation remains possible future work, not part of this test.

## 5. Will haptic steering assist interfere?

**It can interact mechanically and through override, but there is no direct intent connection today.** The model can continue evaluating during an override; our applied steering override does not currently notify `DesireHelper`.

Current path:

```text
UGV model action -> curvature limiting -> angle controller -> underlying model angle
                                                        |                   |
                                                        |                   +-> controlsState -> GCS haptic target
                                                        +-> Turbo override selection/fade -> actuator limits -> ECU
```

The GCS follows `controlsState.lateralControlState.angleState.steeringAngleDesiredDeg`, not the overridden `carOutput` target. That preserves the ability to feel the underlying model while intervening. A lane-change desire changes the model trajectory, so its steering target and haptic target should change through this same path.

Relevant settings in [g29d.py](openpilot/tools/turbo/g29d.py) and [steer_assist.py](openpilot/tools/turbo/steer_assist.py):

- 50 Hz GCS loop; haptic target slew limit 180 command-degrees/s.
- Tracking acquisition: within 5 degrees for 300 ms.
- Override candidate: outward wheel movement above 10 degrees/s, beyond the 5-degree residual threshold, with 80 ms candidate persistence.
- Release: 200 ms qualifying wheel-to-haptic alignment, 7-degree release band, 10-degree abort threshold, 60 ms failure grace.
- UGV normal release fade: 200 ms. Invalid/stale safety fallbacks do not use that fade.
- UGV override input freshness: 250 ms; target-context freshness: 350 ms. Initial target mismatch limit: 15 degrees, relaxed for an already accepted active override session.

These are software command-angle units, not independently measured physical road-wheel angles.

### Interaction rules

1. **For this first test, the paddle press is the explicit request.** Do not also require wheel motion, haptic force, R2, or override activation. A future nudge mode would need its own deliberate-intervention arbitration; it is not fundamentally incompatible with haptics, but is outside this single-press test.
2. Accept/start only with fresh feedback, stable GCS tracking, and no UGV-applied override or release fade. Check both the GCS's current decision and the UGV's local applied state. An old inactive acknowledgment is insufficient. Reject a conflict instead of waiting to start after the operator releases.
3. Keep override available throughout a maneuver. A haptic-target jump alone should not be classified as deliberate wheel motion, but wheel inertia/overshoot can still satisfy the detector. Test this physically; the existing route does not prove passive-wheel false-positive immunity.
4. Do not mask override detection or add a force “kick” for lane-change acknowledgment. A kick could itself create the input used to detect override. Use the UI first.
5. A paddle pull can move the wheel slightly. Test the single press while lightly holding the wheel, as well as hands off during subsequent target changes.
6. During an active lane change, ordinary override remains temporary: the model may still intend to finish the maneuver when the wheel hands back. Do not silently present that as an aborted maneuver. To abandon it, the operator uses global disengage and manual control until ready to re-engage.
7. Preserve existing freshness/lineage rejection. A new lane-change target can make delayed steering feedback differ sharply; do not loosen mismatch or stale thresholds to hide that race.

The [last outdoor analysis](/home/yeezy/turbo-test-artifacts/20260911-outdoor-steer-assist/report.md) verified release behavior, but R2 was never held. It cannot establish exactly when hands were removed or distinguish every intentional intervention from passive wheel movement. Use marked trials for the new feature.

## 6. GCS–UGV networking

### Current transport, verified in this checkout

| Traffic | Current path |
| --- | --- |
| G29 wheel/pedal state and `turboSteerAssist` | Raw latest-state UDP when the uplink negotiates a UDP control endpoint |
| `turboTeleopCommand` discrete commands | Reliable ordered WebRTC `data` channel in the UDP-enabled setup |
| Feedback | Configured UDP feedback; also an unordered, zero-retransmission WebRTC feedback-channel option and a legacy fallback |
| Model inference and vehicle steering application | Local to UGV; neither waits on a GCS round trip |

Source: [webrtc_controls.py](openpilot/tools/turbo/webrtc_controls.py), [webrtc_signald.py](openpilot/tools/turbo/webrtc_signald.py), [webrtc_uplink.py](openpilot/tools/turbo/webrtc_uplink.py), [webrtcd.py](openpilot/system/webrtc/webrtcd.py), and [teleoprtc stream](teleoprtc_repo/teleoprtc/stream.py). Local configuration selects `ui_model,steer_assist` feedback and a UDP feedback endpoint. No remote connection was needed for this review.

“SCTP versus UDP” here means a reliable WebRTC data channel versus our raw datagram transport. WebRTC data channels themselves run SCTP over DTLS over ICE/UDP; they are not a separate physical LTE link. [RFC 8831](https://www.rfc-editor.org/rfc/rfc8831.html)

### Proposed split

```text
GCS: existing G29 reader / 50 Hz loop
  -> paddle interaction controller
  -> turboIntentRequest -- existing reliable SCTP data channel --> UGV modeld intent adapter
                                                                      |
                                                   local gates -> DesireHelper -> model pulse
                                                                      |
GCS UI <-------------- turboIntentState, lightweight latest feedback ---+

GCS wheel/pedals + steering override ----- existing UDP path -----> unchanged vehicle control paths
```

Use two custom Cereal services, proposed names `turboIntentRequest` and `turboIntentState`. Reuse reserved schema slots while preserving their type IDs and existing field ordinals. This is cleaner than extending `turboTeleopCommand` into unrelated lane-change semantics: `teleopd` currently translates every command on that service into CAN and raises on unsupported commands.

The request is a **versioned state snapshot with stable identity**, not a one-frame paddle bit:

| Request field | Purpose |
| --- | --- |
| Protocol version/capability | Old peers fail closed; no silent steering-angle fallback |
| Transport/operator session identity | Reject traffic from a previous WebRTC/operator session |
| UGV-issued intent epoch | Invalidates requests after modeld restart or disengage/re-engage |
| Request ID + revision | Identify one maneuver and any cancellation revision; idempotent duplicates |
| Direction + action | `left/right`, `request/cancel`; no arm/confirm cycle or arbitrary numeric desire injection |
| Referenced UGV feedback timestamp | Bound command age on the UGV's own monotonic clock |
| Current operator-control context | Reject while override/invalid input is present; retain correlation in logs |

The echoed feedback timestamp is frozen for each action/revision. Repeated transmission must not refresh an old paddle request into a new command. Do not subtract GCS monotonic time from UGV monotonic time. The existing steering-assist target-context approach is the useful precedent. A cancel action supports clearing a pending request on L2 or invalidation; it does not introduce another user-facing gesture or guarantee a post-pulse abort.

The UGV feedback contains its current epoch, accepted request/revision, direction, phase, rejection reason, actual pulse-consumption frame/time, maneuver progress, and operator-control conflict. Publish at **10 Hz initially**, with last terminal result retained until superseded. The vehicle state, not the GCS optimistic display, is authoritative.

### Delivery/lifetime rules

1. Bind intent requests to the current WebRTC session in the bridge and invalidate that binding on teardown. Wire this explicitly; neither a user-supplied ID nor the outer Cereal timestamp alone provides session validation. The UGV epoch must change on modeld restart and engagement reset.
2. Consume intent snapshots through existing bridge/daemon subscriptions added at startup. Do not launch another subscriber during a drive. Validate protocol, fields, session, epoch, monotonic revision, and legal phase transition before changing intent state.
3. Only one request may be pending/executing. A single press is sufficient authorization; do not wait for another human confirmation. UGV acknowledgment reports acceptance/execution, not a request to press again. No maneuver queue.
4. Proposed new-action context limit: **350 ms**, plus locally fresh control/vehicle state. This is an initial engineering bound, not measured new-feature latency. Reject a delayed request; require a new physical action. Recheck expiry immediately before pulse consumption, not just on receipt.
5. Republish an unacknowledged action snapshot at a bounded rate, initially 10 Hz, until acknowledgment or its original deadline. A retry keeps the same request/revision and frozen context. Deduplicate on the UGV; never emit a second model pulse for a duplicate.
6. A rejected/expired revision stays terminal. It must not later become acceptable simply because speed rises, an override releases, or a blind spot clears.
7. Preserve snapshots through local message conflation. `CerealDataChannelSender` uses `SubMaster` latest-message semantics and can skip updates under backpressure: reliable SCTP alone does not guarantee an ephemeral local event reaches the channel.
8. When blocked, keep only the newest relevant intent snapshot; do not build an unbounded local retry queue. Once handed to reliable SCTP, delayed bytes cannot be assumed retractable, so receiver-side expiry remains mandatory.
9. Feedback loss means unknown status, not success. After an uncertain request, query/wait for the same request's state; never create a second request as an automatic retry.
10. Pending intent expires on disconnect, stale operator presence, wheel disconnect, or local feature disable. Bridge teardown and the bounded request lifetime both need coverage; a stopped G29 must not leave an indefinitely pending request.
11. Keep intent off raw UDP in version one. A robust datagram request protocol is possible with repetition/acknowledgment, but one-frame unreliable paddle events are not sufficient. Do not duplicate the same action on both transports.

Do not equate protocol IDs with security. The existing raw UDP receiver is not DTLS-protected at the application layer. Keep its established trusted network boundary; this feature must not expose a new unauthenticated public control port. The intent channel uses the existing authenticated/encrypted WebRTC transport association, subject to the application's signaling/access controls.

## 7. UGV integration and model acknowledgment

Recommended implementation shape:

- A pure, unit-testable `TurboIntentManager` owned by `modeld`, created only for Turbo. It validates remote input and produces a small typed lane-change input.
- Add an optional explicit input to `DesireHelper`, or extract its input decoding into a typed helper. The normal-car default continues to derive blinkers, torque confirmation, and BSM from `CarState` exactly as before.
- Parameterize the helper's minimum speed with an unchanged default of 20 mph. Only Turbo supplies a separate configured value under the feature gate.
- Explicit intent carries direction and one-use authorization from the paddle press; it does not masquerade as sensed driver torque. Maintain existing lane-change probability tracking locally. If the shared helper internally passes through `preLaneChange`, satisfy its explicit input from the same validated request without a second human action; suppress a misleading confirmation prompt.
- Clear the virtual selection after the one-shot start is consumed so completion cannot return to an automatically confirmed pre-lane-change state.
- Keep network handling out of tensor/model execution. Drain/validate requests in the normal modeld loop, and latch the pulse until a real evaluation can consume it.
- Distinguish local `pending`, UGV `awaitingEvaluation`, and `executing`. There is no operator-facing `armed` phase. “Executing” requires the selected pulse to have reached inference. `modelV2.meta` by itself currently mixes post-evaluation helper state with that frame's output, so it is insufficient as a precise pulse-delivery acknowledgment.
- `completed` must mean the model/helper's maneuver-completion criterion, not independent proof of crossing into a valid adjacent lane. Record the probability and elapsed time; a model that never raises its lane-change probability can otherwise look “done” after 0.5 seconds. Treat lack of a meaningful model response as a diagnostic/non-success outcome, with the response threshold established in simulation.

No GCS inference and no new trajectory planner are needed for the initial lane-change request path. The model and all steering/actuator limits remain on the UGV.

## 8. Speed, scene, and health gates

The previous outdoor route peaked at reported **6.15 m/s**, below stock lane-change entry speed. Turbo also currently has neither blinker inputs nor torque confirmation. Adding paddle bits alone therefore cannot enable the stock behavior.

Recommended feature gate: disabled by default, with shadow-only mode for the first implementation stage. Shadow mode logs would-accept/would-reject decisions but never changes the model desire.

For subsequent closed-course experiments, make the Turbo minimum explicit. **2.0 m/s held continuously for 0.5 seconds** is a candidate to evaluate in simulation, not a validated setting to deploy automatically. Apply a separately configured upper test speed and require a real forward-motion indication. Do not change `vEgo`, the firmware scale, or the global 20 mph constant to bypass the gate, and do not accelerate the UGV to 20 mph just to exercise this feature.

Why a sustained-motion condition matters: the last route showed isolated 2/4 m/s CAN speed pulses lasting 30–121 ms. That route also had no valid GPS fix. Re-verify physical speed and investigate those pulses before powered lane-change tests; the 0.5-second condition is defense in depth, not a speed-sensor repair.

At request acceptance and immediately before pulse consumption require:

- Lateral control active and valid, fresh CAN/carState, calibration/model health adequate, and no existing disabling event.
- The configured forward speed window and sustained-motion condition. Turbo's current `gearShifter` is hard-coded drive; it is not proof the operator is not commanding reverse. Carry/check fresh reverse-pedal context through the existing controls path or inhibit at both ends with an explicit UGV input before enabling execution.
- Fresh operator/control feedback, no steering override or release fade, current session/epoch, and a valid unconsumed single-press request.
- No indicated blind spot where an actual sensor exists. Turbo has no BSM: false defaults mean **unavailable**, not “lane clear.”
- No conflicting lateral maneuver test mode. `controlsd` can select `lateralManeuverPlan` instead of the model action, so intent testing must not run with that alternate controller source active.

Use an empty, controlled course with a spotter and adjacent drivable space. A low camera on a small UGV is a different visual/vehicle domain from a road car. Lane markings/road edges can inform test eligibility, but their probabilities are not obstacle or adjacent-lane clearance checks. Parking-lot wandering or selecting a fork is not automatically equivalent to a trained lane change.

The operator still controls throttle. This feature must not imply automated longitudinal control or collision avoidance.

## 9. Cancellation, timeout, and link loss

Make the boundary explicit in both code and UI:

| Moment | Meaning of cancel |
| --- | --- |
| Request remains local and unsent | L2/invalidation clears it; no model pulse was requested over the network |
| Request sent but pulse not consumed | L2/invalidation cancels pending intent if processed before consumption; otherwise report too late, not canceled |
| Pulse already consumed | No verified model “undo” exists; use wheel override for temporary intervention or L2 for global disengage/manual control |
| Model/helper finishes | Clear the one-shot request; another maneuver requires a fresh paddle press after release |

A cancel packet can race with a request. Even reliable ordering cannot undo a pulse already consumed. Show the UGV's result and preserve steering/global cancel priority. Releasing the paddle does not cancel the one-shot request or maneuver; it only permits a later new press once the system is idle.

After a steering override during execution, retain an honest `executing / operator overriding` state until the maneuver ends or engagement is canceled. The current release behavior hands back to the current model, which may still be executing the lane change. Do not reset a UI state to idle and then unexpectedly resume a hidden maneuver.

Use an independent monotonic execution watchdog, initially matching the stock 10-second budget. On timeout, mark failure/takeover required and block new requests; clearing the desire/helper does not prove the vehicle has stopped changing lanes. Do not automatically send an opposite desire, reset model hidden state, or impose a steering snap-back.

On link loss **before start**, expire pending intent. **After start**, stop accepting new intents and report operator-link loss through the existing control-safety path; the local model may continue its current maneuver. Do not pretend network loss automatically cancels it. Auditing the existing teleop/ECU link-loss stop/disengage behavior is a mandatory gate before powered testing. This review has not certified that broader failsafe. A new automatic abort trajectory or stop policy would require a separate control design and validation, not just a paddle mapping.

## 10. UI and logging

Keep the green/amber engagement frame semantics unchanged. Lane change is still model steering, so it does not itself turn the frame orange. Applied override and its fade retain orange priority.

Add a small left/right indicator with distinguishable states: local request pending, awaiting evaluation, executing, rejected/expired, and unknown/stale. There is no armed/confirm screen. Short instructional text can be transient; no new permanent “manual mode” label. Never show an optimistic request as already executing.

The existing Mici `TurnIntent` widget reads lane-change events, and the default pre-lane-change alert says to steer left/right. If standard `preLaneChange` metadata occurs briefly inside the helper, suppress the Turbo-only human-confirmation prompt; the paddle request already authorized it. Leave road-car prompts untouched. A generic “Car in Blindspot” message would also be misleading for Turbo's unavailable BSM.

The current `modelV2` network projection strips `laneChangeState`, `laneChangeDirection`, and `desireState`, and sends model UI feedback at only 2 Hz. Therefore do not base a responsive new indicator or request acknowledgment on projected `modelV2`. Use the dedicated small `turboIntentState`, explicitly included in the relevant feedback profiles/rate configuration. Optionally retain the two lane-change metadata fields in the projection for debug rendering, without raising the full model stream rate.

Add fields to the existing producer-written 50 Hz GCS trace:

- Raw paddle levels and ordered edges, request/cancel action, request/revision/session identity.
- Referenced UGV timestamp, local send/ack times, pending age, and rejection reason.
- GCS tracking/override status and existing wheel/model/haptic target measurements.
- Latest UGV intent phase, pulse frame/time, and applied override/fade status.

Log UGV request/state services and the actual pulse-consumption event. Verify generated service definitions/logger builds so new services really appear in rlogs; do not assume adding a Python registry entry rebuilds an already installed logger. Terminal state repetition makes a lost feedback packet recoverable.

Observe through `/tmp/turbo-metrics/latest/`, producer trace files, and post-route logs. No extra live Cereal/msgq monitors or coding-agent PTY attached to the driving GCS manager.

## 11. Implementation sequence and files

### A. Prerequisites, kept separate from feature activation

1. Fix combined button decoding and tests in `/home/yeezy/g29-py`; publish/pin the tested version via `pyproject.toml` and its lockfile. Correct GCS cancel priority and test multiple simultaneous edges.
2. Remove the lane-change-to-headlight mapping in [Turbo carcontroller](opendbc_repo/opendbc/car/turbo/carcontroller.py), preserving explicit teleop headlight commands. Update the submodule reference through its normal branch workflow. No ECU message change is necessary.
3. Add skipped-evaluation desire-pulse regression coverage and correct pulse consumption ordering.
4. Record and separately fix a pre-existing non-Turbo regression found during this review: [controlsd.py](openpilot/selfdrive/controls/controlsd.py) unconditionally asserts the Turbo applicator exists in the non-curvature branch, although it is only constructed for Turbo angle control. Keep this fix narrow; do not mix it into an unexplained lane-change rewrite.

### B. Shadow intent path and UI

1. Add request/state definitions in `openpilot/cereal/custom.capnp`, event bindings in `log.capnp`, and service/logging entries in `services.py`.
2. Add a pure GCS paddle interaction class, for example `openpilot/tools/turbo/intent.py`; integrate it into `g29d.py` without starting another HID reader or changing haptic tuning.
3. Extend the existing bridge allowlists, session lifecycle, and feedback profiles in `webrtc_controls.py`, `webrtc_signald.py`, `webrtc_uplink.py`, and `system/webrtc/webrtcd.py`. Enable request transport in both negotiated-UDP and legacy-SCTP configurations; no double delivery.
4. Add the Turbo-only UGV manager, for example `openpilot/selfdrive/modeld/turbo_intent.py`, and wire it into modeld's existing subscriptions/publications after vehicle configuration is known.
5. Display and log the full request lifecycle with model desire unchanged. Test real LTE single-press requests, L2/invalidation cancellation, and loss behavior in shadow mode.

### C. Gated model integration

1. Introduce the typed input boundary and configurable speed minimum in `desire_helper.py`, preserving stock defaults and adding road-car regression tests.
2. Integrate single-press authorization, freshness/override gates, pulse-consumed acknowledgment, timeout, and completion diagnostics.
3. Adapt Turbo prompts/indicator in `selfdrive/ui` and the actual GCS debug/onroad UI. Keep the existing simple camera UI in sync if it also displays intent.
4. Keep actuation behind a separate disabled-by-default execution flag until simulation and stationary tests pass.

### D. Closed-course validation

Verify speed/GNSS and link-loss behavior first. Then test one direction at a time with a spotter, marked interventions, recorded model output, and an independently usable stop/disengage procedure. Collect failures before changing haptic or speed thresholds.

Turning execution off must restore the existing steering-assist behavior and invalidate all pending intent. Rolling back must not replay a cached paddle request after the next startup.

## 12. Required test matrix

| Area | Minimum acceptance coverage |
| --- | --- |
| HID | Each paddle, both paddles, paddle + L2/L3/R2, transition edges, release order, disconnect/reconnect; no false request or lost cancel |
| GCS interaction | One press suffices; hold/bounce/backlog does not repeat; new requests require release and idle; opposing simultaneous inputs rejected; pending/executing inputs not queued; held paddle across engage/reconnect ignored; L2 dominates |
| Request transport | Conflation/backpressure, lost feedback/ack, duplicate/reordered revisions, delayed SCTP delivery, stale frozen context, future timestamps, epoch/session restart, unsupported peer/version |
| Lifecycle | Bounded request expiry on receipt and before evaluation, fresh press after rejection, no start on speed increase/override release after rejection, no queued second lane change, reconnect requires a new action |
| Model pulse | Exactly one consumed pulse per accepted single-press request; skipped frames preserve it within its deadline; duplicates do not repeat it; left/right slots correct; no start acknowledgment before evaluation |
| Stock regression | Normal blinkers/torque/BSM, 20 mph gate, lateral inactive/reset, timeouts, supported car control branches; no new dependencies for non-Turbo |
| Override | New requests rejected during override and release fade; real intervention still works mid-maneuver; large target steps, sign reversals, overshoot, quantized haptic target, stale feedback, rapid re-grab |
| Actuation separation | Intent never directly emits steering/throttle CAN; no headlight changes from lane-change metadata; normal headlight buttons preserved |
| Safety/scene | Forward versus reverse context, speed glitches, standstill, sensor/model invalidity, missing BSM treated as unavailable, alternate lateral maneuver mode rejected |
| Cancellation | Before-pulse cancel succeeds; after-pulse cancel reports too late/takeover required; no fake abort or opposite-direction pulse; L2 clears pending state and disengages through verified existing path |
| UI/logging | Pending/awaiting-evaluation/executing distinguishable, no arm/confirm prompt, stale state not success, orange still means applied override, actual pulse event and terminal results present in logs |
| Network failure | Pending intent cannot survive loss/restart; active-maneuver link-loss behavior matches the verified vehicle failsafe; intent traffic does not degrade the 50 Hz writer or steering feedback |

Reuse fake messages/clocks and existing tests in `openpilot/tools/turbo/tests/test_g29d.py`, `test_webrtc_controls.py`, `test_webrtc_signald.py`, `test_webrtc_uplink.py`, and `openpilot/selfdrive/controls/tests/test_turbo_steer_assist.py`. Add targeted model/desire and UI tests rather than relying only on end-to-end driving.

## 13. Checks actually performed for this report

- Read-only local code inspection, including every checked-out brand's input handling and controller blinker references; primary-source web lookup for upstream model/desire behavior, G29 hardware/button mapping, and WebRTC transport.
- Hardware-free decoder calls on six synthetic reports: individual paddles/L2 decode; combined paddle/L2/R2 reports lose the presses, as shown above. No wheel was opened or force command sent.
- Hardware-free `DesireHelper` checks: 6.15 m/s with a blinker remains off; crossing to 10 m/s with the same held blinker remains off; a new blinker at 10 m/s enters pre-lane-change; matching positive torque starts left desire; removing the blinker afterward leaves it starting; low model probability eventually completes it.
- Inspected the existing post-route report for speed and override limitations. No new remote connection, message subscriber, driving test, or full regression suite was run.

## Decisions to settle before execution is enabled

The first-test input decision is settled: **single paddle press, no arming cycle or nudge**.

1. Establish the Turbo speed window and model-response acceptance criteria in simulation and with verified physical speed. The 2 m/s candidate is not yet an approved operational threshold.
2. Verify existing global disengage and link-loss behavior during an already-started maneuver. If a lane-change-specific automatic abort/stop is wanted, design and validate it separately; `desire = none` is not that feature.

The smallest useful first delivery is **correct paddle decoding + acknowledged single-press shadow requests + a clear UI**, with the haptic steering behavior untouched. Once that path is proven, connect accepted requests to the existing local model desire mechanism under a separate execution gate.

## 14. Implemented single-paddle path

The branch now contains the shadow path **and** opt-in model integration. Default mode is `shadow`: no lane-change desire is emitted. The operator interaction is one fresh left/right paddle press, with no second pull or nudge in either mode.

### Implementation choices

- `openpilot/tools/turbo/g29_compat.py` subclasses the installed G29 reader to repair only byte-1 bit decoding. This supersedes the separate g29py release proposed above: no dependency release, new HID reader, or changes in `../g29-py` are needed. Hardware packet verification remains outstanding.
- The existing 50 Hz `g29d` loop owns `PaddleIntentController`. It records local status and retains an unacknowledged request/cancel for 10 Hz publication, using a frozen UGV feedback timestamp. No repeat on hold, no queue, and both paddles must be released before another request. L2 wins simultaneous command events in both reliable-command and legacy G29/CAN paths.
- Requests travel only over the existing reliable SCTP channel. Raw UDP remains allowlisted to `g29` and `turboSteerAssist`. Feedback adds `turboIntentState` to the existing `ui_model`/`steer_assist` profiles at 10 Hz.
- A third, **UGV-local-only** service, `turboIntentLinkState`, carries the WebRTC bridge's actual session ID and heartbeat. Network attempts to publish that service are rejected; intent requests must match the bridge's session. Cleanup publishes disconnected, and a stalled bridge heartbeat fails freshness checks.
- `TurboIntentManager`, owned by a Turbo-only modeld adapter, provides one-shot desires directly at the existing model input. This supersedes modifying the shared `DesireHelper`: ordinary cars retain their existing helper and 20 mph/torque/BSM rules. No fake CarState blinkers or steering torque.
- The wire protocol has `request` and terminal `cancel` actions for the same request ID, not a general revision counter. A cancel tombstones that identity even if local conflation delivers it before the request. Duplicate requests cannot revalidate a rejected action. Wrong-session/epoch traffic is ignored. A competing request cannot replace an active transaction; its sender may report unknown rather than receive a separate busy acknowledgment.
- Acceptance and actual inference consumption are distinct. `awaitingEvaluation` is not execution; `executing` records the frame and UGV monotonic time after evaluation. Skipped evaluations no longer consume the desire's rising edge. There is no additional model compilation/weight change.
- Turbo lane-change metadata does not set CarControl blinker outputs, because the checked-out Turbo controller maps them to headlights. Explicit D-pad headlights remain unchanged; no opendbc/ECU change required. The non-Turbo angle-control path also no longer asserts the presence of a Turbo override applicator.
- A compact directional/status label is present in the normal and Mici onroad renderers and camera-only GCS UI. It distinguishes local pending/unknown, UGV awaiting/executing, rejection reasons and stale feedback. Safety alerts draw above it. Orange override borders and all haptic/override tuning are unchanged.
- All three Cereal services are logged. Existing producer-written 50 Hz steering traces also contain intent requests, feedback, and paddle levels; `/tmp/turbo-metrics/latest/` contains the latest request/feedback. No ad-hoc live subscriber is needed.

### Gates and configuration

These environment variables are read by **UGV modeld at startup**, so changing them requires restarting the normal process stack:

| Variable | Default | Meaning |
| --- | --- | --- |
| `TURBO_INTENT_MODE` | `shadow` | `off`, `shadow`, or explicit `execute` |
| `TURBO_INTENT_MIN_SPEED` | `2.0` | Candidate minimum forward speed, m/s |
| `TURBO_INTENT_MAX_SPEED` | `5.0` | Candidate maximum forward speed, m/s |

The candidate window must hold for 0.5 s with healthy vehicle state before acceptance. It is **not physically validated**, and it is not an endorsement of lane-change behavior at those speeds. Shadow requests use the same gates; stationary presses therefore exercise rejection and transport, not `would_execute` acceptance.

Before a pulse, the UGV requires current session/epoch, a feedback reference no more than 350 ms old on its own clock, local link/control freshness, lateral engagement, enabled selfdrive state, calibrated status, valid CAN, no steering faults, no standstill/reverse, no reported blind spot, and no alternate lateral-maneuver mode. Fresh G29 and steering-assist messages/context are required. Local or applied steering override (including release fade) blocks entry. Turbo has no populated BSM sensors: absence of a flag does **not** establish that the adjacent area is clear.

Completion is deliberately narrow: model lane-change probability must first reach 0.02, then fall below 0.02 after at least 0.5 s. It is a model-reported result, not proof of a successful or safe physical lane change. No response after 2 s, invalid probability, or an execution timeout after 10 s produces a takeover-required state that prevents another request until disengagement/reset. These thresholds remain experimental.

Once a pulse has been consumed, link loss, cancellation, or wheel override is **not** reported as a successful abort. The existing wheel takeover path still acts on steering; releasing it can hand control back to a model that remains in the maneuver. L2 uses the existing global disengage path. There is no new automatic braking, stop, or return-to-lane failsafe in this change.

### Verification and first boot

Hardware-free tests cover all 256 byte-1 masks and combination transitions, one-shot/held/reconnect behavior, L2 priority, expiry and frozen context, deduplication, canceled-request tombstones, unknown acknowledgments, local health/override gates, bridge session validation and UDP exclusion, actual model-run skipped-frame ordering, model response/timeout states, UI labels, normal-car helper behavior, and the Turbo headlight guard. Existing haptic, teleop, UI-state and transport regressions are included. Cereal's native schema/library build is checked separately.

Final local result: **492 tests passed** across `openpilot/tools/turbo/tests`, the intent/steering-assist controls tests, modeld intent tests, UI Turbo-state tests and WebRTC stream-session tests. `.venv/bin/scons -j4 openpilot/cereal` passed. Ruff passed for changed Python files, with only the existing `UP031` finding in the unchanged header-formatting code of `cereal/services.py` excluded (confirmed also present at HEAD). `git diff --check` passed. No hardware, GPU inference, physical maneuver, LTE latency or vehicle failsafe validation is claimed by these tests.

Deployment requires both GCS and UGV on the same schema/code revision, with native targets rebuilt through the normal build/launch workflow. Older peers do not advertise the required feedback/session and cannot issue an executable request. No UGV update or reboot has been performed; wait for the user to boot it.

First test remains shadow-only: verify real paddle direction/combined L2 packets while stationary, confirm explicit rejection at standstill, then verify acknowledged `would_execute` on a clear course with validated speed. Confirm existing L2 and link-loss behavior and exercise the haptic handback before setting `TURBO_INTENT_MODE=execute`. Keep the normal GCS launch in charge of its output and use producer metrics/offline logs for inspection.
