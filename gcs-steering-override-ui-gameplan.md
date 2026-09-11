# GCS Steering Override Indicator Gameplan

## Goal

Make the GCS clearly show when the UGV is actually applying the remote steering
override. The existing green border continues to mean that self-driving is
engaged. While the UGV is applying the override, the GCS border becomes amber.

The display must be driven by the UGV's final control decision, not by the G29
request on the GCS. This prevents the UI from claiming an override when the
request was rejected because it was stale, invalid, out of order, mismatched to
the model context, or disabled by configuration.

## State and transport

1. Add a logged `turboSteerAssistState` Cereal event published by `controlsd` on
   the UGV at 20 Hz.
2. Include at least:
   - whether the override was applied;
   - the decision/status reason;
   - whether a requested target was available;
   - requested, model, and final steering angles;
   - the source sequence and source model timestamp.
3. Add the event to the existing `steer_assist` feedback profile. The existing
   feedback bridge republishes it into the GCS message graph, so the standard UI
   state subscriber can consume it without creating another subscriber or live
   monitor.

With the current `TURBO_GCS_FEEDBACK_UDP_HOST` configuration, the event travels
UGV-to-GCS over Tailscale UDP on the existing feedback path. If the GCS does not
advertise a feedback UDP endpoint when the WebRTC session is negotiated, the
same event uses the WebRTC data channel (SCTP). The current transport makes this
choice per session; it does not dynamically switch from UDP to SCTP in response
to packet loss.

## UGV behavior

`controlsd` remains the authority because it knows both the validated assist
decision and the angle ultimately written to `CarControl`.

- `applied=true` only when assist application is enabled and a validated remote
  target replaces the model angle.
- A valid target received while application is disabled reports
  `apply_disabled`, not applied.
- Rejected targets preserve the existing validator status such as `stale`,
  `out_of_order`, or `target_mismatch`.
- The state event is valid only when the underlying control input is valid and
  is route-logged for later diagnosis.

## GCS UI behavior

- Enable the additional UI subscription only in `gcs_debug_ui`; the normal
  device UI remains unchanged.
- Require the feedback service to be seen, alive, valid, and `applied=true`.
- Also require the GCS UI to be on-road and self-driving to be enabled.
- Use amber/orange for the border without an additional text badge.
- Retain the existing colors otherwise: green for engaged, blue for disengaged,
  and gray for openpilot's existing pre-enabled/driver-override status.
- On stale or missing feedback, fail visually back to the ordinary engagement
  color rather than leaving an override indication latched.

## Verification

- Unit-test applied, disabled, and rejected state resolution.
- Test that the `steer_assist` feedback profile carries the new event and that
  packed Cereal feedback is republished correctly.
- Run the focused steering-assist and WebRTC-control tests.
- Run formatting/static checks for every edited Python file and validate the
  Cap'n Proto schema through the Cereal test/build tooling.
- Inspect the final diff and commit only the files belonging to this change;
  preserve unrelated local worktree changes.

## Outdoor-test acceptance criteria

- Green border when engaged with no applied manual steering override.
- Amber border while the UGV applies the override.
- Return to green within the feedback liveness window after release, rejection,
  disconnect, or stale input.
- Route logs show the applied flag, status, source lineage, and final angle for
  any unexpected transition.
