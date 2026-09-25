import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.turbo_intent import PROTOCOL_VERSION, REQUEST_SERVICE, STATE_SERVICE, FEEDBACK_TIMEOUT_S


_STATUS_TEXT = {
  "pending": "REQUESTING", "awaitingEvaluation": "WAITING FOR MODEL", "executing": "EXECUTING",
  "shadow": "SHADOW OK", "rejected": "REJECTED", "completed": "RESPONSE CLEARED",
  "responseCleared": "RESPONSE CLEARED", "unconfirmed": "UNCONFIRMED", "stopped": "STOPPED", "faulted": "CHECK CONTROL",
  "ugv_unavailable": "WAITING", "rejected_busy": "BUSY",
  "canceling": "CANCELING", "canceled": "CANCELED", "expired": "EXPIRED", "interrupted": "INTERRUPTED",
  "unknown": "STATUS UNKNOWN", "timedOut": "TAKE OVER", "noModelResponse": "TAKE OVER",
  "operator_not_ready": "NOT READY", "feedback_unavailable": "LINK STALE", "busy_or_unknown": "BUSY",
  "ambiguous_paddles": "RELEASE BOTH PADDLES", "cancel_or_engage": "ENGAGE / CANCEL INPUT",
  "ambiguous_maneuvers": "RELEASE MANEUVER BUTTONS",
}
_REASON_TEXT = {
  "standstill": "STOPPED", "not_engaged": "NOT ENGAGED", "vehicle_unhealthy": "CHECK VEHICLE",
  "steering_override": "WHEEL OVERRIDE", "operator_overriding": "WHEEL OVERRIDE", "operator_stale": "INPUT STALE",
  "link_stale": "LINK STALE", "operator_link_lost": "LINK LOST", "reverse": "REVERSE",
  "motion_not_stable": "SPEED NOT STABLE", "stale_context": "REQUEST TOO OLD", "disabled": "OFF",
  "already_started_use_l2": "USE L2 TO DISENGAGE", "operator_cancel": "", "would_execute": "",
  "model_reports_complete": "", "takeover_required": "", "engagement_or_session_reset": "SESSION RESET",
  "model_turn_response_cleared": "MODEL RESPONSE CLEARED",
}


def _reason_text(state) -> str:
  reason = str(state.reason)
  if reason == "speed_out_of_range":
    if state.maxSpeed == 0:
      return f"SPEED >= {state.minSpeed:g} M/S REQUIRED" if state.minSpeed > 0 else "VALID FORWARD SPEED REQUIRED"
    return f"SPEED {state.minSpeed:g}-{state.maxSpeed:g} M/S REQUIRED"
  return _REASON_TEXT.get(reason, reason.replace("_", " ").upper())


def _idle_label(state) -> str:
  mode = "SHADOW" if state.mode == "shadow" else "OFF" if state.mode == "off" else "READY" if state.available else "WAITING"
  # A terminal reason describes an old request, not necessarily the current gate.
  reason = _reason_text(state) if state.status == "idle" and not state.available else ""
  uncapped = " | NO SPEED CAP" if state.mode == "execute" and state.maxSpeed == 0 else ""
  return f"PADDLES: {mode}{uncapped}" + (f" | {reason}" if reason and reason != mode else "")


def intent_label(sm, now: float) -> str:
  """Human-readable diagnostics; on-screen intent feedback uses borders only."""
  if STATE_SERVICE not in sm.seen or not sm.seen[STATE_SERVICE]:
    return ""
  if not sm.valid[STATE_SERVICE] or not 0 <= now - sm.recv_time[STATE_SERVICE] <= FEEDBACK_TIMEOUT_S:
    return "LANE CHANGE: LINK STALE"
  state = sm[STATE_SERVICE]
  if state.protocolVersion != PROTOCOL_VERSION:
    return "INTENT: PROTOCOL MISMATCH"
  direction = str(state.direction).upper() if str(state.direction) in ("left", "right") else ""
  kind = "TURN" if str(state.maneuver) == "turn" else "LANE CHANGE"
  prefix = f"{direction} {kind}".strip()
  status = str(state.status)
  if status in ("executing", "timedOut", "noModelResponse"):
    suffix = "WHEEL OVERRIDE" if state.operatorOverride and status == "executing" else _reason_text(state)
    return f"{prefix}: {_STATUS_TEXT[status]}" + (f" | {suffix}" if suffix else "")
  if REQUEST_SERVICE in sm.seen and sm.seen[REQUEST_SERVICE]:
    if not sm.valid[REQUEST_SERVICE] or not 0 <= now - sm.recv_time[REQUEST_SERVICE] <= FEEDBACK_TIMEOUT_S:
      return "PADDLES: INPUT STALE"
    local = sm[REQUEST_SERVICE]
    local_status = str(local.localStatus)
    acknowledged = (state.requestId > 0 and state.requestId == local.requestId and
                    state.operatorId == local.operatorId and state.maneuver == local.maneuver and
                    state.direction == local.direction and status != "idle")
    if (local_status not in ("", "idle", "completed", "shadow", "rejected", "awaitingEvaluation", "executing") and
        not (local_status == "pending" and acknowledged)):
      local_kind = "TURN" if str(local.maneuver) == "turn" else "LANE CHANGE"
      return local_kind + ": " + _STATUS_TEXT.get(local_status, local_status.replace("_", " ").upper())
    if status != "awaitingEvaluation" and (local_status == "idle" or (local.operatorId and local.operatorId != state.operatorId)):
      return _idle_label(state)
  if status == "idle":
    return _idle_label(state)
  reason = _reason_text(state)
  return f"{prefix}: {_STATUS_TEXT.get(status, status.upper())}" + (f" | {reason}" if reason else "")


LANE_CHANGE_COLOR = (0xAD, 0x66, 0xFF, 0xFF)  # Violet; distinct from engagement/override/alerts.
COOLDOWN_COLOR = (0x68, 0x3D, 0x99, 0xFF)
TAKEOVER_COLOR = (0xC9, 0x22, 0x31, 0xFF)


@dataclass(frozen=True)
class IntentBorder:
  side: str = "none"
  takeover: bool = False
  dim: bool = False


def intent_color(visual: IntentBorder):
  return COOLDOWN_COLOR if visual.dim else LANE_CHANGE_COLOR


def intent_border(sm, now: float, *, engaged: bool, override: bool = False, critical: bool = False) -> IntentBorder:
  if critical:
    return IntentBorder(takeover=True)
  if not engaged or not sm.seen.get(STATE_SERVICE, False):
    return IntentBorder()
  state = sm[STATE_SERVICE]
  if state.protocolVersion != PROTOCOL_VERSION:
    return IntentBorder(takeover=str(state.status) in ("executing", "timedOut", "noModelResponse"))
  if state.mode != "execute":
    return IntentBorder()
  fresh = sm.valid[STATE_SERVICE] and 0 <= now - sm.recv_time[STATE_SERVICE] <= FEEDBACK_TIMEOUT_S
  active = str(state.status) in ("awaitingEvaluation", "executing") or str(state.phase) == "cooldown"
  local_unknown = (sm.seen.get(REQUEST_SERVICE, False) and sm.valid[REQUEST_SERVICE] and
                   0 <= now - sm.recv_time[REQUEST_SERVICE] <= FEEDBACK_TIMEOUT_S and
                   str(sm[REQUEST_SERVICE].localStatus) == "unknown")
  # A bounded request outcome is not a control fault. Never infer success from
  # missing authoritative feedback, however, even during cooldown/reconciliation.
  if local_unknown or (active and not fresh) or (fresh and state.faultReason):
    return IntentBorder(takeover=True)
  if not fresh or override or state.operatorOverride:
    return IntentBorder()
  side = str(state.direction)
  if side not in ("left", "right"):
    return IntentBorder()
  if state.status == "executing":
    return IntentBorder(side=side)
  if state.phase == "cooldown":
    age = state.outcomeAgeS + max(0.0, now - sm.recv_time[STATE_SERVICE])
    if state.outcome in ("unconfirmed", "expired") and age < 1.0:
      return IntentBorder(side=side, dim=not (age < 0.2 or 0.4 <= age < 0.6))
    return IntentBorder(side=side, dim=True)
  return IntentBorder()


def half_border_clip(rect, side: str, thickness: float) -> tuple[int, int, int, int]:
  # Include the stroke outside the rounded rectangle, split at its center.
  left, right = math.floor(rect.x - thickness), math.ceil(rect.x + rect.width + thickness)
  top, bottom = math.floor(rect.y - thickness), math.ceil(rect.y + rect.height + thickness)
  center = math.floor(rect.x + rect.width / 2)
  return (left, top, center - left, bottom - top) if side == "left" else (center, top, right - center, bottom - top)


def draw_intent_border(rect, visual: IntentBorder, thickness: float, roundness: float = 0.12):
  """Overlay the existing outline; call outside any other scissor region."""
  import pyray as rl
  if visual.takeover:
    rl.draw_rectangle_rounded_lines_ex(rect, roundness, 10, thickness, rl.Color(*TAKEOVER_COLOR))
    return
  if visual.side not in ("left", "right"):
    return
  rl.begin_scissor_mode(*half_border_clip(rect, visual.side, thickness))
  try:
    # Clip a complete outline, not a half-width rectangle: no divider through video.
    rl.draw_rectangle_rounded_lines_ex(rect, roundness, 10, thickness, rl.Color(*intent_color(visual)))
  finally:
    rl.end_scissor_mode()


def split_border_segment(start, end, center_x: float, side: str):
  """Split projected box edges without disturbing the camera's scissor region."""
  if side not in ("left", "right"):
    return [(start, end, False)]
  points = [start, end]
  if (start[0] < center_x < end[0]) or (end[0] < center_x < start[0]):
    t = (center_x - start[0]) / (end[0] - start[0])
    points.insert(1, (center_x, start[1] + t * (end[1] - start[1])))
  return [(a, b, ((a[0] + b[0]) / 2 < center_x) == (side == "left")) for a, b in zip(points, points[1:], strict=False)]
