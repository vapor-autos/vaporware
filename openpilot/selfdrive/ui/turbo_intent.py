import time

from openpilot.selfdrive.controls.lib.turbo_intent import REQUEST_SERVICE, STATE_SERVICE, FEEDBACK_TIMEOUT_S


_STATUS_TEXT = {
  "pending": "REQUESTING", "awaitingEvaluation": "WAITING FOR MODEL", "executing": "EXECUTING",
  "shadow": "SHADOW OK", "rejected": "REJECTED", "completed": "MODEL COMPLETE",
  "canceling": "CANCELING", "canceled": "CANCELED", "expired": "EXPIRED", "interrupted": "INTERRUPTED",
  "unknown": "STATUS UNKNOWN", "timedOut": "TAKE OVER", "noModelResponse": "TAKE OVER",
  "operator_not_ready": "NOT READY", "feedback_unavailable": "LINK STALE", "busy_or_unknown": "BUSY",
  "ambiguous_paddles": "RELEASE BOTH PADDLES", "cancel_or_engage": "ENGAGE / CANCEL INPUT",
}
_REASON_TEXT = {
  "standstill": "STOPPED", "not_engaged": "NOT ENGAGED", "vehicle_unhealthy": "CHECK VEHICLE",
  "steering_override": "WHEEL OVERRIDE", "operator_overriding": "WHEEL OVERRIDE", "operator_stale": "INPUT STALE",
  "link_stale": "LINK STALE", "operator_link_lost": "LINK LOST", "reverse": "REVERSE",
  "motion_not_stable": "SPEED NOT STABLE", "stale_context": "REQUEST TOO OLD", "disabled": "OFF",
  "already_started_use_l2": "USE L2 TO DISENGAGE", "operator_cancel": "", "would_execute": "",
  "model_reports_complete": "", "takeover_required": "", "engagement_or_session_reset": "SESSION RESET",
}


def _reason_text(state) -> str:
  reason = str(state.reason)
  if reason == "speed_out_of_range":
    return f"SPEED {state.minSpeed:g}-{state.maxSpeed:g} M/S REQUIRED"
  return _REASON_TEXT.get(reason, reason.replace("_", " ").upper())


def _idle_label(state) -> str:
  mode = "SHADOW" if state.mode == "shadow" else "OFF" if state.mode == "off" else "READY" if state.available else "WAITING"
  # A terminal reason describes an old request, not necessarily the current gate.
  reason = _reason_text(state) if state.status == "idle" and not state.available else ""
  return f"PADDLES: {mode}" + (f" | {reason}" if reason and reason != mode else "")


def intent_label(sm, now: float) -> str:
  if STATE_SERVICE not in sm.seen or not sm.seen[STATE_SERVICE]:
    return ""
  if not sm.valid[STATE_SERVICE] or not 0 <= now - sm.recv_time[STATE_SERVICE] <= FEEDBACK_TIMEOUT_S:
    return "LANE CHANGE: LINK STALE"
  state = sm[STATE_SERVICE]
  direction = str(state.direction).upper() if str(state.direction) in ("left", "right") else ""
  prefix = f"{direction} LANE CHANGE".strip()
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
                    state.operatorId == local.operatorId and status != "idle")
    if (local_status not in ("", "idle", "completed", "shadow", "rejected", "awaitingEvaluation", "executing") and
        not (local_status == "pending" and acknowledged)):
      return "LANE CHANGE: " + _STATUS_TEXT.get(local_status, local_status.replace("_", " ").upper())
    if status != "awaitingEvaluation" and (local_status == "idle" or (local.operatorId and local.operatorId != state.operatorId)):
      return _idle_label(state)
  if status == "idle":
    return _idle_label(state)
  reason = _reason_text(state)
  return f"{prefix}: {_STATUS_TEXT.get(status, status.upper())}" + (f" | {reason}" if reason else "")


def intent_panel_geometry(rect) -> tuple[float, float, float, float, int]:
  # Fixed footprint: changing status text must not resize/recenter the badge.
  size = int(max(16, min(28, rect.width / 50)))
  width = max(0, min(rect.width - 32, max(340, rect.width * 0.45)))
  height = size * 1.25 + 20
  return rect.x + (rect.width - width) / 2, rect.y + rect.height * 0.86 - height / 2, width, height, size


def draw_intent_status(sm, rect):
  # Imported only by the UI; label logic remains usable in hardware-free tests.
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight
  from openpilot.system.ui.widgets.label import gui_label

  label = intent_label(sm, time.monotonic())
  if not label:
    return
  x, y, width, height, size = intent_panel_geometry(rect)
  if width <= 24:
    return
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, height), 0.2, 6, rl.Color(0, 0, 0, 190))
  gui_label(rl.Rectangle(x + 12, y, width - 24, height), label, font_size=size, font_weight=FontWeight.MEDIUM,
            alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER)
