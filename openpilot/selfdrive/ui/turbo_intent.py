import time

from openpilot.selfdrive.controls.lib.turbo_intent import REQUEST_SERVICE, STATE_SERVICE, FEEDBACK_TIMEOUT_S


def intent_label(sm, now: float) -> str:
  if STATE_SERVICE not in sm.seen or not sm.seen[STATE_SERVICE]:
    return ""
  if not sm.valid[STATE_SERVICE] or not 0 <= now - sm.recv_time[STATE_SERVICE] <= FEEDBACK_TIMEOUT_S:
    return "LANE CHANGE: LINK STALE"
  state = sm[STATE_SERVICE]
  direction = str(state.direction).upper()
  status = str(state.status)
  if status in ("executing", "timedOut", "noModelResponse"):
    suffix = "OVERRIDE" if state.operatorOverride else str(state.reason).replace("_", " ").upper()
    return f"{direction} LANE CHANGE: {status.upper()} {suffix}".strip()
  if (REQUEST_SERVICE in sm.seen and sm.seen[REQUEST_SERVICE] and sm.valid[REQUEST_SERVICE] and
      0 <= now - sm.recv_time[REQUEST_SERVICE] <= FEEDBACK_TIMEOUT_S):
    local_status = str(sm[REQUEST_SERVICE].localStatus)
    acknowledged = (state.requestId > 0 and state.requestId == sm[REQUEST_SERVICE].requestId and
                    state.operatorId == sm[REQUEST_SERVICE].operatorId and status != "idle")
    if (local_status not in ("", "idle", "completed", "shadow", "rejected", "awaitingEvaluation", "executing") and
        not (local_status == "pending" and acknowledged)):
      return "LANE CHANGE: " + local_status.replace("_", " ").upper()
  if status == "idle":
    return "PADDLES: " + ("SHADOW" if state.mode == "shadow" else "READY" if state.available else str(state.reason).replace("_", " ").upper())
  return f"{direction} LANE CHANGE: {status.upper()} {str(state.reason).replace('_', ' ').upper()}".strip()


def draw_intent_status(sm, rect):
  # Imported only by the UI; label logic remains usable in hardware-free tests.
  import pyray as rl
  from openpilot.system.ui.lib.application import gui_app, FontWeight
  from openpilot.system.ui.lib.text_measure import measure_text_cached

  label = intent_label(sm, time.monotonic())
  if not label:
    return
  font = gui_app.font(FontWeight.MEDIUM)
  size = int(max(16, min(28, rect.width / 50)))
  while len(label) > 4 and measure_text_cached(font, label, size).x > rect.width - 56:
    label = label[:-4] + "..."
  width = min(rect.width - 32, measure_text_cached(font, label, size).x + 24)
  x, y = rect.x + (rect.width - width) / 2, rect.y + rect.height * 0.76
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, size + 20), 0.2, 6, rl.Color(0, 0, 0, 190))
  rl.draw_text_ex(font, label, rl.Vector2(x + 12, y + 10), size, 0, rl.WHITE)
