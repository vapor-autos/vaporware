"""Single-press GCS intent interaction, driven by the existing 50 Hz reader."""
import uuid

from openpilot.selfdrive.controls.lib.turbo_intent import BUSY_STATUSES, CONTEXT_TIMEOUT_S, PROTOCOL_VERSION


class PaddleIntentController:
  def __init__(self):
    self.operator_id = uuid.uuid4().hex
    self.sequence = 0
    self.released = False
    self.context = ("", "")
    self.pending: dict | None = None
    self.pending_since = 0.0
    self.last_status = "idle"
    self.status_since = 0.0
    self.uncertain = False

  def update(self, buttons: dict, events: list[dict], feedback: dict, feedback_time: int,
             fresh: bool, ready: bool, reverse: bool, now: float) -> dict:
    left, right = bool(buttons.get("left_paddle")), bool(buttons.get("right_paddle"))
    downs = [e.get("control") for e in events if e.get("type") == "button_down"]
    paddle_downs = [b for b in downs if b in ("left_paddle", "right_paddle")]
    context = (feedback.get("sessionId", ""), feedback.get("epoch", "")) if fresh else ("", "")
    changed = context != self.context
    if changed:
      self.context, self.pending, self.released = context, None, False
      self.uncertain = False
      self.last_status = "idle"
    can_press = self.released and not changed
    if left or right or paddle_downs:
      self.released = False
    elif not left and not right:
      self.released = True

    wire = None
    if self.pending is not None:
      ack = (fresh and feedback.get("requestId") == self.pending["requestId"] and
             feedback.get("operatorId") == self.operator_id and feedback.get("status") != "idle")
      invalidate = "L2" in downs or bool(buttons.get("L2")) or not ready or reverse or not fresh
      if invalidate:
        if self.pending["action"] != "cancel":
          self.pending = {**self.pending, "action": "cancel", "baseFeedbackLogMonoTime": feedback_time}
        wire = self.pending
        self.last_status, self.status_since = "canceled", now
      if self.pending["action"] == "cancel" and feedback.get("status") == "awaitingEvaluation":
        ack = False  # Receipt of the request is not acknowledgment of its cancellation.
      if ack:
        self.last_status, self.status_since = feedback["status"], now
        self.pending = None
        self.uncertain = False
        wire = None
      elif now - self.pending_since > CONTEXT_TIMEOUT_S:
        self.last_status, self.status_since = "unknown", now
        # Do not automatically issue another request when an acknowledgment is lost.
        self.pending = None
        self.uncertain = True
        wire = None
      else:
        wire = self.pending

    busy = self.pending is not None or (fresh and feedback.get("status") in BUSY_STATUSES)
    if paddle_downs and can_press:
      reason = ""
      if "L2" in downs or bool(buttons.get("L2")) or "L3" in downs:
        reason = "cancel_or_engage"
      elif len(paddle_downs) != 1 or (left and right):
        reason = "ambiguous_paddles"
      elif busy or self.uncertain:
        reason = "busy_or_unknown"
      elif not fresh or feedback.get("protocolVersion") != PROTOCOL_VERSION or not all(context):
        reason = "feedback_unavailable"
      elif not ready or reverse:
        reason = "operator_not_ready"
      if reason:
        self.last_status, self.status_since = reason, now
      else:
        self.sequence += 1
        self.pending_since = now
        self.pending = {
          "protocolVersion": PROTOCOL_VERSION, "operatorId": self.operator_id, "requestId": self.sequence,
          "sessionId": context[0], "epoch": context[1], "action": "request",
          "direction": "left" if paddle_downs[0] == "left_paddle" else "right",
          "baseFeedbackLogMonoTime": feedback_time, "ready": ready, "reverse": reverse,
          "createdMonoTime": int(now * 1e9),
        }
        wire = self.pending
        self.last_status, self.status_since = "pending", now

    if fresh and self.uncertain and feedback.get("operatorId") == self.operator_id and feedback.get("requestId") == self.sequence:
      self.last_status, self.status_since = feedback.get("status", "unknown"), now
      self.uncertain = False
    if self.last_status not in ("pending", "unknown") and now - self.status_since > 2.0:
      self.last_status = "idle"
    return {**(wire or {"action": "none"}), "localStatus": "unknown" if self.uncertain else self.last_status}
