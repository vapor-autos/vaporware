"""Single-press GCS intent interaction, driven by the existing 50 Hz reader."""
import uuid
import time
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.turbo_intent import BUSY_STATUSES, CONTEXT_TIMEOUT_S, FEEDBACK_TIMEOUT_S, PROTOCOL_VERSION, STATE_SERVICE


@dataclass(frozen=True)
class IntentFeedback:
  now: float
  data: dict
  log_mono_time: int
  fresh: bool
  applied_fresh: bool
  applied: bool
  age_s: float | None
  applied_age_s: float | None


def read_intent_feedback(sm, now: float | None = None) -> IntentFeedback:
  # Call AFTER polling the existing SubMaster. A loop-start timestamp precedes
  # its recv_time and incorrectly rejects every newly received packet as future.
  now = time.monotonic() if now is None else now
  applied_service = "turboSteerAssistState"
  age = now - sm.recv_time[STATE_SERVICE] if sm.seen[STATE_SERVICE] else None
  applied_age = now - sm.recv_time[applied_service] if sm.seen[applied_service] else None
  fresh = age is not None and sm.valid[STATE_SERVICE] and 0 <= age <= FEEDBACK_TIMEOUT_S
  applied_fresh = applied_age is not None and sm.valid[applied_service] and 0 <= applied_age <= FEEDBACK_TIMEOUT_S
  return IntentFeedback(now, sm[STATE_SERVICE].to_dict() if fresh else {}, sm.logMonoTime[STATE_SERVICE],
                        fresh, applied_fresh, bool(sm[applied_service].applied), age, applied_age)


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
    self.last_request: dict = {}

  def update(self, buttons: dict, events: list[dict], feedback: dict, feedback_time: int,
             fresh: bool, ready: bool, reverse: bool, now: float) -> dict:
    left, right = bool(buttons.get("left_paddle")), bool(buttons.get("right_paddle"))
    downs = [e.get("control") for e in events if e.get("type") == "button_down"]
    paddle_downs = [b for b in downs if b in ("left_paddle", "right_paddle")]
    context = (feedback.get("sessionId", ""), feedback.get("epoch", "")) if fresh else self.context
    changed = fresh and all(context) and context != self.context
    if changed:
      self.context, self.pending, self.released = context, None, False
      self.uncertain = False
      self.last_request = {}
      self.last_status = "idle"
    can_press = self.released and not changed and fresh
    if not fresh or left or right or paddle_downs:
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
          self.pending = {**self.pending, "action": "cancel",
                          "baseFeedbackLogMonoTime": feedback_time if fresh else self.pending["baseFeedbackLogMonoTime"]}
          self.last_status, self.status_since = "canceling", now
        wire = self.pending
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
        self.last_request = self.pending.copy()
        wire = self.pending
        self.last_status, self.status_since = "pending", now

    if (fresh and self.uncertain and feedback.get("operatorId") == self.operator_id and feedback.get("requestId") == self.sequence and
        feedback.get("status") not in (None, "idle")):
      self.last_status, self.status_since = feedback.get("status", "unknown"), now
      self.uncertain = False
    if self.last_status not in ("pending", "unknown") and now - self.status_since > 2.0:
      self.last_status = "idle"
    # Retain identity for UI correlation even after the action is acknowledged.
    return {**(wire or {**self.last_request, "operatorId": self.operator_id, "action": "none"}),
            "localStatus": "unknown" if self.uncertain else self.last_status}
