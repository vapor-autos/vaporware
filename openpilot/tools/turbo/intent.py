"""Single-press GCS intent interaction, driven by the existing 50 Hz reader."""
import uuid
import time
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.turbo_intent import ACK_TIMEOUT_S, BUSY_STATUSES, CONTEXT_TIMEOUT_S, FEEDBACK_TIMEOUT_S, PROTOCOL_VERSION, STATE_SERVICE

MANEUVER_BUTTONS = {
  "left_paddle": ("laneChange", "left"), "right_paddle": ("laneChange", "right"),
  "S": ("turn", "left"), "O": ("turn", "right"),
}


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


class ManeuverIntentController:
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

  def _acknowledgment(self, feedback: dict) -> str | None:
    if self.pending is None:
      return None
    keys = ("protocolVersion", "sessionId", "epoch", "operatorId", "requestId", "maneuver", "direction")
    def matches(data):
      return all(data.get(k) == self.pending.get(k) for k in keys)

    receipt = feedback.get("receipt", {})
    resolved = ("accepted", "rejected", "rejected_busy", "retired", "canceled_before_consumption", "already_consumed")
    if matches(receipt) and receipt.get("action") == self.pending["action"] and receipt.get("result") in resolved:
      return receipt.get("status") or receipt["result"]
    status = feedback.get("status")
    if matches(feedback) and status not in (None, "idle"):
      # The active state itself is also an acknowledgement, except that receiving
      # a pending request is not proof that its cancellation was processed.
      if self.pending["action"] == "request" or feedback.get("pulseMonoTime", 0) > 0 or status not in BUSY_STATUSES:
        return status
    return None

  def update(self, buttons: dict, events: list[dict], feedback: dict, feedback_time: int,
             fresh: bool, ready: bool, reverse: bool, now: float) -> dict:
    held = [name for name in MANEUVER_BUTTONS if buttons.get(name)]
    downs = [e.get("control") for e in events if e.get("type") == "button_down"]
    maneuver_downs = [b for b in downs if b in MANEUVER_BUTTONS]
    context = (feedback.get("sessionId", ""), feedback.get("epoch", "")) if fresh else self.context
    changed = fresh and feedback.get("protocolVersion") == PROTOCOL_VERSION and all(context) and context != self.context
    if changed:
      self.context, self.pending, self.released = context, None, False
      self.uncertain = False
      self.last_request = {}
      self.last_status = "idle"
    can_press = self.released and not changed and fresh
    if not fresh or held or maneuver_downs:
      self.released = False
    else:
      self.released = True

    wire = None
    if self.pending is not None:
      invalidate = "L2" in downs or bool(buttons.get("L2")) or not ready or reverse or not fresh
      if invalidate and self.pending["action"] != "cancel":
        self.pending = {**self.pending, "action": "cancel"}
        self.last_status, self.status_since = "canceling", now
      ack = self._acknowledgment(feedback) if fresh else None
      if ack is not None:
        self.last_status, self.status_since = ack, now
        self.pending = None
        self.uncertain = False
      else:
        age = now - self.pending_since
        if age >= ACK_TIMEOUT_S:
          if not self.uncertain:
            self.last_status, self.status_since = "unknown", now
          self.uncertain = True
          # Reconcile/tombstone the same ID; never turn a delayed request into a
          # fresh executable command or clear uncertainty on a local timer.
          self.pending = {**self.pending, "action": "cancel"}
        if self.pending["action"] == "cancel":
          if fresh and feedback.get("protocolVersion") == PROTOCOL_VERSION:
            self.pending = {**self.pending, "baseFeedbackLogMonoTime": feedback_time}
            wire = self.pending
        elif age <= CONTEXT_TIMEOUT_S:
          wire = self.pending

    busy = self.pending is not None or (fresh and feedback.get("status") in BUSY_STATUSES)
    if maneuver_downs and can_press:
      reason = ""
      if "L2" in downs or bool(buttons.get("L2")) or "L3" in downs or bool(buttons.get("L3")):
        reason = "cancel_or_engage"
      elif len(maneuver_downs) != 1 or len(set(held + maneuver_downs)) != 1:
        reason = "ambiguous_maneuvers"
      elif busy or self.uncertain:
        reason = "busy_or_unknown"
      elif not fresh or feedback.get("protocolVersion") != PROTOCOL_VERSION or not all(context):
        reason = "feedback_unavailable"
      elif not ready or reverse:
        reason = "operator_not_ready"
      elif not feedback.get("available", False):
        reason = "ugv_unavailable"
      if reason:
        self.last_status, self.status_since = reason, now
      else:
        self.sequence += 1
        self.pending_since = now
        maneuver, direction = MANEUVER_BUTTONS[maneuver_downs[0]]
        self.pending = {
          "protocolVersion": PROTOCOL_VERSION, "operatorId": self.operator_id, "requestId": self.sequence,
          "sessionId": context[0], "epoch": context[1], "action": "request",
          "direction": direction, "maneuver": maneuver,
          "baseFeedbackLogMonoTime": feedback_time, "ready": ready, "reverse": reverse,
          "createdMonoTime": int(now * 1e9),
        }
        self.last_request = self.pending.copy()
        wire = self.pending
        self.last_status, self.status_since = "pending", now

    if self.last_status not in ("pending", "unknown") and now - self.status_since > 2.0:
      self.last_status = "idle"
    # Retain identity for UI correlation even after the action is acknowledged.
    return {**(wire or {**self.last_request, "operatorId": self.operator_id, "action": "none"}),
            "localStatus": "unknown" if self.uncertain else self.last_status}


# Compatibility for existing callers; both names use the same shared transaction slot.
PaddleIntentController = ManeuverIntentController


class IntentPublishSchedule:
  """Send edges immediately; retry/refresh at the existing 10 Hz cadence."""
  def __init__(self):
    self.signature = None

  def update(self, request: dict, heartbeat: bool) -> bool:
    signature = tuple(request.get(k) for k in ("operatorId", "requestId", "sessionId", "epoch", "action", "maneuver", "direction"))
    publish = heartbeat or signature != self.signature
    self.signature = signature
    return publish
