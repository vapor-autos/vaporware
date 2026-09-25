"""UGV-local one-shot requests. Response tracking is not physical maneuver completion."""
from collections import deque
from dataclasses import dataclass
import math
import uuid

from openpilot.cereal import log

PROTOCOL_VERSION = 3
CONTEXT_TIMEOUT_S = 0.35
FEEDBACK_TIMEOUT_S = 0.35
ACK_TIMEOUT_S = 1.0
REQUEST_SERVICE = "turboIntentRequest"
STATE_SERVICE = "turboIntentState"
LINK_SERVICE = "turboIntentLinkState"
BUSY_STATUSES = frozenset(("awaitingEvaluation", "executing"))
MANEUVER_DESIRES = {
  ("laneChange", "left"): log.Desire.laneChangeLeft, ("laneChange", "right"): log.Desire.laneChangeRight,
  ("turn", "left"): log.Desire.turnLeft, ("turn", "right"): log.Desire.turnRight,
}
TURN_RESPONSE_THRESHOLD = 0.60
TURN_CLEAR_THRESHOLD = 0.40
LANE_RESPONSE_THRESHOLD = 0.05
LANE_CLEAR_THRESHOLD = 0.02
RESPONSE_DURATION_S = 0.20
CLEAR_DURATION_S = 0.50
MIN_DURATION_S = 1.0
EVALUATION_GAP_S = 0.15


@dataclass(frozen=True)
class IntentConfig:
  mode: str = "shadow"
  min_speed: float = 2.0
  max_speed: float = 5.0  # Zero explicitly disables the upper speed gate.
  motion_duration_s: float = 0.5
  execution_timeout_s: float = 10.0
  response_timeout_s: float = 2.0
  turn_response_timeout_s: float = 2.0
  turn_execution_timeout_s: float = 12.0
  standstill_timeout_s: float = 2.0
  cooldown_s: float = 1.0
  pulse_interval_s: float = 6.0
  recovery_s: float = 1.0

  def __post_init__(self):
    if self.mode not in ("off", "shadow", "execute"):
      raise ValueError("TURBO_INTENT_MODE must be off, shadow or execute")
    times = (self.motion_duration_s, self.execution_timeout_s, self.response_timeout_s,
             self.turn_response_timeout_s, self.turn_execution_timeout_s, self.standstill_timeout_s,
             self.cooldown_s, self.pulse_interval_s, self.recovery_s)
    speeds = (self.min_speed, self.max_speed)
    if (not all(math.isfinite(v) and v > 0 for v in times) or
        not all(math.isfinite(v) and v >= 0 for v in speeds) or
        (self.max_speed != 0 and self.max_speed <= self.min_speed)):
      raise ValueError("invalid Turbo intent speed/time limits")

  def speed_allowed(self, speed: float) -> bool:
    return (math.isfinite(speed) and speed > 0 and speed >= self.min_speed and
            (self.max_speed == 0 or speed <= self.max_speed))


@dataclass(frozen=True)
class IntentHealth:
  session_id: str = ""
  link_fresh: bool = False
  lateral_active: bool = False
  vehicle_healthy: bool = False
  operator_fresh: bool = False
  operator_override: bool = False
  reverse: bool = False
  speed: float = 0.0
  standstill: bool = False


class TurboIntentManager:
  def __init__(self, config: IntentConfig | None = None, *, history_evaluations: int = 100):
    if not isinstance(history_evaluations, int) or history_evaluations < 1:
      raise ValueError("model desire history must contain at least one evaluation")
    self.config = config or IntentConfig()
    self.history_evaluations = history_evaluations
    self.epoch = uuid.uuid4().hex
    self.session_id = ""
    self.lateral_active = False
    self.health = IntentHealth()
    self.motion_since: float | None = None
    self.health_since: float | None = None
    self.model_since: float | None = None
    self.stopped_since: float | None = None
    self.last_now: float | None = None
    self.last_evaluation: float | None = None
    self.last_frame: int | None = None
    self.model_valid = False
    self.model_error = "model_unavailable"
    self.request: dict = {}
    self.receipt: dict = {}
    self.seen: dict[str, int] = {}
    self.status = "idle"
    self.reason = ""
    self.pulse_time = 0.0
    self.pulse_frame = 0
    # These belong to the model process, not the request/session/engagement epoch.
    self.last_pulse_time: float | None = None
    self.history_remaining = 0
    self.cooldown_until = 0.0
    self.terminal_time: float | None = None
    self.scores = (0.0, 0.0, 0.0, 0.0)  # turn L/R, lane change L/R
    self.baseline = self.scores
    self.recent_scores: deque = deque(maxlen=20)
    self.probability = self.opposite_probability = self.lane_change_probability = 0.0
    self.peak_probability = 0.0
    self.responded = False
    self.consumed_desire = log.Desire.none
    self.response_since: float | None = None
    self.clear_since: float | None = None
    self.opposite_since: float | None = None
    self.opposite_dominant = False

  @property
  def busy(self):
    return self.status in BUSY_STATUSES

  @property
  def desire(self):
    if self.status == "awaitingEvaluation" and self.config.mode == "execute":
      return MANEUVER_DESIRES.get((self.request.get("maneuver"), self.request.get("direction")), log.Desire.none)
    return log.Desire.none

  @property
  def lane_change_state(self):
    return (log.LaneChangeState.laneChangeStarting if self.status == "executing" and self.request.get("maneuver") == "laneChange"
            else log.LaneChangeState.off)

  @property
  def lane_change_direction(self):
    if self.lane_change_state == log.LaneChangeState.off:
      return log.LaneChangeDirection.none
    return log.LaneChangeDirection.left if self.request["direction"] == "left" else log.LaneChangeDirection.right

  def fault(self, now: float) -> str:
    if not self.health.link_fresh or not self.health.session_id:
      return "link_stale"
    if not self.health.vehicle_healthy:
      return "vehicle_unhealthy"
    if not self.health.operator_fresh:
      return "operator_stale"
    if self.model_error:
      return self.model_error
    if self.last_evaluation is None or not 0 <= now - self.last_evaluation <= FEEDBACK_TIMEOUT_S:
      return "model_stale"
    return ""

  def gate(self, now: float) -> str:
    h = self.health
    if self.config.mode == "off":
      return "disabled"
    if not h.link_fresh or not h.session_id:
      return "link_stale"
    if not h.lateral_active:
      return "not_engaged"
    if not h.vehicle_healthy:
      return "vehicle_unhealthy"
    if h.standstill:
      return "standstill"
    if not h.operator_fresh:
      return "operator_stale"
    if h.reverse:
      return "reverse"
    if h.operator_override:
      return "steering_override"
    if not self.config.speed_allowed(h.speed):
      return "speed_out_of_range"
    if self.motion_since is None or now - self.motion_since < self.config.motion_duration_s:
      return "motion_not_stable"
    if fault := self.fault(now):
      return fault
    if self.health_since is None or self.model_since is None or now - max(self.health_since, self.model_since) < self.config.recovery_s:
      return "health_recovering"
    return ""

  def availability_reason(self, now: float) -> str:
    if reason := self.gate(now):
      return reason
    if self.busy:
      return "busy"
    if now < self.cooldown_until:
      return "cooldown"
    if self.history_remaining:
      return "model_history_pending"
    return ""

  def _finish(self, status: str, reason: str, now: float):
    self.status, self.reason, self.terminal_time = status, reason, now
    self.cooldown_until = max(self.cooldown_until, now + self.config.cooldown_s)
    self.response_since = self.clear_since = self.opposite_since = None

  def _deadlines(self, now: float):
    if self.status != "executing":
      return
    elapsed = now - self.pulse_time
    turn = self.request.get("maneuver") == "turn"
    timeout = self.config.turn_execution_timeout_s if turn else self.config.execution_timeout_s
    response_timeout = self.config.turn_response_timeout_s if turn else self.config.response_timeout_s
    if elapsed >= timeout:
      self._finish("expired", "observation_timeout", now)
    elif self.stopped_since is not None and now - self.stopped_since >= self.config.standstill_timeout_s:
      self._finish("stopped", "standstill", now)
    elif not self.responded and elapsed >= response_timeout:
      self._finish("unconfirmed", "response_not_observed", now)

  def update(self, health: IntentHealth, request: dict | None, now: float) -> None:
    reset = health.session_id != self.session_id or health.lateral_active != self.lateral_active
    backwards = self.last_now is not None and now < self.last_now
    self.last_now, self.health = now, health
    if reset or backwards:
      self.epoch = uuid.uuid4().hex
      self.seen.clear()
      self.receipt = {}
      self.motion_since = None
      if self.status == "awaitingEvaluation" or (self.busy and not health.lateral_active) or backwards:
        self._finish("interrupted", "engagement_or_session_reset", now)
      # Never erase the consumed pulse/history guard on an engagement or LTE reset.
    self.session_id, self.lateral_active = health.session_id, health.lateral_active

    healthy = health.link_fresh and bool(health.session_id) and health.vehicle_healthy and health.operator_fresh
    self.health_since = (now if self.health_since is None else self.health_since) if healthy else None
    moving = health.vehicle_healthy and health.lateral_active and not health.reverse and not health.standstill and self.config.speed_allowed(health.speed)
    self.motion_since = (now if self.motion_since is None else self.motion_since) if moving else None
    stopped = health.vehicle_healthy and health.standstill and self.status == "executing"
    self.stopped_since = (now if self.stopped_since is None else self.stopped_since) if stopped else None

    if self.status == "awaitingEvaluation":
      if reason := self.gate(now) or self._context_error(self.request, now):
        self._finish("expired", reason, now)
    elif self.status == "executing":
      if fault := self.fault(now):
        self._finish("faulted", fault, now)
      else:
        self.reason = "operator_overriding" if health.operator_override else "standstill" if health.standstill else ""
        self._deadlines(now)

    if request and request.get("action") != "none":
      self._accept(request, now)

  def _context_error(self, request: dict, now: float) -> str:
    if request.get("protocolVersion") != PROTOCOL_VERSION:
      return "unsupported_protocol"
    if request.get("maneuver") not in ("laneChange", "turn"):
      return "invalid_maneuver"
    if request.get("direction") not in ("left", "right"):
      return "invalid_direction"
    if request.get("sessionId") != self.session_id or request.get("epoch") != self.epoch:
      return "stale_session"
    reference = request.get("baseFeedbackLogMonoTime", 0)
    if type(reference) is not int or reference <= 0 or not 0 <= now - reference / 1e9 <= CONTEXT_TIMEOUT_S:
      return "stale_context"
    return ""

  def _receipt(self, request: dict, result: str, *, matches: bool = False):
    self.receipt = {
      "protocolVersion": PROTOCOL_VERSION, "sessionId": self.session_id, "epoch": self.epoch,
      "operatorId": request["operatorId"], "requestId": request["requestId"],
      "maneuver": request.get("maneuver") if request.get("maneuver") in ("laneChange", "turn") else "none",
      "direction": request.get("direction") if request.get("direction") in ("left", "right") else "none",
      "action": request["action"], "result": result, "status": self.status if matches else "",
      "pulseMonoTime": int(self.pulse_time * 1e9) if matches else 0,
    }

  def _accept(self, request: dict, now: float) -> None:
    operator, request_id = request.get("operatorId", ""), request.get("requestId", 0)
    if request.get("sessionId") != self.session_id or request.get("epoch") != self.epoch:
      return
    if not isinstance(operator, str) or not operator or type(request_id) is not int or request_id <= 0:
      return
    if operator not in self.seen and len(self.seen) >= 128:
      return
    previous_id = self.seen.get(operator, 0)
    matches = all(request.get(k) == self.request.get(k) for k in ("operatorId", "requestId", "maneuver", "direction"))
    action = request.get("action")
    if action == "cancel":
      if reason := self._context_error(request, now):
        self._receipt(request, reason)
        return
      self.seen[operator] = max(previous_id, request_id)
      if matches and self.pulse_time:
        result = "already_consumed"
      elif matches and self.status == "awaitingEvaluation":
        self._finish("canceled", "operator_cancel", now)
        result = "canceled_before_consumption"
      else:
        # Retired IDs cannot execute in the future, but may have run in the past.
        result = "retired" if request_id <= previous_id else "canceled_before_consumption"
        if request_id > previous_id and not self.busy:
          self.cooldown_until = max(self.cooldown_until, now + self.config.cooldown_s)
      self._receipt(request, result, matches=matches)
      return
    if action != "request":
      return
    if request_id <= previous_id:
      result = "already_consumed" if matches and self.pulse_time else "accepted" if matches and self.busy else "retired"
      self._receipt(request, result, matches=matches)
      return
    self.seen[operator] = request_id
    if self.busy or now < self.cooldown_until or self.history_remaining:
      self._receipt(request, "rejected_busy")
      return
    self.request = dict(request)
    self.pulse_time, self.pulse_frame, self.consumed_desire = 0.0, 0, log.Desire.none
    self.responded, self.opposite_dominant, self.peak_probability = False, False, 0.0
    self.response_since = self.clear_since = self.opposite_since = self.stopped_since = self.terminal_time = None
    recent = [scores for t, scores in self.recent_scores if 0 <= now - t <= 1.0]
    self.baseline = tuple(sum(row[i] for row in recent) / len(recent) for i in range(4)) if recent else self.scores
    reason = self._context_error(request, now) or self.gate(now)
    if not request.get("ready") or request.get("reverse"):
      reason = "operator_not_ready"
    if reason:
      self._finish("rejected", reason, now)
    elif self.config.mode == "shadow":
      self._finish("shadow", "would_execute", now)
    else:
      self.status, self.reason = "awaitingEvaluation", ""
    self._receipt(request, "accepted" if self.busy else "rejected", matches=True)

  def evaluated(self, consumed_desire: int, probability: float, frame_id: int, now: float,
                *, opposite_probability: float = 0.0, lane_change_probability: float | None = None,
                scores: tuple[float, float, float, float] | None = None, model_valid: bool = True) -> None:
    # The policy has already run. Even an invalid/duplicate output cannot undo
    # consumption or allow its explicit input-history guard to be forgotten.
    consumed = self.status == "awaitingEvaluation" and consumed_desire == self.desire and consumed_desire != log.Desire.none
    if consumed:
      self.status, self.reason = "executing", ""
      self.pulse_time, self.pulse_frame, self.last_pulse_time = now, frame_id, now
      self.consumed_desire = consumed_desire
      self.history_remaining = self.history_evaluations
      self.cooldown_until = max(self.cooldown_until, now + self.config.pulse_interval_s)
    contiguous = (self.last_evaluation is not None and 0 < now - self.last_evaluation <= EVALUATION_GAP_S and
                  self.last_frame is not None and frame_id > self.last_frame)
    if not contiguous:
      self.response_since = self.clear_since = self.opposite_since = self.model_since = None
    if self.last_evaluation is not None and (now <= self.last_evaluation or frame_id <= self.last_frame):
      self.model_error, self.model_valid = "invalid_model_evaluation", False
      if now > self.last_evaluation and frame_id < self.last_frame:
        # A camera restart can reset frame IDs. Fault this sample, then require
        # fresh advancing evaluations/recovery instead of waiting for IDs to catch up.
        self.last_evaluation, self.last_frame = now, frame_id
      if self.busy:
        self._finish("faulted", self.model_error, now)
      return
    self.last_evaluation, self.last_frame = now, frame_id
    if not consumed:
      self.history_remaining = max(0, self.history_remaining - 1)

    lane_change_probability = probability if lane_change_probability is None else lane_change_probability
    if scores is None:
      pair = (probability, opposite_probability) if self.request.get("direction") != "right" else (opposite_probability, probability)
      scores = (*pair, lane_change_probability, 0.0) if self.request.get("maneuver") == "turn" else (0.0, 0.0, *pair)
    valid = model_valid and len(scores) == 4 and all(math.isfinite(p) and 0 <= p <= 1.00001 for p in scores)
    valid &= all(math.isfinite(p) and 0 <= p <= 1.00001 for p in (probability, opposite_probability, lane_change_probability))
    self.model_valid, self.model_error = bool(valid), "" if valid else "invalid_model_output"
    if not valid:
      self.model_since = None
      if self.busy:
        self._finish("faulted", self.model_error, now)
      return  # Keep finite last-good scores, explicitly marked invalid and timestamped.
    self.model_since = now if self.model_since is None else self.model_since
    self.scores = tuple(scores)
    self.recent_scores.append((now, self.scores))
    self.probability, self.opposite_probability, self.lane_change_probability = probability, opposite_probability, lane_change_probability
    self._deadlines(now)
    if self.status != "executing":
      return
    self.peak_probability = max(self.peak_probability, probability)
    turn = self.request.get("maneuver") == "turn"
    threshold, margin = (TURN_RESPONSE_THRESHOLD, 0.20) if turn else (LANE_RESPONSE_THRESHOLD, 0.02)
    strong = probability >= threshold and probability - opposite_probability >= margin
    self.response_since = (now if self.response_since is None else self.response_since) if strong else None
    if self.response_since is not None and now - self.response_since >= RESPONSE_DURATION_S:
      self.responded = True
    opposite = opposite_probability >= threshold and opposite_probability - probability >= margin
    self.opposite_since = (now if self.opposite_since is None else self.opposite_since) if opposite else None
    self.opposite_dominant |= self.opposite_since is not None and now - self.opposite_since >= CLEAR_DURATION_S
    clear = (max(probability, opposite_probability) < TURN_CLEAR_THRESHOLD) if turn else lane_change_probability < LANE_CLEAR_THRESHOLD
    self.clear_since = (now if self.clear_since is None else self.clear_since) if self.responded and clear else None
    if self.clear_since is not None and now - self.pulse_time >= MIN_DURATION_S and now - self.clear_since >= CLEAR_DURATION_S:
      self._finish("responseCleared", "model_response_cleared", now)

  def snapshot(self, now: float) -> dict:
    direction, maneuver = self.request.get("direction", "none"), self.request.get("maneuver", "none")
    phase = ("awaitingEvaluation" if self.status == "awaitingEvaluation" else "observing" if self.status == "executing" else
             "cooldown" if now < self.cooldown_until or self.history_remaining else "idle")
    availability = self.availability_reason(now)

    def dwell(since):
      return max(0.0, now - since) if since is not None else 0.0

    return {
      "protocolVersion": PROTOCOL_VERSION, "sessionId": self.session_id, "epoch": self.epoch,
      "operatorId": self.request.get("operatorId", ""), "requestId": self.request.get("requestId", 0),
      "direction": direction if direction in ("left", "right") else "none", "status": self.status,
      "maneuver": maneuver if maneuver in ("laneChange", "turn") else "none",
      "reason": self.reason or availability, "mode": self.config.mode, "available": not availability,
      "minSpeed": self.config.min_speed, "maxSpeed": self.config.max_speed,
      "pulseFrameId": self.pulse_frame, "pulseMonoTime": int(self.pulse_time * 1e9),
      "laneChangeProbability": self.lane_change_probability, "operatorOverride": self.health.operator_override,
      "modelResponseProbability": self.probability, "oppositeTurnProbability": self.opposite_probability,
      "consumedDesire": self.consumed_desire, "phase": phase,
      "outcome": self.status if not self.busy and self.status != "idle" else "",
      "availabilityReason": availability, "faultReason": self.fault(now), "responseObserved": self.responded,
      "modelSampleMonoTime": int((self.last_evaluation or 0) * 1e9), "modelSampleValid": self.model_valid,
      "turnLeftProbability": self.scores[0], "turnRightProbability": self.scores[1],
      "laneChangeLeftProbability": self.scores[2], "laneChangeRightProbability": self.scores[3],
      "elapsedS": max(0.0, now - self.pulse_time) if self.pulse_time else 0.0,
      "cooldownRemainingS": max(0.0, self.cooldown_until - now), "historyRemaining": self.history_remaining,
      "outcomeAgeS": dwell(self.terminal_time), "responseDwellS": dwell(self.response_since), "clearDwellS": dwell(self.clear_since),
      "receipt": self.receipt, "baselineProbabilities": self.baseline, "peakResponseProbability": self.peak_probability,
      "oppositeDominant": bool(self.opposite_dominant),
    }
