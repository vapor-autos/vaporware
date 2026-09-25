"""UGV-local one-shot maneuver requests. No sockets, CAN or tensor execution."""
from dataclasses import dataclass
import math
import uuid

from openpilot.cereal import log

PROTOCOL_VERSION = 2
CONTEXT_TIMEOUT_S = 0.35
FEEDBACK_TIMEOUT_S = 0.35
REQUEST_SERVICE = "turboIntentRequest"
STATE_SERVICE = "turboIntentState"
LINK_SERVICE = "turboIntentLinkState"
BUSY_STATUSES = frozenset(("awaitingEvaluation", "executing", "timedOut", "noModelResponse"))
MANEUVER_DESIRES = {
  ("laneChange", "left"): log.Desire.laneChangeLeft, ("laneChange", "right"): log.Desire.laneChangeRight,
  ("turn", "left"): log.Desire.turnLeft, ("turn", "right"): log.Desire.turnRight,
}
# Experimental turn-response bookkeeping, not physical maneuver completion.
TURN_RESPONSE_THRESHOLD = 0.1
TURN_CLEAR_THRESHOLD = 0.02
TURN_CLEAR_DURATION_S = 0.5
TURN_MIN_DURATION_S = 1.0


@dataclass(frozen=True)
class IntentConfig:
  mode: str = "shadow"
  min_speed: float = 2.0
  max_speed: float = 5.0  # Zero explicitly disables the upper speed gate.
  motion_duration_s: float = 0.5
  execution_timeout_s: float = 10.0
  response_timeout_s: float = 2.0
  turn_response_timeout_s: float = 5.0
  turn_execution_timeout_s: float = 20.0

  def __post_init__(self):
    if self.mode not in ("off", "shadow", "execute"):
      raise ValueError("TURBO_INTENT_MODE must be off, shadow or execute")
    times = (self.motion_duration_s, self.execution_timeout_s, self.response_timeout_s,
             self.turn_response_timeout_s, self.turn_execution_timeout_s)
    speeds = (self.min_speed, self.max_speed)
    if (not all(math.isfinite(v) and v > 0 for v in times) or
        not all(math.isfinite(v) and v >= 0 for v in speeds) or
        (self.max_speed != 0 and self.max_speed <= self.min_speed)):
      raise ValueError("invalid Turbo intent speed/time limits")

  def speed_allowed(self, speed: float) -> bool:
    # Disabling numeric gates does not permit stopped/reverse/invalid motion.
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
  def __init__(self, config: IntentConfig | None = None):
    self.config = config or IntentConfig()
    self.epoch = uuid.uuid4().hex
    self.session_id = ""
    self.lateral_active = False
    self.health = IntentHealth()
    self.motion_since: float | None = None
    self.last_now: float | None = None
    self.request: dict = {}
    self.seen: dict[str, int] = {}  # Per-operator high-water marks, retained for the epoch.
    self.status = "idle"
    self.reason = ""
    self.pulse_time = 0.0
    self.pulse_frame = 0
    self.probability = 0.0
    self.responded = False
    self.opposite_probability = 0.0
    self.lane_change_probability = 0.0
    self.consumed_desire = log.Desire.none
    self.clear_since: float | None = None
    self.last_evaluation: float | None = None

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
    return ""

  def update(self, health: IntentHealth, request: dict | None, now: float) -> None:
    reset = health.session_id != self.session_id or health.lateral_active != self.lateral_active
    reset |= self.last_now is not None and now < self.last_now
    self.last_now = now
    self.health = health
    if reset:
      self.epoch = uuid.uuid4().hex
      self.seen.clear()
      self.motion_since = None
      if self.status == "awaitingEvaluation" or (self.busy and not health.lateral_active):
        self.status, self.reason = "interrupted", "engagement_or_session_reset"
      elif not self.busy:
        # An old terminal result is not the status of a new engagement/session.
        self.request = {}
        self.status, self.reason = "idle", ""
        self.pulse_time, self.pulse_frame, self.probability, self.responded = 0.0, 0, 0.0, False
        self.opposite_probability, self.lane_change_probability, self.consumed_desire, self.clear_since = 0.0, 0.0, log.Desire.none, None
        self.last_evaluation = None
    self.session_id = health.session_id
    self.lateral_active = health.lateral_active

    moving = (health.vehicle_healthy and health.lateral_active and not health.reverse and not health.standstill and
              self.config.speed_allowed(health.speed))
    if not moving:
      self.motion_since = None
    elif self.motion_since is None:
      self.motion_since = now

    if self.status == "awaitingEvaluation":
      reason = self.gate(now) or self._context_error(self.request, now)
      if reason:
        self.status, self.reason = "expired", reason
    elif self.status == "executing":
      # A lost link/override does not undo a model pulse. Keep the maneuver visible.
      self.reason = ("operator_link_lost" if not health.link_fresh else "vehicle_unhealthy" if not health.vehicle_healthy else
                     "operator_overriding" if health.operator_override else "standstill" if health.standstill else "")
      timeout = self.config.turn_execution_timeout_s if self.request.get("maneuver") == "turn" else self.config.execution_timeout_s
      if now - self.pulse_time > timeout:
        self.status, self.reason = "timedOut", "takeover_required"

    if request and request.get("action") != "none":
      self._accept(request, now)

  def _context_error(self, request: dict, now: float) -> str:
    if request.get("protocolVersion") != PROTOCOL_VERSION:
      return "unsupported_protocol"
    if request.get("maneuver") not in ("laneChange", "turn"):
      return "invalid_maneuver"
    if request.get("sessionId") != self.session_id or request.get("epoch") != self.epoch:
      return "stale_session"
    reference = request.get("baseFeedbackLogMonoTime", 0)
    if not isinstance(reference, int) or reference <= 0 or not 0 <= now - reference / 1e9 <= CONTEXT_TIMEOUT_S:
      return "stale_context"
    return ""

  def _accept(self, request: dict, now: float) -> None:
    key = (request.get("operatorId", ""), request.get("requestId", 0))
    # Wrong-session traffic must not change the current transaction or its display.
    if request.get("sessionId") != self.session_id or request.get("epoch") != self.epoch:
      return
    if not key[0] or not isinstance(key[1], int) or key[1] <= 0:
      return
    if key[0] not in self.seen and len(self.seen) >= 128:
      return  # Bound memory without evicting an identity and permitting a replay.
    previous_id = self.seen.get(key[0], 0)
    if request.get("action") == "cancel":
      if key == (self.request.get("operatorId"), self.request.get("requestId")) and not self._context_error(request, now):
        if self.status == "awaitingEvaluation":
          self.status, self.reason = "canceled", "operator_cancel"
        elif self.busy:
          self.reason = "already_started_use_l2"
      # Tombstone even a cancel that overtook its request in a local latest-state queue.
      self.seen[key[0]] = max(previous_id, key[1])
      return
    if request.get("action") != "request" or key[1] <= previous_id:
      return
    self.seen[key[0]] = key[1]
    if self.busy:
      return  # Preserve authoritative active request; never queue the new one.
    self.request = dict(request)
    self.pulse_time, self.pulse_frame, self.probability, self.responded = 0.0, 0, 0.0, False
    self.opposite_probability, self.lane_change_probability, self.consumed_desire, self.clear_since = 0.0, 0.0, log.Desire.none, None
    self.last_evaluation = None
    reason = self._context_error(request, now) or self.gate(now)
    if request.get("direction") not in ("left", "right"):
      reason = "invalid_direction"
    if not request.get("ready") or request.get("reverse"):
      reason = "operator_not_ready"
    if reason:
      self.status, self.reason = "rejected", reason
    elif self.config.mode == "shadow":
      self.status, self.reason = "shadow", "would_execute"
    else:
      self.status, self.reason = "awaitingEvaluation", ""

  def evaluated(self, consumed_desire: int, probability: float, frame_id: int, now: float,
                *, opposite_probability: float = 0.0, lane_change_probability: float | None = None) -> None:
    if self.status == "awaitingEvaluation" and consumed_desire == self.desire and consumed_desire != log.Desire.none:
      self.status, self.reason = "executing", ""
      self.pulse_time, self.pulse_frame = now, frame_id
      self.consumed_desire = consumed_desire
    if self.status != "executing":
      return
    if self.last_evaluation is None or not 0 <= now - self.last_evaluation <= 0.15:
      self.clear_since = None
    self.last_evaluation = now
    if not all(math.isfinite(p) and 0 <= p <= 1.00001 for p in (probability, opposite_probability)):
      self.status, self.reason = "noModelResponse", "invalid_model_output_takeover_required"
      return
    self.probability = probability
    self.opposite_probability = opposite_probability
    self.lane_change_probability = probability if lane_change_probability is None else lane_change_probability
    elapsed = now - self.pulse_time
    if self.request.get("maneuver") == "turn":
      # Never interpret lane-change output (or an opposite turn) as success.
      if opposite_probability >= TURN_RESPONSE_THRESHOLD and opposite_probability >= probability:
        self.status, self.reason = "noModelResponse", "opposite_turn_response_takeover_required"
        return
      self.responded |= probability >= TURN_RESPONSE_THRESHOLD
      if self.responded and probability < TURN_CLEAR_THRESHOLD and opposite_probability < TURN_CLEAR_THRESHOLD:
        self.clear_since = now if self.clear_since is None else self.clear_since
        if elapsed >= TURN_MIN_DURATION_S and now - self.clear_since >= TURN_CLEAR_DURATION_S:
          self.status, self.reason = "completed", "model_turn_response_cleared"
      else:
        self.clear_since = None
      if not self.responded and elapsed >= self.config.turn_response_timeout_s:
        self.status, self.reason = "noModelResponse", "takeover_required"
      return
    self.responded |= probability >= 0.02
    if self.responded and elapsed >= 0.5 and probability < 0.02:
      self.status, self.reason = "completed", "model_reports_complete"
    elif not self.responded and elapsed >= self.config.response_timeout_s:
      self.status, self.reason = "noModelResponse", "takeover_required"

  def snapshot(self, now: float) -> dict:
    direction = self.request.get("direction", "none")
    maneuver = self.request.get("maneuver", "none")
    return {
      "protocolVersion": PROTOCOL_VERSION, "sessionId": self.session_id, "epoch": self.epoch,
      "operatorId": self.request.get("operatorId", ""), "requestId": self.request.get("requestId", 0),
      "direction": direction if direction in ("left", "right") else "none", "status": self.status,
      "maneuver": maneuver if maneuver in ("laneChange", "turn") else "none",
      "reason": self.reason or self.gate(now), "mode": self.config.mode,
      "available": not self.busy and not self.gate(now),
      "minSpeed": self.config.min_speed, "maxSpeed": self.config.max_speed,
      "pulseFrameId": self.pulse_frame, "pulseMonoTime": int(self.pulse_time * 1e9),
      "laneChangeProbability": self.lane_change_probability, "operatorOverride": self.health.operator_override,
      "modelResponseProbability": self.probability, "oppositeTurnProbability": self.opposite_probability,
      "consumedDesire": self.consumed_desire,
    }
