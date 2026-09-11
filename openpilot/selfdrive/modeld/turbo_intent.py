import os

from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.lib.turbo_intent import (
  IntentConfig, IntentHealth, TurboIntentManager, REQUEST_SERVICE, STATE_SERVICE, LINK_SERVICE, FEEDBACK_TIMEOUT_S,
)

INTENT_SUBSCRIPTIONS = [REQUEST_SERVICE, LINK_SERVICE, "g29", "turboSteerAssist", "turboSteerAssistState", "selfdriveState"]


class TurboIntentRuntime:
  def __init__(self, pm, config: IntentConfig | None = None):
    self.manager = TurboIntentManager(config or IntentConfig(
      mode=os.getenv("TURBO_INTENT_MODE", "shadow"),
      min_speed=float(os.getenv("TURBO_INTENT_MIN_SPEED", "2.0")),
      max_speed=float(os.getenv("TURBO_INTENT_MAX_SPEED", "5.0")),
    ))
    self.pm = pm
    self.last_publish = -float("inf")
    self.last_signature = None

  def before_inference(self, sm, now: float, maneuver_mode: bool = False):
    def fresh(service, timeout=0.25):
      return sm.seen[service] and sm.valid[service] and 0 <= now - sm.recv_time[service] <= timeout

    cs, cc = sm["carState"], sm["carControl"]
    link = sm[LINK_SERVICE]
    assist = sm["turboSteerAssist"]
    assist_context_fresh = 0 < assist.baseModelLogMonoTime <= now * 1e9 and now - assist.baseModelLogMonoTime / 1e9 <= 0.35
    health = IntentHealth(
      session_id=str(link.sessionId) if sm.seen[LINK_SERVICE] else "",
      link_fresh=fresh(LINK_SERVICE, FEEDBACK_TIMEOUT_S) and bool(link.connected),
      lateral_active=sm.seen["carControl"] and cc.latActive,
      vehicle_healthy=(fresh("carControl") and fresh("carState") and cs.canValid and not cs.standstill and
                       not cs.steerFaultTemporary and not cs.steerFaultPermanent and
                       not cs.leftBlindspot and not cs.rightBlindspot and
                       fresh("selfdriveState") and str(sm["selfdriveState"].state) == "enabled" and
                       fresh("liveCalibration", 1.0) and sm["liveCalibration"].calStatus == log.LiveCalibrationData.Status.calibrated and
                       not maneuver_mode),
      operator_fresh=(fresh("g29") and fresh("turboSteerAssist") and assist_context_fresh and fresh("turboSteerAssistState")),
      operator_override=bool(assist.active or sm["turboSteerAssistState"].applied),
      reverse=float(sm["g29"].reverse) > -0.9,
      speed=float(cs.vEgo),
    )
    request = sm[REQUEST_SERVICE].to_dict() if sm.updated[REQUEST_SERVICE] and sm.valid[REQUEST_SERVICE] else None
    self.manager.update(health, request, now)
    self.publish(now)
    return self.manager.desire

  def after_inference(self, desire, probability: float, frame_id: int, now: float):
    self.manager.evaluated(desire, probability, frame_id, now)
    self.publish(now)

  def publish(self, now: float):
    state = self.manager.snapshot(now)
    signature = (state["epoch"], state["requestId"], state["status"], state["reason"])
    if now - self.last_publish >= 0.1 or signature != self.last_signature:
      msg = messaging.new_message(STATE_SERVICE, valid=True)
      msg.turboIntentState = state
      self.pm.send(STATE_SERVICE, msg)
      self.last_publish, self.last_signature = now, signature
