from __future__ import annotations

from dataclasses import dataclass
import math
import time


STEERING_TARGET_MAX_ANGLE_DEG = 180.0
DEFAULT_STALE_TIMEOUT_S = 0.25
DEFAULT_CONTEXT_TIMEOUT_S = 0.35
DEFAULT_TARGET_MISMATCH_DEG = 15.0


def clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def steering_angle_to_g29_target(steering_angle_deg: float) -> float:
  return clip(-steering_angle_deg / STEERING_TARGET_MAX_ANGLE_DEG, -1.0, 1.0)


def g29_steering_to_angle_deg(steering: float) -> float:
  return -clip(steering, -1.0, 1.0) * STEERING_TARGET_MAX_ANGLE_DEG


def clip_steering_angle_deg(steering_angle_deg: float) -> float:
  return clip(steering_angle_deg, -STEERING_TARGET_MAX_ANGLE_DEG, STEERING_TARGET_MAX_ANGLE_DEG)


@dataclass(frozen=True)
class TurboSteerAssistDecision:
  target_angle_deg: float | None
  status: str
  receive_age_s: float | None
  context_age_s: float | None
  base_model_delta_deg: float | None
  sequence: int
  base_model_log_mono_time: int


class TurboSteerAssistSource:
  def __init__(
    self,
    sm,
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
    context_timeout_s: float = DEFAULT_CONTEXT_TIMEOUT_S,
    target_mismatch_deg: float = DEFAULT_TARGET_MISMATCH_DEG,
  ):
    self.sm = sm
    self.stale_timeout_s = stale_timeout_s
    self.context_timeout_s = context_timeout_s
    self.target_mismatch_deg = target_mismatch_deg
    self._current_key: tuple[int, int] | None = None
    self._current_key_status = "unseen"
    self._last_accepted_base_model_log_mono_time = 0
    self._last_accepted_sequence = 0
    self._override_session_active = False

  def update(self, lat_active: bool, model_angle_deg: float, now: float | None = None) -> TurboSteerAssistDecision:
    now = time.monotonic() if now is None else now
    receive_age_s = self._age(now)
    context_age_s: float | None = None
    base_model_delta_deg: float | None = None
    sequence = 0
    base_model_log_mono_time = 0

    def finish(status: str, target_angle_deg: float | None = None) -> TurboSteerAssistDecision:
      self._override_session_active = status == "active"
      return TurboSteerAssistDecision(
        target_angle_deg=target_angle_deg,
        status=status,
        receive_age_s=receive_age_s,
        context_age_s=context_age_s,
        base_model_delta_deg=base_model_delta_deg,
        sequence=sequence,
        base_model_log_mono_time=base_model_log_mono_time,
      )

    if not lat_active:
      return finish("lat_inactive")
    if not self.sm.seen["turboSteerAssist"]:
      return finish("unseen")
    if not self.sm.valid["turboSteerAssist"]:
      return finish("invalid")
    if receive_age_s is None or receive_age_s > self.stale_timeout_s:
      return finish("stale")

    assist = self.sm["turboSteerAssist"]
    if not assist.active:
      return finish("inactive")

    sequence = int(assist.sequence)
    base_model_log_mono_time = int(assist.baseModelLogMonoTime)
    key = (base_model_log_mono_time, sequence)
    if key != self._current_key:
      self._current_key = key
      if base_model_log_mono_time == 0 or sequence == 0:
        self._current_key_status = "missing_target_context"
      elif base_model_log_mono_time < self._last_accepted_base_model_log_mono_time or (
        base_model_log_mono_time == self._last_accepted_base_model_log_mono_time and sequence <= self._last_accepted_sequence
      ):
        self._current_key_status = "out_of_order"
      else:
        self._current_key_status = "accepted"
        self._last_accepted_base_model_log_mono_time = base_model_log_mono_time
        self._last_accepted_sequence = sequence

    if self._current_key_status != "accepted":
      return finish(self._current_key_status)

    context_age_s = now - base_model_log_mono_time / 1e9
    if context_age_s < 0.0:
      return finish("future_target_context")
    if context_age_s > self.context_timeout_s:
      return finish("stale_target_context")

    base_model_angle_deg = float(assist.baseModelSteeringAngleDeg)
    base_model_delta_deg = float(model_angle_deg) - base_model_angle_deg
    if not math.isfinite(base_model_delta_deg):
      return finish("target_mismatch")
    if not self._override_session_active and abs(base_model_delta_deg) > self.target_mismatch_deg:
      return finish("target_mismatch")

    requested_target_angle_deg = float(assist.requestedSteeringAngleDeg)
    if not math.isfinite(requested_target_angle_deg):
      return finish("invalid_target")

    return finish("active", clip_steering_angle_deg(requested_target_angle_deg))

  def _age(self, now: float) -> float | None:
    if not self.sm.seen["turboSteerAssist"]:
      return None
    return now - self.sm.recv_time["turboSteerAssist"]
