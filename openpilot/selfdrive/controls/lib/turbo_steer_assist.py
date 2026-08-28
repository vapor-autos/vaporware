from __future__ import annotations

import math
import time


STEERING_TARGET_MAX_ANGLE_DEG = 180.0
DEFAULT_INNER_DEADBAND_DEG = 4.0
DEFAULT_FULL_ASSIST_ERROR_DEG = 8.0
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


def smoothstep(value: float) -> float:
  x = clip(value, 0.0, 1.0)
  return x * x * (3.0 - 2.0 * x)


def compute_nudge_angle_deg(
  wheel_steering: float,
  target_steering_angle_deg: float,
  inner_deadband_deg: float = DEFAULT_INNER_DEADBAND_DEG,
  full_assist_error_deg: float = DEFAULT_FULL_ASSIST_ERROR_DEG,
) -> float:
  wheel_angle_deg = g29_steering_to_angle_deg(wheel_steering)
  angle_error_deg = wheel_angle_deg - target_steering_angle_deg
  abs_error_deg = abs(angle_error_deg)
  inner = max(0.0, inner_deadband_deg)
  outer = max(inner, full_assist_error_deg)

  if abs_error_deg <= inner:
    return 0.0
  if abs_error_deg >= outer:
    return angle_error_deg
  if outer == inner:
    return angle_error_deg

  return angle_error_deg * smoothstep((abs_error_deg - inner) / (outer - inner))


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
    self.last_age_s: float | None = None
    self.last_context_age_s: float | None = None
    self.last_target_delta_deg: float | None = None
    self.last_sequence = 0
    self.last_base_target_log_mono_time = 0
    self.last_raw_nudge_angle_deg = 0.0
    self.last_nudge_angle_deg = 0.0
    self.last_status = "unseen"
    self._current_key: tuple[int, int] | None = None
    self._current_key_status = "unseen"
    self._last_accepted_base_target_log_mono_time = 0
    self._last_accepted_sequence = 0

  def update(self, lat_active: bool, model_angle_deg: float, now: float | None = None) -> tuple[float, str]:
    now = time.monotonic() if now is None else now
    self.last_age_s = self._age(now)
    self.last_context_age_s = None
    self.last_target_delta_deg = None
    self.last_raw_nudge_angle_deg = 0.0
    self.last_nudge_angle_deg = 0.0

    if not lat_active:
      self.last_status = "lat_inactive"
      return 0.0, self.last_status
    if not self.sm.seen["turboSteerAssist"]:
      self.last_status = "unseen"
      return 0.0, self.last_status
    if not self.sm.valid["turboSteerAssist"]:
      self.last_status = "invalid"
      return 0.0, self.last_status
    if self.last_age_s is None or self.last_age_s > self.stale_timeout_s:
      self.last_status = "stale"
      return 0.0, self.last_status

    assist = self.sm["turboSteerAssist"]
    if not assist.active:
      self.last_status = "inactive"
      return 0.0, self.last_status

    self.last_sequence = int(assist.sequence)
    self.last_base_target_log_mono_time = int(assist.baseTargetLogMonoTime)
    key = (self.last_base_target_log_mono_time, self.last_sequence)
    if key != self._current_key:
      self._current_key = key
      if self.last_base_target_log_mono_time == 0 or self.last_sequence == 0:
        self._current_key_status = "missing_target_context"
      elif self.last_base_target_log_mono_time < self._last_accepted_base_target_log_mono_time or (
        self.last_base_target_log_mono_time == self._last_accepted_base_target_log_mono_time and self.last_sequence <= self._last_accepted_sequence
      ):
        self._current_key_status = "out_of_order"
      else:
        self._current_key_status = "accepted"
        self._last_accepted_base_target_log_mono_time = self.last_base_target_log_mono_time
        self._last_accepted_sequence = self.last_sequence

    if self._current_key_status != "accepted":
      self.last_status = self._current_key_status
      return 0.0, self.last_status

    self.last_context_age_s = now - self.last_base_target_log_mono_time / 1e9
    if self.last_context_age_s < 0.0:
      self.last_status = "future_target_context"
      return 0.0, self.last_status
    if self.last_context_age_s > self.context_timeout_s:
      self.last_status = "stale_target_context"
      return 0.0, self.last_status

    target_angle_deg = float(assist.targetSteeringAngleDeg)
    self.last_target_delta_deg = float(model_angle_deg) - target_angle_deg
    if not math.isfinite(self.last_target_delta_deg) or abs(self.last_target_delta_deg) > self.target_mismatch_deg:
      self.last_status = "target_mismatch"
      return 0.0, self.last_status

    self.last_raw_nudge_angle_deg = float(assist.nudgeAngleDeg)
    if not math.isfinite(self.last_raw_nudge_angle_deg):
      self.last_raw_nudge_angle_deg = 0.0
      self.last_status = "invalid_nudge"
      return 0.0, self.last_status

    self.last_nudge_angle_deg = self.last_raw_nudge_angle_deg
    self.last_status = "active"
    return self.last_nudge_angle_deg, self.last_status

  def _age(self, now: float) -> float | None:
    if not self.sm.seen["turboSteerAssist"]:
      return None
    return now - self.sm.recv_time["turboSteerAssist"]
