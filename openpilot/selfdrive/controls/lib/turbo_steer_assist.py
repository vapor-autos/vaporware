from __future__ import annotations

import time


STEERING_TARGET_MAX_ANGLE_DEG = 180.0
DEFAULT_INNER_DEADBAND_DEG = 1.0
DEFAULT_FULL_ASSIST_ERROR_DEG = 3.0
DEFAULT_STALE_TIMEOUT_S = 0.25


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
  ):
    self.sm = sm
    self.stale_timeout_s = stale_timeout_s
    self.last_age_s: float | None = None
    self.last_raw_nudge_angle_deg = 0.0
    self.last_nudge_angle_deg = 0.0
    self.last_status = "unseen"

  def update(self, lat_active: bool, now: float | None = None) -> tuple[float, str]:
    now = time.monotonic() if now is None else now
    self.last_age_s = self._age(now)
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

    self.last_raw_nudge_angle_deg = float(assist.nudgeAngleDeg)
    self.last_nudge_angle_deg = self.last_raw_nudge_angle_deg
    self.last_status = "active"
    return self.last_nudge_angle_deg, self.last_status

  def _age(self, now: float) -> float | None:
    if not self.sm.seen["turboSteerAssist"]:
      return None
    return now - self.sm.recv_time["turboSteerAssist"]
