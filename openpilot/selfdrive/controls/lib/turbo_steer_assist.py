from __future__ import annotations

import time


STEERING_TARGET_MAX_ANGLE_DEG = 180.0
DEFAULT_MAX_NUDGE_ANGLE_DEG = 3.0
DEFAULT_NUDGE_GAIN_DEG = 30.0
DEFAULT_NUDGE_DEADBAND_DEG = 0.0
DEFAULT_STALE_TIMEOUT_S = 0.25


def clip(value: float, lo: float, hi: float) -> float:
  return min(max(value, lo), hi)


def steering_angle_to_g29_target(steering_angle_deg: float) -> float:
  return clip(-steering_angle_deg / STEERING_TARGET_MAX_ANGLE_DEG, -1.0, 1.0)


def compute_nudge_angle_deg(
  wheel_steering: float,
  target_steering: float,
  gain_deg: float = DEFAULT_NUDGE_GAIN_DEG,
  max_nudge_angle_deg: float = DEFAULT_MAX_NUDGE_ANGLE_DEG,
  deadband_deg: float = DEFAULT_NUDGE_DEADBAND_DEG,
) -> float:
  wheel_delta = clip(wheel_steering, -1.0, 1.0) - clip(target_steering, -1.0, 1.0)
  nudge_angle_deg = -wheel_delta * max(0.0, gain_deg)
  if abs(nudge_angle_deg) < max(0.0, deadband_deg):
    return 0.0
  max_nudge = max(0.0, max_nudge_angle_deg)
  return clip(nudge_angle_deg, -max_nudge, max_nudge)


class TurboSteerAssistSource:
  def __init__(
    self,
    sm,
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
    max_nudge_angle_deg: float = DEFAULT_MAX_NUDGE_ANGLE_DEG,
  ):
    self.sm = sm
    self.stale_timeout_s = stale_timeout_s
    self.max_nudge_angle_deg = max_nudge_angle_deg
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
    max_nudge = max(0.0, self.max_nudge_angle_deg)
    self.last_nudge_angle_deg = clip(self.last_raw_nudge_angle_deg, -max_nudge, max_nudge)
    self.last_status = "active"
    return self.last_nudge_angle_deg, self.last_status

  def _age(self, now: float) -> float | None:
    if not self.sm.seen["turboSteerAssist"]:
      return None
    return now - self.sm.recv_time["turboSteerAssist"]
