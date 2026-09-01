from __future__ import annotations

from dataclasses import dataclass
import math

from openpilot.selfdrive.controls.lib.turbo_steer_assist import (
  DEFAULT_FULL_ASSIST_ERROR_DEG,
  DEFAULT_INNER_DEADBAND_DEG,
  clip,
  compute_nudge_angle_deg,
  steering_angle_to_g29_target,
)


@dataclass(frozen=True)
class SteerAssistConfig:
  inner_deadband_deg: float = DEFAULT_INNER_DEADBAND_DEG
  full_assist_error_deg: float = DEFAULT_FULL_ASSIST_ERROR_DEG
  tracking_error_deg: float = 5.0
  tracking_duration_s: float = 0.3
  min_opposing_velocity_deg_s: float = 10.0
  candidate_duration_s: float = 0.08
  max_candidate_target_rate_deg_s: float = 60.0
  max_target_step_deg: float = 15.0
  max_target_rate_deg_s: float = 300.0
  wheel_velocity_tau_s: float = 0.06
  override_slew_rate_deg_s: float = 180.0
  release_duration_s: float = 0.2
  max_release_relative_velocity_deg_s: float = 10.0

  def __post_init__(self) -> None:
    nonnegative_fields = (
      "tracking_error_deg",
      "tracking_duration_s",
      "min_opposing_velocity_deg_s",
      "candidate_duration_s",
      "max_candidate_target_rate_deg_s",
      "max_target_step_deg",
      "max_target_rate_deg_s",
      "wheel_velocity_tau_s",
      "override_slew_rate_deg_s",
      "release_duration_s",
      "max_release_relative_velocity_deg_s",
    )
    for field in nonnegative_fields:
      object.__setattr__(self, field, max(0.0, getattr(self, field)))


@dataclass(frozen=True)
class SteerAssistInput:
  wheel_angle_deg: float
  model_target_angle_deg: float | None
  haptic_target_angle_deg: float
  base_target_log_mono_time: int
  fresh: bool
  now: float


@dataclass(frozen=True)
class SteerAssistDecision:
  active: bool
  requested_steering_angle_deg: float
  tracking_status: str
  requested_active: bool
  input_fresh: bool
  wheel_angle_deg: float
  model_target_angle_deg: float | None
  haptic_target_angle_deg: float
  base_target_log_mono_time: int
  model_error_deg: float
  residual_angle_deg: float
  model_haptic_delta_deg: float
  haptic_nudge_angle_deg: float
  desired_blended_target_angle_deg: float
  blended_target_angle_deg: float
  override_slewing: bool
  raw_nudge_angle_deg: float
  nudge_angle_deg: float
  wheel_velocity_deg_s: float
  relative_velocity_deg_s: float
  target_step_deg: float
  target_rate_deg_s: float
  target_interval_s: float | None
  haptic_target_rate_deg_s: float
  candidate_evidence_s: float
  release_since: float | None
  release_evidence_s: float


class SteerAssistController:
  def __init__(self, config: SteerAssistConfig | None = None):
    self.config = SteerAssistConfig() if config is None else config
    self._active = False
    self._tracking_status = "disengaged"
    self._wheel_velocity_deg_s = 0.0
    self._relative_velocity_deg_s = 0.0
    self._last_haptic_target_angle_deg: float | None = None
    self._last_model_target_angle_deg: float | None = None
    self._last_model_target_log_mono_time = 0
    self._target_step_deg = 0.0
    self._target_rate_deg_s = 0.0
    self._target_interval_s: float | None = None
    self._haptic_target_rate_deg_s = 0.0
    self._last_update_time: float | None = None
    self._last_motion_wheel_angle_deg: float | None = None
    self._tracking_since: float | None = None
    self._candidate_last_update_time: float | None = None
    self._candidate_evidence_s = 0.0
    self._candidate_error_sign = 0.0
    self._release_since: float | None = None
    self._release_evidence_s = 0.0
    self._override_slew_last_update_time: float | None = None
    self._override_slew_target_angle_deg: float | None = None
    self._override_slew_complete = False
    self._override_slewing = False

  def update(self, input_data: SteerAssistInput) -> SteerAssistDecision:
    requested_active = input_data.model_target_angle_deg is not None
    wheel_steering = steering_angle_to_g29_target(input_data.wheel_angle_deg)
    model_error_deg = (
      input_data.wheel_angle_deg - input_data.model_target_angle_deg
      if input_data.model_target_angle_deg is not None
      else 0.0
    )
    residual_angle_deg = (
      input_data.wheel_angle_deg - input_data.haptic_target_angle_deg
      if requested_active
      else 0.0
    )
    clipped_model_target_angle_deg = (
      clip(float(input_data.model_target_angle_deg), -180.0, 180.0)
      if input_data.model_target_angle_deg is not None
      else 0.0
    )
    model_haptic_delta_deg = (
      clipped_model_target_angle_deg - input_data.haptic_target_angle_deg
      if requested_active
      else 0.0
    )

    if requested_active and input_data.fresh:
      target_unstable = self._update_motion(input_data)
      model_haptic_aligned = abs(model_haptic_delta_deg) <= self.config.tracking_error_deg
      self._update_detection(residual_angle_deg, target_unstable, model_haptic_aligned, input_data.now)
    elif requested_active:
      self._hold_detection_for_stale_target(input_data.now)
    else:
      self._reset_detection("disengaged")
      self._reset_motion()

    active = requested_active and input_data.fresh and self._tracking_status == "override"
    haptic_nudge_angle_deg = (
      compute_nudge_angle_deg(
        wheel_steering,
        input_data.haptic_target_angle_deg,
        inner_deadband_deg=self.config.inner_deadband_deg,
        full_assist_error_deg=self.config.full_assist_error_deg,
      )
      if active
      else 0.0
    )
    desired_blended_target_angle_deg = (
      clip(input_data.haptic_target_angle_deg + haptic_nudge_angle_deg, -180.0, 180.0)
      if active
      else input_data.haptic_target_angle_deg
    )
    blended_target_angle_deg = (
      self._slew_override_target(
        desired_blended_target_angle_deg,
        clipped_model_target_angle_deg,
        input_data.now,
      )
      if active
      else input_data.haptic_target_angle_deg
    )
    if not active:
      self._override_slewing = False
      self._override_slew_last_update_time = None
      self._override_slew_target_angle_deg = None
      self._override_slew_complete = False

    raw_nudge_angle_deg = (
      blended_target_angle_deg - input_data.model_target_angle_deg
      if active and input_data.model_target_angle_deg is not None
      else 0.0
    )
    self._active = active
    return SteerAssistDecision(
      active=active,
      requested_steering_angle_deg=blended_target_angle_deg if active else 0.0,
      tracking_status=self._tracking_status,
      requested_active=requested_active,
      input_fresh=input_data.fresh,
      wheel_angle_deg=input_data.wheel_angle_deg,
      model_target_angle_deg=input_data.model_target_angle_deg,
      haptic_target_angle_deg=input_data.haptic_target_angle_deg,
      base_target_log_mono_time=input_data.base_target_log_mono_time if requested_active else 0,
      model_error_deg=model_error_deg,
      residual_angle_deg=residual_angle_deg,
      model_haptic_delta_deg=model_haptic_delta_deg,
      haptic_nudge_angle_deg=haptic_nudge_angle_deg,
      desired_blended_target_angle_deg=desired_blended_target_angle_deg,
      blended_target_angle_deg=blended_target_angle_deg,
      override_slewing=self._override_slewing,
      raw_nudge_angle_deg=raw_nudge_angle_deg,
      nudge_angle_deg=raw_nudge_angle_deg,
      wheel_velocity_deg_s=self._wheel_velocity_deg_s,
      relative_velocity_deg_s=self._relative_velocity_deg_s,
      target_step_deg=self._target_step_deg,
      target_rate_deg_s=self._target_rate_deg_s,
      target_interval_s=self._target_interval_s,
      haptic_target_rate_deg_s=self._haptic_target_rate_deg_s,
      candidate_evidence_s=self._candidate_evidence_s,
      release_since=self._release_since,
      release_evidence_s=self._release_evidence_s,
    )

  def _clear_candidate(self) -> None:
    self._candidate_last_update_time = None
    self._candidate_evidence_s = 0.0
    self._candidate_error_sign = 0.0

  def _clear_release_candidate(self) -> None:
    self._release_since = None
    self._release_evidence_s = 0.0

  def _reset_detection(self, status: str) -> None:
    self._tracking_status = status
    self._tracking_since = None
    self._clear_candidate()
    self._clear_release_candidate()
    self._override_slewing = False
    self._override_slew_last_update_time = None
    self._override_slew_target_angle_deg = None
    self._override_slew_complete = False

  def _reset_motion(self) -> None:
    self._last_update_time = None
    self._last_motion_wheel_angle_deg = None
    self._last_haptic_target_angle_deg = None
    self._last_model_target_angle_deg = None
    self._last_model_target_log_mono_time = 0
    self._wheel_velocity_deg_s = 0.0
    self._relative_velocity_deg_s = 0.0
    self._target_step_deg = 0.0
    self._target_rate_deg_s = 0.0
    self._target_interval_s = None
    self._haptic_target_rate_deg_s = 0.0

  def _update_motion(self, input_data: SteerAssistInput) -> bool:
    model_target_angle_deg = float(input_data.model_target_angle_deg)
    if self._last_update_time is None or input_data.now <= self._last_update_time:
      self._reset_motion()
      self._last_update_time = input_data.now
      self._last_motion_wheel_angle_deg = input_data.wheel_angle_deg
      self._last_model_target_angle_deg = model_target_angle_deg
      self._last_model_target_log_mono_time = input_data.base_target_log_mono_time
      self._last_haptic_target_angle_deg = input_data.haptic_target_angle_deg
      return False

    dt = input_data.now - self._last_update_time
    raw_wheel_velocity_deg_s = (input_data.wheel_angle_deg - self._last_motion_wheel_angle_deg) / dt
    alpha = 1.0 if self.config.wheel_velocity_tau_s == 0.0 else 1.0 - math.exp(-dt / self.config.wheel_velocity_tau_s)
    self._wheel_velocity_deg_s += (raw_wheel_velocity_deg_s - self._wheel_velocity_deg_s) * alpha

    self._target_step_deg = model_target_angle_deg - self._last_model_target_angle_deg
    target_timestamp_invalid = False
    if input_data.base_target_log_mono_time > 0 and self._last_model_target_log_mono_time > 0:
      target_interval_s = (input_data.base_target_log_mono_time - self._last_model_target_log_mono_time) / 1e9
      self._target_interval_s = target_interval_s
      if target_interval_s > 0.0:
        self._target_rate_deg_s = self._target_step_deg / target_interval_s
      elif target_interval_s == 0.0 and self._target_step_deg == 0.0:
        self._target_rate_deg_s = 0.0
      else:
        self._target_rate_deg_s = self._target_step_deg / dt
        target_timestamp_invalid = True
    else:
      self._target_interval_s = dt
      self._target_rate_deg_s = self._target_step_deg / dt
    self._haptic_target_rate_deg_s = (input_data.haptic_target_angle_deg - self._last_haptic_target_angle_deg) / dt
    self._relative_velocity_deg_s = self._wheel_velocity_deg_s - self._haptic_target_rate_deg_s

    self._last_update_time = input_data.now
    self._last_motion_wheel_angle_deg = input_data.wheel_angle_deg
    self._last_model_target_angle_deg = model_target_angle_deg
    self._last_model_target_log_mono_time = input_data.base_target_log_mono_time
    self._last_haptic_target_angle_deg = input_data.haptic_target_angle_deg
    return (
      target_timestamp_invalid
      or abs(self._target_step_deg) > self.config.max_target_step_deg
      or abs(self._target_rate_deg_s) > self.config.max_target_rate_deg_s
    )

  def _hold_detection_for_stale_target(self, now: float) -> None:
    self._clear_release_candidate()
    if self._tracking_status == "candidate":
      self._tracking_status = "tracking"
      self._clear_candidate()
    elif self._tracking_status == "acquiring_tracking":
      self._tracking_since = now

  def _update_detection(
    self,
    error_deg: float,
    target_unstable: bool,
    model_haptic_aligned: bool,
    now: float,
  ) -> None:
    target_rate_deg_s = self._haptic_target_rate_deg_s
    relative_velocity_deg_s = self._relative_velocity_deg_s

    if target_unstable and self._tracking_status != "override":
      self._reset_detection("target_unstable")
      return

    if self._tracking_status not in ("tracking", "candidate", "override"):
      if abs(error_deg) > self.config.tracking_error_deg:
        self._reset_detection("disarmed")
        return
      if self._tracking_since is None:
        self._tracking_since = now
      if now - self._tracking_since < self.config.tracking_duration_s:
        self._tracking_status = "acquiring_tracking"
        return
      self._tracking_status = "tracking"

    if self._tracking_status == "tracking":
      opposing = (
        abs(error_deg) > self.config.inner_deadband_deg
        and abs(target_rate_deg_s) <= self.config.max_candidate_target_rate_deg_s
        and abs(relative_velocity_deg_s) >= self.config.min_opposing_velocity_deg_s
        and error_deg * relative_velocity_deg_s > 0.0
      )
      if opposing:
        self._tracking_status = "candidate"
        self._candidate_last_update_time = now
        self._candidate_evidence_s = 0.0
        self._candidate_error_sign = math.copysign(1.0, error_deg)

    if self._tracking_status == "candidate":
      candidate_dt = max(0.0, now - self._candidate_last_update_time) if self._candidate_last_update_time is not None else 0.0
      self._candidate_last_update_time = now
      error_sign_changed = error_deg == 0.0 or math.copysign(1.0, error_deg) != self._candidate_error_sign
      target_stable = abs(target_rate_deg_s) <= self.config.max_candidate_target_rate_deg_s
      moving_away_from_spring = (
        target_stable
        and abs(relative_velocity_deg_s) >= self.config.min_opposing_velocity_deg_s
        and error_deg * relative_velocity_deg_s > 0.0
      )
      moving_with_spring = (
        target_stable
        and abs(relative_velocity_deg_s) >= self.config.min_opposing_velocity_deg_s
        and error_deg * relative_velocity_deg_s < 0.0
      )
      if abs(error_deg) <= self.config.inner_deadband_deg or error_sign_changed or moving_with_spring:
        self._tracking_status = "tracking"
        self._clear_candidate()
      elif moving_away_from_spring:
        self._candidate_evidence_s += candidate_dt
        if self._candidate_evidence_s >= self.config.candidate_duration_s:
          self._tracking_status = "override"

    if self._tracking_status == "override":
      release_ready = (
        abs(error_deg) <= self.config.tracking_error_deg
        and model_haptic_aligned
        and abs(relative_velocity_deg_s) <= self.config.max_release_relative_velocity_deg_s
      )
      if not release_ready:
        self._clear_release_candidate()
      else:
        if self._release_since is None:
          self._release_since = now
        self._release_evidence_s = max(0.0, now - self._release_since)
        if self._release_evidence_s >= self.config.release_duration_s:
          self._tracking_status = "tracking"
          self._clear_candidate()
          self._clear_release_candidate()
    else:
      self._clear_release_candidate()

  def _slew_override_target(self, desired_target_angle_deg: float, model_target_angle_deg: float, now: float) -> float:
    desired_target_angle_deg = clip(desired_target_angle_deg, -180.0, 180.0)
    model_target_angle_deg = clip(model_target_angle_deg, -180.0, 180.0)
    if not self._active or self._override_slew_last_update_time is None or self._override_slew_target_angle_deg is None:
      self._override_slew_last_update_time = now
      self._override_slew_target_angle_deg = model_target_angle_deg
      self._override_slew_complete = not math.isfinite(self.config.override_slew_rate_deg_s)

    if self._override_slew_complete:
      self._override_slewing = False
      return desired_target_angle_deg

    dt = max(0.0, now - self._override_slew_last_update_time)
    self._override_slew_last_update_time = now
    max_delta_deg = self.config.override_slew_rate_deg_s * dt
    target_delta_deg = desired_target_angle_deg - self._override_slew_target_angle_deg
    if abs(target_delta_deg) <= max_delta_deg:
      self._override_slew_target_angle_deg = desired_target_angle_deg
      self._override_slew_complete = True
      self._override_slewing = False
      return desired_target_angle_deg

    self._override_slew_target_angle_deg += clip(target_delta_deg, -max_delta_deg, max_delta_deg)
    self._override_slewing = True
    return self._override_slew_target_angle_deg
