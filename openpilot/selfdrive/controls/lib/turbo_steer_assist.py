from __future__ import annotations

from dataclasses import dataclass
import math
import time


STEERING_TARGET_MAX_ANGLE_DEG = 180.0
DEFAULT_STALE_TIMEOUT_S = 0.25
DEFAULT_CONTEXT_TIMEOUT_S = 0.35
DEFAULT_TARGET_MISMATCH_DEG = 15.0
DEFAULT_RELEASE_FADE_S = 0.2


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
  requested_angle_deg: float | None
  status: str
  receive_age_s: float | None
  context_age_s: float | None
  base_model_delta_deg: float | None
  sequence: int
  base_model_log_mono_time: int


@dataclass(frozen=True)
class TurboSteerAssistAppliedState:
  applied: bool
  status: str
  target_available: bool
  requested_angle_deg: float
  model_angle_deg: float
  final_angle_deg: float
  source_sequence: int
  source_base_model_log_mono_time: int


def resolve_turbo_steer_assist_state(
  apply_enabled: bool,
  decision: TurboSteerAssistDecision | None,
  model_angle_deg: float,
) -> TurboSteerAssistAppliedState:
  target_available = decision is not None and decision.target_angle_deg is not None
  applied = bool(apply_enabled and target_available)
  requested_angle_deg = 0.0
  if decision is not None and decision.requested_angle_deg is not None and math.isfinite(decision.requested_angle_deg):
    requested_angle_deg = float(decision.requested_angle_deg)
  final_angle_deg = float(decision.target_angle_deg) if applied else float(model_angle_deg)

  if decision is None:
    status = "unsupported"
  elif decision.status == "active":
    status = "applied" if applied else "apply_disabled"
  else:
    status = decision.status

  return TurboSteerAssistAppliedState(
    applied=applied,
    status=status,
    target_available=target_available,
    requested_angle_deg=requested_angle_deg,
    model_angle_deg=float(model_angle_deg),
    final_angle_deg=final_angle_deg,
    source_sequence=0 if decision is None else decision.sequence,
    source_base_model_log_mono_time=0 if decision is None else decision.base_model_log_mono_time,
  )


class TurboSteerAssistApplicator:
  def __init__(self, release_fade_s: float = DEFAULT_RELEASE_FADE_S):
    self.release_fade_s = max(0.0, release_fade_s)
    self._was_applied = False
    self._last_applied_correction_deg = 0.0
    self._last_requested_angle_deg = 0.0
    self._release_start_time: float | None = None
    self._release_start_correction_deg = 0.0

  def update(
    self,
    apply_enabled: bool,
    decision: TurboSteerAssistDecision | None,
    model_angle_deg: float,
    now: float | None = None,
  ) -> TurboSteerAssistAppliedState:
    now = time.monotonic() if now is None else now
    state = resolve_turbo_steer_assist_state(apply_enabled, decision, model_angle_deg)

    if state.applied:
      self._release_start_time = None
      self._release_start_correction_deg = 0.0
      self._was_applied = True
      self._last_applied_correction_deg = state.final_angle_deg - state.model_angle_deg
      self._last_requested_angle_deg = state.requested_angle_deg
      return state

    # Only a fresh, intentional release fades the previous correction. Safety
    # fallbacks such as stale/invalid input or lateral disengagement remain immediate.
    if apply_enabled and decision is not None and decision.status == "inactive" and self._was_applied:
      if self._release_start_time is None or now < self._release_start_time:
        self._release_start_time = now
        self._release_start_correction_deg = self._last_applied_correction_deg

      release_elapsed_s = max(0.0, now - self._release_start_time)
      release_progress = 1.0 if self.release_fade_s == 0.0 else clip(release_elapsed_s / self.release_fade_s, 0.0, 1.0)
      if release_progress < 1.0:
        correction_deg = self._release_start_correction_deg * (1.0 - release_progress)
        return TurboSteerAssistAppliedState(
          applied=True,
          status="releasing",
          target_available=False,
          requested_angle_deg=self._last_requested_angle_deg,
          model_angle_deg=float(model_angle_deg),
          final_angle_deg=clip_steering_angle_deg(float(model_angle_deg) + correction_deg),
          source_sequence=decision.sequence,
          source_base_model_log_mono_time=decision.base_model_log_mono_time,
        )

    self._reset()
    return state

  def _reset(self) -> None:
    self._was_applied = False
    self._last_applied_correction_deg = 0.0
    self._last_requested_angle_deg = 0.0
    self._release_start_time = None
    self._release_start_correction_deg = 0.0


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
    requested_angle_deg: float | None = None
    sequence = 0
    base_model_log_mono_time = 0

    def finish(status: str, target_angle_deg: float | None = None) -> TurboSteerAssistDecision:
      self._override_session_active = status == "active"
      return TurboSteerAssistDecision(
        target_angle_deg=target_angle_deg,
        requested_angle_deg=requested_angle_deg,
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
    assist = self.sm["turboSteerAssist"]
    requested_angle_deg = float(assist.requestedSteeringAngleDeg)
    sequence = int(assist.sequence)
    base_model_log_mono_time = int(assist.baseModelLogMonoTime)
    if receive_age_s is None or receive_age_s > self.stale_timeout_s:
      return finish("stale")
    if not assist.active:
      return finish("inactive")

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

    if not math.isfinite(requested_angle_deg):
      return finish("invalid_target")

    return finish("active", clip_steering_angle_deg(requested_angle_deg))

  def _age(self, now: float) -> float | None:
    if not self.sm.seen["turboSteerAssist"]:
      return None
    return now - self.sm.recv_time["turboSteerAssist"]
