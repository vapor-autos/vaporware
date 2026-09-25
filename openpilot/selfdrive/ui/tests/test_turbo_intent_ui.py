from types import SimpleNamespace

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.turbo_intent import PROTOCOL_VERSION, REQUEST_SERVICE, STATE_SERVICE
from openpilot.selfdrive.ui.turbo_intent import IntentBorder, intent_label, intent_border, half_border_clip, split_border_segment


class SM(dict):
  def __init__(self):
    super().__init__((s, getattr(messaging.new_message(s), s)) for s in (STATE_SERVICE, REQUEST_SERVICE))
    self.seen = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.recv_time = dict.fromkeys(self, 10.0)
    self[STATE_SERVICE].from_dict({"protocolVersion": PROTOCOL_VERSION, "status": "idle", "mode": "shadow", "minSpeed": 2.0, "maxSpeed": 5.0})
    self[REQUEST_SERVICE].operatorId = "operator"
    self[REQUEST_SERVICE].localStatus = "idle"


@pytest.mark.parametrize("reason,label", [("not_engaged", "NOT ENGAGED"), ("standstill", "STOPPED"),
                                         ("speed_out_of_range", "SPEED 2-5 M/S REQUIRED")])
def test_idle_display_shows_mode_and_current_gate(reason, label):
  sm = SM()
  sm[STATE_SERVICE].reason = reason
  assert intent_label(sm, 10.0) == f"PADDLES: SHADOW | {label}"


def test_uncapped_execute_mode_is_visible_and_does_not_show_zero_speed_range():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "minSpeed": 0, "maxSpeed": 0,
                             "reason": "speed_out_of_range"})
  assert intent_label(sm, 10.0) == "PADDLES: WAITING | NO SPEED CAP | VALID FORWARD SPEED REQUIRED"
  sm[STATE_SERVICE].available = True
  sm[STATE_SERVICE].reason = ""
  assert intent_label(sm, 10.0) == "PADDLES: READY | NO SPEED CAP"


def test_terminal_status_visible_then_returns_to_steady_idle():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"status": "rejected", "reason": "standstill", "direction": "right", "requestId": 1,
                             "operatorId": "operator"})
  sm[REQUEST_SERVICE].requestId = 1
  sm[REQUEST_SERVICE].direction = "right"
  sm[REQUEST_SERVICE].localStatus = "pending"
  assert intent_label(sm, 10.0) == "RIGHT LANE CHANGE: REJECTED | STOPPED"
  sm[REQUEST_SERVICE].localStatus = "rejected"
  assert intent_label(sm, 10.0) == "RIGHT LANE CHANGE: REJECTED | STOPPED"
  sm[REQUEST_SERVICE].localStatus = "idle"
  assert intent_label(sm, 10.0) == "PADDLES: SHADOW"


def test_new_operator_does_not_display_previous_operators_rejection():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"status": "rejected", "reason": "vehicle_unhealthy", "operatorId": "previous", "requestId": 4})
  assert intent_label(sm, 10.0) == "PADDLES: SHADOW"


@pytest.mark.parametrize("status,label", [("pending", "REQUESTING"), ("canceling", "CANCELING"), ("unknown", "STATUS UNKNOWN")])
def test_local_pending_cancel_unknown_are_not_claimed_complete(status, label):
  sm = SM()
  sm[REQUEST_SERVICE].localStatus = status
  assert intent_label(sm, 10.0) == f"LANE CHANGE: {label}"


@pytest.mark.parametrize("status,label", [("executing", "EXECUTING"), ("timedOut", "TAKE OVER"), ("noModelResponse", "TAKE OVER")])
def test_active_or_takeover_state_always_wins_over_local_idle(status, label):
  sm = SM()
  sm[STATE_SERVICE].status = status
  sm[STATE_SERVICE].direction = "left"
  assert intent_label(sm, 10.0) == f"LEFT LANE CHANGE: {label}"
  sm[STATE_SERVICE].operatorOverride = True
  if status == "executing":
    assert intent_label(sm, 10.0).endswith(" | WHEEL OVERRIDE")
  else:
    assert "TAKE OVER" in intent_label(sm, 10.0)


def test_stale_input_is_not_shown_as_ready():
  sm = SM()
  sm.recv_time[REQUEST_SERVICE] = 9.0
  assert intent_label(sm, 10.0) == "PADDLES: INPUT STALE"
  sm.recv_time[STATE_SERVICE] = 9.0
  assert intent_label(sm, 10.0) == "LANE CHANGE: LINK STALE"


@pytest.mark.parametrize("side", ["left", "right"])
def test_only_authoritative_executing_direction_lights_border(side):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": "executing", "direction": side})
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder(side=side)
  # A stale/local opposite press cannot move the highlight to the other side.
  sm[REQUEST_SERVICE].direction = "right" if side == "left" else "left"
  sm[REQUEST_SERVICE].localStatus = "pending"
  assert intent_border(sm, 10.0, engaged=True).side == side


@pytest.mark.parametrize("status", ["idle", "awaitingEvaluation", "shadow", "completed", "rejected", "canceled", "expired", "interrupted"])
def test_nonexecuting_states_have_no_intent_decoration(status):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": status, "direction": "left"})
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()


@pytest.mark.parametrize("mode", ["off", "shadow"])
def test_shadow_and_off_never_look_like_actual_lane_change(mode):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": mode, "status": "executing", "direction": "left"})
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()


def test_override_disengagement_and_critical_alert_take_priority():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": "executing", "direction": "left"})
  assert intent_border(sm, 10.0, engaged=False) == IntentBorder()
  assert intent_border(sm, 10.0, engaged=True, override=True) == IntentBorder()
  assert intent_border(sm, 10.0, engaged=True, override=True, critical=True) == IntentBorder(takeover=True)
  sm[STATE_SERVICE].operatorOverride = True
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()


@pytest.mark.parametrize("status", ["timedOut", "noModelResponse"])
def test_takeover_remains_red_instead_of_silently_looking_complete(status):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": status, "direction": "left"})
  assert intent_border(sm, 10.0, engaged=True, override=True) == IntentBorder(takeover=True)
  assert intent_border(sm, 10.0, engaged=False) == IntentBorder()


@pytest.mark.parametrize("age", [-0.001, 0.351, 10])
def test_lost_or_invalid_active_feedback_does_not_keep_violet(age):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": "executing", "direction": "right"})
  sm.recv_time[STATE_SERVICE] = 10.0 - age
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder(takeover=True)


def test_missing_feedback_and_invalid_direction_never_light_a_side():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"mode": "execute", "status": "executing", "direction": "none"})
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()
  sm.seen[STATE_SERVICE] = False
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()


@pytest.mark.parametrize("side", ["left", "right"])
def test_turn_uses_same_authoritative_half_border_and_turn_diagnostics(side):
  sm = SM()
  sm[STATE_SERVICE].from_dict({"maneuver": "turn", "mode": "execute", "status": "executing", "direction": side})
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder(side=side)
  assert intent_label(sm, 10.0) == f"{side.upper()} TURN: EXECUTING"
  assert intent_border(sm, 10.0, engaged=True, override=True) == IntentBorder()
  sm[STATE_SERVICE].mode = "shadow"
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder()


def test_old_protocol_feedback_never_looks_like_valid_execution():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"protocolVersion": 1, "mode": "execute", "status": "executing", "direction": "left"})
  assert intent_label(sm, 10.0) == "INTENT: PROTOCOL MISMATCH"
  assert intent_border(sm, 10.0, engaged=True) == IntentBorder(takeover=True)


@pytest.mark.parametrize("width,height", [(1920, 1080), (536, 240), (321, 181)])
def test_half_border_clip_covers_full_stroke_without_center_divider(width, height):
  rect = SimpleNamespace(x=30, y=30, width=width, height=height)
  left = half_border_clip(rect, "left", 12)
  right = half_border_clip(rect, "right", 12)
  assert left[0] == 18 and right[0]+right[2] == 30+width+12
  assert left[0]+left[2] == right[0]
  assert left[1] == right[1] == 18
  assert left[3] == right[3] == height+24


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("side", ["left", "right"])
def test_projected_box_edge_splits_at_its_own_center(side, reverse):
  start, end = ((20, 30), (100, 70))
  if reverse:
    start, end = end, start
  segments = split_border_segment(start, end, 60, side)
  assert len(segments) == 2
  assert segments[0][0] == start and segments[-1][1] == end
  assert segments[0][1] == segments[1][0] == (60, 50)
  for a, b, highlight in segments:
    assert highlight == (((a[0]+b[0])/2 < 60) == (side == "left"))
