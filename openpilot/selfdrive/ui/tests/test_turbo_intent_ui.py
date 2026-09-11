from types import SimpleNamespace

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.turbo_intent import REQUEST_SERVICE, STATE_SERVICE
from openpilot.selfdrive.ui.turbo_intent import intent_label, intent_panel_geometry


class SM(dict):
  def __init__(self):
    super().__init__((s, getattr(messaging.new_message(s), s)) for s in (STATE_SERVICE, REQUEST_SERVICE))
    self.seen = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)
    self.recv_time = dict.fromkeys(self, 10.0)
    self[STATE_SERVICE].from_dict({"status": "idle", "mode": "shadow", "minSpeed": 2.0, "maxSpeed": 5.0})
    self[REQUEST_SERVICE].operatorId = "operator"
    self[REQUEST_SERVICE].localStatus = "idle"


@pytest.mark.parametrize("reason,label", [("not_engaged", "NOT ENGAGED"), ("standstill", "STOPPED"),
                                         ("speed_out_of_range", "SPEED 2-5 M/S REQUIRED")])
def test_idle_display_shows_mode_and_current_gate(reason, label):
  sm = SM()
  sm[STATE_SERVICE].reason = reason
  assert intent_label(sm, 10.0) == f"PADDLES: SHADOW | {label}"


def test_terminal_status_visible_then_returns_to_steady_idle():
  sm = SM()
  sm[STATE_SERVICE].from_dict({"status": "rejected", "reason": "standstill", "direction": "right", "requestId": 1,
                             "operatorId": "operator"})
  sm[REQUEST_SERVICE].requestId = 1
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


@pytest.mark.parametrize("width,height", [(1920, 1080), (536, 240), (320, 180)])
def test_badge_has_fixed_footprint_and_fits_camera_view(width, height):
  rect = SimpleNamespace(x=30, y=30, width=width, height=height)
  x, y, w, h, _ = intent_panel_geometry(rect)
  assert x >= rect.x and x+w <= rect.x+width
  assert y >= rect.y and y+h <= rect.y+height
  assert x+w/2 == pytest.approx(rect.x+width/2)
