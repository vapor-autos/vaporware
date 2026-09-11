import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.turbo_intent import STATE_SERVICE
from openpilot.tools.turbo.g29_compat import TurboG29
from openpilot.tools.turbo.intent import PaddleIntentController, read_intent_feedback


FEEDBACK = {"protocolVersion": 1, "sessionId": "session", "epoch": "epoch", "status": "idle"}


def step(c, buttons=None, downs=(), feedback=None, fresh=True, ready=True, reverse=False, now=10.0):
  return c.update(buttons or {}, [{"type": "button_down", "control": b} for b in downs],
                  FEEDBACK if feedback is None else feedback, 10_000_000_000, fresh, ready, reverse, now)


def controller():
  c = PaddleIntentController()
  step(c)
  return c


@pytest.mark.parametrize("paddle,direction", [("left_paddle", "left"), ("right_paddle", "right")])
def test_single_press_and_held_button_are_one_transaction(paddle, direction):
  c = controller()
  first = step(c, {paddle: True}, [paddle], now=10.01)
  assert first["action"] == "request" and first["direction"] == direction
  for t in (10.02, 10.1, 10.2):
    repeated = step(c, {paddle: True}, now=t)
    assert repeated["requestId"] == first["requestId"]
    assert repeated["baseFeedbackLogMonoTime"] == first["baseFeedbackLogMonoTime"]
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "shadow"}
  assert step(c, {paddle: True}, feedback=ack, now=10.21)["action"] == "none"
  assert step(c, {paddle: True}, [paddle], feedback=ack, now=10.22)["action"] == "none"
  step(c, feedback=ack, now=10.23)
  assert step(c, {paddle: True}, [paddle], feedback=ack, now=10.24)["requestId"] == 2


@pytest.mark.parametrize("buttons,downs,reason", [
  ({"left_paddle": True, "right_paddle": True}, ["left_paddle", "right_paddle"], "ambiguous_paddles"),
  ({}, ["left_paddle", "right_paddle"], "ambiguous_paddles"),
  ({"left_paddle": True, "L2": True}, ["left_paddle", "L2"], "cancel_or_engage"),
  ({"left_paddle": True, "L2": True}, ["left_paddle"], "cancel_or_engage"),
  ({"left_paddle": True}, ["left_paddle", "L3"], "cancel_or_engage"),
])
def test_ambiguous_and_cancel_combinations_never_request(buttons, downs, reason):
  c = controller()
  wire = step(c, buttons, downs)
  assert wire["action"] == "none" and wire["localStatus"] == reason


def test_start_or_reconnect_with_paddle_held_requires_release():
  c = PaddleIntentController()
  assert step(c, {"left_paddle": True}, ["left_paddle"])["action"] == "none"
  step(c)
  assert step(c, {"left_paddle": True}, ["left_paddle"], now=10.01)["action"] == "request"
  new = {**FEEDBACK, "epoch": "new"}
  assert step(c, {"left_paddle": True}, ["left_paddle"], feedback=new, now=10.02)["action"] == "none"
  step(c, feedback=new, now=10.03)
  assert step(c, {"right_paddle": True}, ["right_paddle"], feedback=new, now=10.04)["action"] == "request"


@pytest.mark.parametrize("kwargs", [{"ready": False}, {"reverse": True}, {"fresh": False}])
def test_unready_request_is_not_queued(kwargs):
  c = controller()
  assert step(c, {"left_paddle": True}, ["left_paddle"], **kwargs)["action"] == "none"
  assert step(c, {"left_paddle": True})["action"] == "none"


def test_unknown_acknowledgment_does_not_allow_retry_press():
  c = controller()
  step(c, {"left_paddle": True}, ["left_paddle"])
  wire = step(c, now=10.36)
  assert wire["action"] == "none" and wire["localStatus"] == "unknown"
  assert step(c, {"right_paddle": True}, ["right_paddle"], now=10.4)["localStatus"] == "unknown"
  assert c.sequence == 1
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "rejected"}
  step(c, feedback=ack, now=10.41)
  assert not c.uncertain


def test_l2_cancels_pending_but_cannot_claim_to_abort_execution():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  canceled = step(c, {"L2": True}, ["L2"], now=10.01)
  assert canceled["action"] == "cancel" and canceled["requestId"] == r["requestId"]
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "executing"}
  assert step(c, {"L2": True}, feedback=ack, now=10.02)["localStatus"] == "executing"


def test_pending_ack_does_not_swallow_cancel():
  c = controller()
  step(c, {"left_paddle": True}, ["left_paddle"])
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "awaitingEvaluation"}
  assert step(c, {"L2": True}, ["L2"], feedback=ack, now=10.01)["action"] == "cancel"
  assert step(c, feedback={**ack, "status": "canceled"}, now=10.02)["action"] == "none"


@pytest.mark.parametrize("status", ["awaitingEvaluation", "executing", "timedOut", "noModelResponse"])
def test_busy_ugv_never_queues_paddle(status):
  c = controller()
  assert step(c, {"left_paddle": True}, ["left_paddle"], feedback={**FEEDBACK, "status": status})["action"] == "none"
  assert c.sequence == 0


@pytest.mark.parametrize("mask", range(256))
def test_g29_all_byte_one_combinations_and_release(mask):
  wheel = TurboG29.__new__(TurboG29)  # No HID handle, thread or force feedback.
  state = {"buttons": {"cross": 1}}
  names = ("right_paddle", "left_paddle", "R2", "L2", "Share", "Options", "R3", "L3")
  wheel.apply_misc(state, mask)
  for bit, name in enumerate(names):
    assert state["buttons"][name] == bool(mask & (1 << bit))
  assert state["buttons"]["cross"] == 1
  wheel.apply_misc(state, 0)
  assert not any(state["buttons"][name] for name in names)


def test_combination_transitions_do_not_retrigger_held_paddle():
  wheel = TurboG29.__new__(TurboG29)
  state = {"buttons": {}}
  previous = {}
  downs = []
  for mask in (0, 2, 10, 2, 3, 1, 0):
    wheel.apply_misc(state, mask)
    downs.append([name for name, value in state["buttons"].items() if value and not previous.get(name)])
    previous = state["buttons"].copy()
  assert downs == [[], ["left_paddle"], ["L2"], [], ["right_paddle"], [], []]


class FeedbackSubMaster:
  """Only in-memory messages: no sockets, HID or live control subscribers."""
  def __init__(self):
    services = (STATE_SERVICE, "turboSteerAssistState")
    self.data = {s: getattr(messaging.new_message(s), s) for s in services}
    self[STATE_SERVICE].from_dict(FEEDBACK)
    self.seen = dict.fromkeys(services, True)
    self.valid = dict.fromkeys(services, True)
    self.recv_time = dict.fromkeys(services, 10.0)
    self.logMonoTime = dict.fromkeys(services, 10_000_000_000)

  def __getitem__(self, service):
    return self.data[service]


def test_clock_is_sampled_after_receiving_feedback(mocker):
  sm = FeedbackSubMaster()
  sm.recv_time = dict.fromkeys(sm.recv_time, 10.00015)
  # The old loop-start clock wrongly calls this brand-new packet stale.
  assert not read_intent_feedback(sm, now=10.0).fresh
  mocker.patch("openpilot.tools.turbo.intent.time.monotonic", return_value=10.0003)
  feedback = read_intent_feedback(sm)
  assert feedback.fresh and feedback.applied_fresh
  assert feedback.age_s == pytest.approx(0.00015)


@pytest.mark.parametrize("press_phase", range(5))
@pytest.mark.parametrize("feedback_phase", range(5))
@pytest.mark.parametrize("paddle", ["left_paddle", "right_paddle"])
def test_50hz_reader_10hz_publisher_preserves_request_at_every_phase(mocker, press_phase, feedback_phase, paddle):
  sm = FeedbackSubMaster()
  c = PaddleIntentController()
  clock = mocker.patch("openpilot.tools.turbo.intent.time.monotonic")
  first_publish = None
  published = []
  received_ack = False
  frozen_reference = None
  for frame in range(40):
    loop_start = 10.0 + frame * 0.02
    # Real SubMaster receives AFTER g29d sampled the loop-start clock.
    if frame % 5 == feedback_phase:
      sm.recv_time[STATE_SERVICE] = loop_start + 0.00015
      sm.logMonoTime[STATE_SERVICE] = int(loop_start * 1e9)
    if frame % 3 == 0:
      sm.recv_time["turboSteerAssistState"] = loop_start + 0.0002
    if first_publish is not None and frame >= first_publish + 4:
      sm[STATE_SERVICE].operatorId = c.operator_id
      sm[STATE_SERVICE].requestId = 1
      sm[STATE_SERVICE].status = "shadow"
    clock.return_value = loop_start + 0.0003
    context = read_intent_feedback(sm)
    assert context.fresh and context.applied_fresh
    pressed = frame == 5 + press_phase
    wire = c.update({paddle: frame >= 5 + press_phase}, [{"type": "button_down", "control": paddle}] if pressed else [],
                    context.data, context.log_mono_time, context.fresh, context.applied_fresh, False, context.now)
    assert wire["localStatus"] not in ("canceling", "unknown")
    if wire["action"] == "request":
      frozen_reference = wire["baseFeedbackLogMonoTime"] if frozen_reference is None else frozen_reference
      assert wire["baseFeedbackLogMonoTime"] == frozen_reference
    if frame % 5 == 0 and wire["action"] != "none":
      published.append(wire)
      first_publish = frame if first_publish is None else first_publish
    received_ack |= wire["localStatus"] == "shadow"
  assert published and received_ack
  assert {r["requestId"] for r in published} == {1}
  assert {r["action"] for r in published} == {"request"}
  assert {r["direction"] for r in published} == {"left" if paddle == "left_paddle" else "right"}


def test_actual_feedback_loss_cancels_pending_without_forgetting_unknown():
  c = controller()
  r = step(c, {"right_paddle": True}, ["right_paddle"])
  lost = step(c, fresh=False, feedback={}, now=10.1)
  assert lost["action"] == "cancel" and lost["localStatus"] == "canceling"
  assert lost["baseFeedbackLogMonoTime"] == r["baseFeedbackLogMonoTime"]
  assert c.context == ("session", "epoch")
  assert step(c, fresh=False, feedback={}, now=10.36)["localStatus"] == "unknown"
  assert step(c, now=10.4)["localStatus"] == "unknown"
  assert step(c, {"right_paddle": True}, ["right_paddle"], now=10.42)["action"] == "none"
  assert c.uncertain and c.sequence == 1
  late_ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "rejected"}
  assert step(c, feedback=late_ack, now=10.5)["localStatus"] == "rejected"


def test_new_epoch_clears_unknown_but_not_from_a_held_paddle():
  c = controller()
  step(c, {"left_paddle": True}, ["left_paddle"])
  step(c, now=10.4)
  assert c.uncertain
  wire = step(c, {"left_paddle": True}, ["left_paddle"], feedback={**FEEDBACK, "epoch": "new"}, now=10.42)
  assert wire["action"] == "none" and not c.uncertain
  assert not c.last_request


def test_acknowledged_identity_survives_for_ui_without_retransmitting_action():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "shadow"}
  wire = step(c, feedback=ack, now=10.1)
  assert wire["action"] == "none" and wire["localStatus"] == "shadow"
  assert wire["requestId"] == r["requestId"] and wire["operatorId"] == r["operatorId"]
  assert step(c, feedback=ack, now=12.2)["localStatus"] == "idle"
