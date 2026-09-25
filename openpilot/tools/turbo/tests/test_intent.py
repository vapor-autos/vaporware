import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.turbo_intent import PROTOCOL_VERSION, STATE_SERVICE
from openpilot.tools.turbo.g29_compat import TurboG29
from openpilot.tools.turbo.intent import MANEUVER_BUTTONS, IntentPublishSchedule, PaddleIntentController, read_intent_feedback


FEEDBACK = {"protocolVersion": PROTOCOL_VERSION, "sessionId": "session", "epoch": "epoch", "status": "idle", "available": True,
            "maneuver": "laneChange", "direction": "left"}


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
  ack = {**FEEDBACK, "direction": direction, "operatorId": c.operator_id, "requestId": 1, "status": "shadow"}
  assert step(c, {paddle: True}, feedback=ack, now=10.21)["action"] == "none"
  assert step(c, {paddle: True}, [paddle], feedback=ack, now=10.22)["action"] == "none"
  step(c, feedback=ack, now=10.23)
  assert step(c, {paddle: True}, [paddle], feedback=ack, now=10.24)["requestId"] == 2


@pytest.mark.parametrize("buttons,downs,reason", [
  ({"left_paddle": True, "right_paddle": True}, ["left_paddle", "right_paddle"], "ambiguous_maneuvers"),
  ({}, ["left_paddle", "right_paddle"], "ambiguous_maneuvers"),
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
  assert step(c, now=10.36)["action"] == "none"
  wire = step(c, now=11.01)
  assert wire["action"] == "cancel" and wire["localStatus"] == "unknown"
  assert step(c, {"right_paddle": True}, ["right_paddle"], now=11.1)["localStatus"] == "unknown"
  assert c.sequence == 1
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "rejected"}
  step(c, feedback=ack, now=11.2)
  assert not c.uncertain


def test_l2_cancels_pending_but_cannot_claim_to_abort_execution():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  canceled = step(c, {"L2": True}, ["L2"], now=10.01)
  assert canceled["action"] == "cancel" and canceled["requestId"] == r["requestId"]
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "executing", "pulseMonoTime": 10_000_000_000}
  assert step(c, {"L2": True}, feedback=ack, now=10.02)["localStatus"] == "executing"


def test_pending_ack_does_not_swallow_cancel():
  c = controller()
  step(c, {"left_paddle": True}, ["left_paddle"])
  ack = {**FEEDBACK, "operatorId": c.operator_id, "requestId": 1, "status": "awaitingEvaluation"}
  assert step(c, {"L2": True}, ["L2"], feedback=ack, now=10.01)["action"] == "cancel"
  assert step(c, feedback={**ack, "status": "canceled"}, now=10.02)["action"] == "none"


def receipt(request, result, *, action=None, **changes):
  keys = ("protocolVersion", "sessionId", "epoch", "operatorId", "requestId", "maneuver", "direction", "action")
  return {**{k: request[k] for k in keys}, "result": result, "status": "", "action": action or request["action"], **changes}


def test_busy_request_receipt_resolves_without_replacing_active_transaction():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  feedback = {**FEEDBACK, "operatorId": "other", "requestId": 70, "status": "executing", "available": False,
              "receipt": receipt(r, "rejected_busy")}
  result = step(c, feedback=feedback, now=10.1)
  assert result["action"] == "none" and result["localStatus"] == "rejected_busy"
  assert c.pending is None and not c.uncertain


def test_lost_request_reconciles_same_id_and_requires_authoritative_available():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  expired = step(c, now=10.4)
  assert expired["action"] == "none" and c.pending is not None
  reconcile = step(c, now=11.1)
  assert reconcile["action"] == "cancel" and reconcile["requestId"] == r["requestId"] and c.uncertain
  feedback = {**FEEDBACK, "available": False, "phase": "cooldown", "receipt": receipt(r, "canceled_before_consumption", action="cancel")}
  assert step(c, feedback=feedback, now=11.2)["action"] == "none"
  assert not c.uncertain and c.pending is None
  assert step(c, {"right_paddle": True}, ["right_paddle"], feedback=feedback, now=11.3)["action"] == "none"
  ready = {**feedback, "available": True, "phase": "idle"}
  assert step(c, {"right_paddle": True}, feedback=ready, now=12.4)["action"] == "none"
  step(c, feedback=ready, now=12.5)
  assert step(c, {"right_paddle": True}, ["right_paddle"], feedback=ready, now=12.6)["requestId"] == 2


@pytest.mark.parametrize("field,value", [("action", "request"), ("epoch", "old"), ("sessionId", "old"),
                                         ("protocolVersion", 2), ("direction", "right"), ("maneuver", "turn"),
                                         ("requestId", 2), ("operatorId", "other"), ("result", "stale_context")])
def test_wrong_cancel_receipt_never_clears_uncertainty(field, value):
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  step(c, now=11.1)
  bad_receipt = receipt(r, "canceled_before_consumption", action="cancel")
  bad_receipt[field] = value
  result = step(c, feedback={**FEEDBACK, "receipt": bad_receipt}, now=11.2)
  assert result["action"] == "cancel" and c.uncertain


def test_late_consumed_state_resolves_unknown_but_cannot_retrigger():
  c = controller()
  r = step(c, {"left_paddle": True}, ["left_paddle"])
  step(c, now=11.1)
  feedback = {**FEEDBACK, **r, "status": "executing", "available": False, "pulseMonoTime": 10_100_000_000}
  assert step(c, feedback=feedback, now=11.2)["action"] == "none"
  assert not c.uncertain
  assert step(c, {"right_paddle": True}, ["right_paddle"], feedback=feedback, now=11.3)["action"] == "none"
  assert c.sequence == 1


@pytest.mark.parametrize("status", ["awaitingEvaluation", "executing"])
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
      sm[STATE_SERVICE].direction = "left" if paddle == "left_paddle" else "right"
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
  assert lost["action"] == "none" and lost["localStatus"] == "canceling"
  assert lost["baseFeedbackLogMonoTime"] == r["baseFeedbackLogMonoTime"]
  assert c.context == ("session", "epoch")
  assert step(c, fresh=False, feedback={}, now=11.01)["localStatus"] == "unknown"
  assert step(c, now=11.1)["localStatus"] == "unknown"
  assert step(c, {"right_paddle": True}, ["right_paddle"], now=11.12)["action"] == "cancel"
  assert c.uncertain and c.sequence == 1
  late_ack = {**FEEDBACK, "direction": "right", "operatorId": c.operator_id, "requestId": 1, "status": "rejected"}
  assert step(c, feedback=late_ack, now=11.2)["localStatus"] == "rejected"


def test_new_epoch_clears_unknown_but_not_from_a_held_paddle():
  c = controller()
  step(c, {"left_paddle": True}, ["left_paddle"])
  step(c, now=11.01)
  assert c.uncertain
  wire = step(c, {"left_paddle": True}, ["left_paddle"], feedback={**FEEDBACK, "epoch": "new"}, now=11.1)
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


@pytest.mark.parametrize("button", MANEUVER_BUTTONS)
def test_all_maneuver_buttons_share_single_press_and_busy_semantics(button):
  c = controller()
  kind, direction = MANEUVER_BUTTONS[button]
  r = step(c, {button: True}, [button])
  assert (r["maneuver"], r["direction"]) == (kind, direction)
  ack = {**FEEDBACK, **r, "status": "executing"}
  assert step(c, {button: True}, feedback=ack, now=10.01)["action"] == "none"
  step(c, feedback=ack, now=10.02)
  for other in MANEUVER_BUTTONS:
    assert step(c, {other: True}, [other], feedback=ack, now=10.03)["action"] == "none"
    step(c, feedback=ack, now=10.04)
  assert c.sequence == 1


@pytest.mark.parametrize("first", MANEUVER_BUTTONS)
@pytest.mark.parametrize("second", MANEUVER_BUTTONS)
def test_mixed_maneuver_batch_never_chooses_by_iteration_order(first, second):
  c = controller()
  result = step(c, {first: True, second: True}, [first, second])
  assert result["action"] == "none" and result["localStatus"] == "ambiguous_maneuvers"


@pytest.mark.parametrize("button", ["S", "O"])
def test_face_button_held_across_reconnect_never_starts(button):
  c = PaddleIntentController()
  assert step(c, {button: True}, [button])["action"] == "none"
  assert step(c, {button: True})["action"] == "none"
  step(c)
  assert step(c, {button: True}, [button])["maneuver"] == "turn"
  assert step(c, {button: True}, [button], feedback={**FEEDBACK, "epoch": "new"})["action"] == "none"


@pytest.mark.parametrize("button", ["S", "O"])
@pytest.mark.parametrize("control", ["L2", "L3"])
def test_engage_cancel_wins_over_face_buttons_even_when_already_held(button, control):
  c = controller()
  assert step(c, {button: True, control: True}, [button])["localStatus"] == "cancel_or_engage"


def test_wrong_kind_ack_cannot_hide_pending_turn():
  c = controller()
  r = step(c, {"S": True}, ["S"])
  ack = {**FEEDBACK, **r, "status": "executing", "maneuver": "laneChange"}
  assert step(c, feedback=ack, now=10.1)["action"] == "request"
  assert step(c, feedback=ack, now=11.01)["localStatus"] == "unknown"
  assert step(c, feedback=ack, now=10.42)["localStatus"] == "unknown"


@pytest.mark.parametrize("mask", range(256))
def test_all_face_button_and_hat_combinations(mask):
  wheel = TurboG29.__new__(TurboG29)
  state = {"buttons": {"L2": 1}}
  wheel.apply_gamepad(state, mask)
  for bit, name in enumerate(("X", "S", "O", "T"), 4):
    assert state["buttons"][name] == bool(mask & (1 << bit))
  hat = mask & 0xf
  expected = {0: {"up"}, 1: {"up", "right"}, 2: {"right"}, 3: {"right", "down"},
              4: {"down"}, 5: {"down", "left"}, 6: {"left"}, 7: {"left", "up"}}.get(hat, set())
  assert {k for k in ("up", "down", "left", "right") if state["buttons"][k]} == expected
  assert state["buttons"]["L2"] == 1
  wheel.apply_gamepad(state, 8)
  assert not any(state["buttons"][k] for k in ("X", "S", "O", "T", "up", "down", "left", "right"))


def test_face_combinations_do_not_retrigger_the_remaining_held_button():
  wheel = TurboG29.__new__(TurboG29)
  previous, downs = {}, []
  state = {"buttons": {}}
  for mask in (8, 0x28, 0x68, 0x48, 0x40, 0x48, 8):
    wheel.apply_gamepad(state, mask)
    downs.append([k for k, v in state["buttons"].items() if v and not previous.get(k)])
    previous = state["buttons"].copy()
  assert downs == [[], ["S"], ["O"], [], ["up"], [], []]


@pytest.mark.parametrize("phase", range(5))
def test_new_request_and_cancel_publish_without_waiting_for_heartbeat(phase):
  schedule = IntentPublishSchedule()
  idle = {"action": "none", "operatorId": "gcs"}
  schedule.update(idle, True)
  request = {**idle, "requestId": 1, "action": "request", "maneuver": "turn", "direction": "left"}
  assert schedule.update(request, phase == 0)
  assert not schedule.update(request, False)
  assert schedule.update(request, True)
  assert schedule.update({**request, "action": "cancel"}, False)
  assert not schedule.update({**request, "action": "cancel"}, False)
