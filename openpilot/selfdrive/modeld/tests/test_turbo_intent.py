import numpy as np
import pytest

from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.lib.turbo_intent import IntentConfig, REQUEST_SERVICE, STATE_SERVICE, LINK_SERVICE
from openpilot.selfdrive.modeld.turbo_intent import TurboIntentRuntime, INTENT_SUBSCRIPTIONS
from openpilot.selfdrive.ui.turbo_intent import intent_label
from openpilot.tools.turbo.intent import PaddleIntentController


class FakeSubMaster:
  def __init__(self):
    services = ["carState", "carControl", "liveCalibration", STATE_SERVICE, *INTENT_SUBSCRIPTIONS]
    self.data = {s: getattr(messaging.new_message(s), s) for s in services}
    self.seen = dict.fromkeys(services, True)
    self.valid = dict.fromkeys(services, True)
    self.updated = dict.fromkeys(services, False)
    self.recv_time = dict.fromkeys(services, 10.0)
    self["carState"].canValid = True
    self["carState"].vEgo = 3.0
    self["carControl"].latActive = True
    self["liveCalibration"].calStatus = log.LiveCalibrationData.Status.calibrated
    self["selfdriveState"].state = "enabled"
    self["g29"].reverse = -1.0
    self["turboSteerAssist"].baseModelLogMonoTime = 10_000_000_000
    self[LINK_SERVICE].sessionId = "session"
    self[LINK_SERVICE].connected = True

  def __getitem__(self, key):
    return self.data[key]

  def advance(self, now):
    self.recv_time = dict.fromkeys(self.data, now)
    self["turboSteerAssist"].baseModelLogMonoTime = int(now * 1e9)


class FakePubMaster:
  def __init__(self):
    self.sent = []

  def send(self, service, message):
    self.sent.append((service, message))


def ready_runtime(mode="execute"):
  sm = FakeSubMaster()
  pm = FakePubMaster()
  runtime = TurboIntentRuntime(pm, IntentConfig(mode=mode))
  runtime.before_inference(sm, 10.0)
  sm.advance(10.6)
  runtime.before_inference(sm, 10.6)
  return runtime, sm, pm


def send_request(runtime, sm, now=10.6):
  state = runtime.manager.snapshot(now)
  c = PaddleIntentController()
  c.update({}, [], state, int(now * 1e9), True, True, False, now)
  request = c.update({"left_paddle": True}, [{"type": "button_down", "control": "left_paddle"}],
                     state, int(now * 1e9), True, True, False, now)
  sm[REQUEST_SERVICE].from_dict(request)
  sm.updated[REQUEST_SERVICE] = True
  return request


@pytest.mark.parametrize("mode", ["shadow", "execute"])
def test_gcs_to_runtime_to_ui_without_hardware(mode):
  runtime, sm, pm = ready_runtime(mode)
  request = send_request(runtime, sm)
  desire = runtime.before_inference(sm, 10.61)
  state = pm.sent[-1][1].turboIntentState
  assert state.requestId == request["requestId"] and state.operatorId == request["operatorId"]
  if mode == "execute":
    assert desire == log.Desire.laneChangeLeft
    assert state.status == "awaitingEvaluation" and state.pulseMonoTime == 0
    runtime.after_inference(desire, 0.4, 123, 10.62)
    state = pm.sent[-1][1].turboIntentState
    assert state.status == "executing" and state.pulseFrameId == 123
  else:
    assert desire == log.Desire.none and state.status == "shadow"
  sm.data[STATE_SERVICE] = state
  assert f"LEFT LANE CHANGE: {'EXECUTING' if mode == 'execute' else 'SHADOW'}" in intent_label(sm, 10.62)


@pytest.mark.parametrize("service", ["carState", "carControl", "liveCalibration", "selfdriveState", "g29",
                                     "turboSteerAssist", "turboSteerAssistState", LINK_SERVICE])
def test_stale_or_missing_health_never_emits_pulse(service):
  runtime, sm, _ = ready_runtime()
  send_request(runtime, sm)
  sm.recv_time[service] = 5.0
  assert runtime.before_inference(sm, 10.61) == log.Desire.none
  assert runtime.manager.status == "rejected"


@pytest.mark.parametrize("service,field,value", [
  ("carState", "canValid", False), ("carState", "standstill", True),
  ("carState", "steerFaultTemporary", True), ("carState", "steerFaultPermanent", True),
  ("carState", "leftBlindspot", True), ("carState", "rightBlindspot", True),
  ("selfdriveState", "state", "softDisabling"), ("g29", "reverse", 0.0),
  ("turboSteerAssist", "active", True), ("turboSteerAssistState", "applied", True),
  ("turboSteerAssist", "baseModelLogMonoTime", 0), (LINK_SERVICE, "connected", False),
])
def test_local_health_gates_are_authoritative(service, field, value):
  runtime, sm, _ = ready_runtime()
  send_request(runtime, sm)
  setattr(sm[service], field, value)
  assert runtime.before_inference(sm, 10.61) == log.Desire.none
  assert runtime.manager.status == "rejected"


def test_maneuver_debug_mode_conflicts_with_lane_change():
  runtime, sm, _ = ready_runtime()
  send_request(runtime, sm)
  assert runtime.before_inference(sm, 10.61, maneuver_mode=True) == log.Desire.none


def test_stale_control_does_not_falsely_claim_active_maneuver_stopped():
  runtime, sm, _ = ready_runtime()
  send_request(runtime, sm)
  desire = runtime.before_inference(sm, 10.61)
  runtime.after_inference(desire, 0.5, 1, 10.62)
  sm.advance(10.9)
  sm.recv_time["carControl"] = 10.6
  runtime.before_inference(sm, 10.9)
  assert runtime.manager.status == "executing"
  assert runtime.manager.reason == "vehicle_unhealthy"


def test_ui_stale_feedback_and_local_unknown():
  _, sm, _ = ready_runtime()
  sm[STATE_SERVICE].status = "idle"
  sm[STATE_SERVICE].mode = "shadow"
  assert intent_label(sm, 10.6) == "PADDLES: SHADOW"
  sm[REQUEST_SERVICE].localStatus = "unknown"
  assert intent_label(sm, 10.6) == "LANE CHANGE: STATUS UNKNOWN"
  assert intent_label(sm, 11.0) == "LANE CHANGE: LINK STALE"
  sm.seen[STATE_SERVICE] = False
  assert intent_label(sm, 11.0) == ""


def test_standstill_is_an_explicit_rejection_not_vehicle_fault():
  runtime, sm, pm = ready_runtime()
  send_request(runtime, sm)
  sm["carState"].standstill = True
  sm["carState"].vEgo = 0.0
  assert runtime.before_inference(sm, 10.61) == log.Desire.none
  assert pm.sent[-1][1].turboIntentState.reason == "standstill"


def test_model_run_skips_do_not_consume_desire_edge(mocker):
  from openpilot.selfdrive.modeld.modeld import ModelState, WARP_INPUTS

  # Exercise the actual run method with no camera, GPU initialization or weights.
  model = ModelState.__new__(ModelState)
  model.prev_desire = np.zeros(8)
  model.npy = {"desire": np.zeros(8), "traffic_convention": np.zeros(2), "action_t": np.zeros(1),
               "tfm": np.zeros((3, 3)), "big_tfm": np.zeros((3, 3)), "prev_feat": np.zeros(1)}
  model.full_frames = {"img": None, "big_img": None}
  model.input_queues = dict.fromkeys(WARP_INPUTS)
  model.warp_enqueue = mocker.Mock(return_value=(None, None))
  observed = []
  output = mocker.Mock()
  output.numpy.return_value = np.zeros((1, 1))
  model.run_policy = lambda **kwargs: (observed.append(model.npy["desire"].copy()) or output,)
  model.output_slices = {"hidden_state": slice(0, 1)}
  model.parser = mocker.Mock()
  model.parser.parse_outputs.return_value = {}
  inputs = {"desire_pulse": np.eye(8)[log.Desire.laneChangeLeft], "traffic_convention": np.zeros(2), "action_t": np.zeros(1)}
  transforms = {"img": np.eye(3), "big_img": np.eye(3)}
  for _ in range(3):
    assert model.run({}, transforms, inputs, prepare_only=True) is None
    assert not model.prev_desire.any()
  model.run({}, transforms, inputs, prepare_only=False)
  model.run({}, transforms, inputs, prepare_only=False)
  assert observed[0][log.Desire.laneChangeLeft] == 1
  assert not observed[1].any()
