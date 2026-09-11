from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper


@pytest.mark.parametrize("brand", ["turbo", "tesla", "toyota"])
@pytest.mark.parametrize("direction", [log.LaneChangeDirection.left, log.LaneChangeDirection.right])
def test_lane_change_metadata_never_controls_turbo_headlights(mocker, brand, direction):
  # Actual controlsd branch with fake dependencies, no sockets or control processes.
  c = Controls.__new__(Controls)
  c.CP = car.CarParams.new_message()
  c.CP.brand = brand
  c.CP.steerControlType = car.CarParams.SteerControlType.angle
  c.CP.lateralTuning.init("pid")
  data = {s: getattr(messaging.new_message(s), s) for s in (
    "carState", "liveParameters", "longitudinalPlan", "modelV2", "selfdriveState", "liveDelay",
  )}
  data["carState"].vEgo = 3.0
  data["selfdriveState"].enabled = data["selfdriveState"].active = True
  data["modelV2"].meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  data["modelV2"].meta.laneChangeDirection = direction

  class SM(dict):
    valid = {"lateralManeuverPlan": False}

  c.sm = SM(data, onroadEvents=[])
  c.VM = mocker.Mock()
  c.VM.calc_curvature.return_value = 0.0
  c.CI = mocker.Mock()
  c.LoC = SimpleNamespace(long_control_state=car.CarControl.Actuators.LongControlState.off,
                         reset=mocker.Mock(), update=mocker.Mock(return_value=0.0))
  c.LaC = mocker.Mock()
  c.LaC.update.return_value = (0.0, 12.5, None)
  c.desired_curvature = 0.0
  c.max_curvature = 0.45
  c.steer_limited_by_safety = False
  c.turbo_steer_assist_source = c.turbo_steer_assist_applicator = None
  cc, _ = c.state_control()
  assert cc.actuators.steeringAngleDeg == 12.5  # Non-Turbo angle path still works without an override applicator.
  assert cc.leftBlinker == (brand != "turbo" and direction == log.LaneChangeDirection.left)
  assert cc.rightBlinker == (brand != "turbo" and direction == log.LaneChangeDirection.right)


def test_stock_helper_still_requires_speed_blinker_and_torque():
  cs = car.CarState.new_message()
  cs.vEgo = 3.0
  cs.leftBlinker = True
  helper = DesireHelper()
  helper.update(cs, True, 0.5)
  assert helper.desire == log.Desire.none
  cs.leftBlinker = False
  helper.update(cs, True, 0.5)
  cs.leftBlinker = True
  cs.vEgo = 10.0
  helper.update(cs, True, 0.5)
  assert helper.lane_change_state == log.LaneChangeState.preLaneChange
  cs.steeringPressed = True
  cs.steeringTorque = 1.0
  cs.leftBlindspot = True
  helper.update(cs, True, 0.5)
  assert helper.desire == log.Desire.none
  cs.leftBlindspot = False
  helper.update(cs, True, 0.5)
  assert helper.desire == log.Desire.laneChangeLeft
