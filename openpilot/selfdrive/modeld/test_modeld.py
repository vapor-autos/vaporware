import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.modeld.modeld import get_action_from_model


def test_get_action_from_model_holds_lateral_action_at_low_speed_by_default():
  model_output = {"action": np.array([[0.1, 0.0]], dtype=np.float32)}
  previous = log.ModelDataV2.Action(desiredCurvature=0.25)

  action = get_action_from_model(model_output, previous, 0.1, 0.1, v_ego=0.0)

  assert action.desiredCurvature == pytest.approx(0.25)


def test_get_action_from_model_updates_lateral_action_when_steering_at_standstill():
  model_output = {"action": np.array([[0.1, 0.0]], dtype=np.float32)}
  previous = log.ModelDataV2.Action(desiredCurvature=0.25)

  action = get_action_from_model(model_output, previous, 0.1, 0.1, v_ego=0.0, steer_at_standstill=True)

  assert action.desiredCurvature == pytest.approx(0.1)
