import math

import pytest

from openpilot.tools.turbo import g29d


class FakeG29:
  def __init__(self):
    self.anticenter_calls = []
    self.friction_calls = []

  def set_anticenter(self, **kwargs):
    self.anticenter_calls.append(kwargs)

  def set_friction(self, val):
    self.friction_calls.append(val)


def test_accelerator_to_simulated_velocity_maps_pedal_range():
  assert g29d._accelerator_to_simulated_velocity_m_s(-1.0, 20.0) == pytest.approx(0.0)
  assert g29d._accelerator_to_simulated_velocity_m_s(0.0, 20.0) == pytest.approx(10.0)
  assert g29d._accelerator_to_simulated_velocity_m_s(1.0, 20.0) == pytest.approx(20.0)


def test_accelerator_to_simulated_velocity_clips_input_and_output():
  assert g29d._accelerator_to_simulated_velocity_m_s(-2.0, 20.0) == pytest.approx(0.0)
  assert g29d._accelerator_to_simulated_velocity_m_s(2.0, 20.0) == pytest.approx(20.0)
  assert g29d._accelerator_to_simulated_velocity_m_s(1.0, -1.0) == pytest.approx(0.0)


def test_torque_controller_uses_script_force_response_default():
  fake = FakeG29()

  controller = g29d._make_torque_controller(fake)
  command = controller.update(longitudinal_velocity_m_s=8.0, steering=0.0)

  assert command.force_factor == pytest.approx(1.0 - math.exp(-0.96875))
  assert fake.anticenter_calls[-1]["force"] == pytest.approx(command.force)


def test_torque_controller_applies_g29py_class_directly():
  fake = FakeG29()
  controller = g29d._make_torque_controller(fake)

  controller.update(longitudinal_velocity_m_s=0.0, steering=0.0)
  controller.update(longitudinal_velocity_m_s=0.0, steering=0.0)

  assert len(fake.anticenter_calls) == 2
  assert len(fake.friction_calls) == 2
