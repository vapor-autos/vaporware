from openpilot.tools.turbo.g29d import SpeedSource
import pytest


class FakeCarState:
  def __init__(self, v_ego: float):
    self.vEgo = v_ego


class FakeSubMaster:
  def __init__(self, seen=False, valid=False, recv_time=0.0, v_ego=0.0):
    self.seen = {"carState": seen}
    self.valid = {"carState": valid}
    self.recv_time = {"carState": recv_time}
    self.data = {"carState": FakeCarState(v_ego)}
    self.update_count = 0

  def update(self, timeout):
    assert timeout == 0
    self.update_count += 1

  def __getitem__(self, service):
    return self.data[service]


def test_speed_source_uses_fresh_carstate_speed():
  sm = FakeSubMaster(seen=True, valid=True, recv_time=10.0, v_ego=-3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 1.0}, now=10.1)

  assert velocity == 3.5
  assert name == "carState"
  assert source.last_carstate_age_s == pytest.approx(0.1)
  assert sm.update_count == 1


def test_speed_source_falls_back_to_pedal_when_carstate_is_stale():
  sm = FakeSubMaster(seen=True, valid=True, recv_time=10.0, v_ego=3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 0.0}, now=10.5)

  assert velocity == 10.0
  assert name == "pedal"


def test_speed_source_falls_back_to_pedal_when_carstate_is_invalid():
  sm = FakeSubMaster(seen=True, valid=False, recv_time=10.0, v_ego=3.5)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": -1.0}, now=10.1)

  assert velocity == 0.0
  assert name == "pedal"


def test_speed_source_falls_back_to_pedal_before_carstate_seen():
  sm = FakeSubMaster(seen=False, valid=False)
  source = SpeedSource(sm=sm, stale_timeout_s=0.25)

  velocity, name = source.update({"accelerator": 1.0}, now=10.1)

  assert velocity == 20.0
  assert name == "pedal"
  assert source.last_carstate_age_s is None
