import pytest

from openpilot.selfdrive.ui.turbo_state import is_turbo_steer_override_active


class FakeMessage:
  def __init__(self, **values):
    self.__dict__.update(values)


class FakeSubMaster:
  def __init__(self):
    self.seen = {"selfdriveState": True, "turboSteerAssistState": True}
    self.alive = {"selfdriveState": True, "turboSteerAssistState": True}
    self.valid = {"selfdriveState": True, "turboSteerAssistState": True}
    self.data = {
      "selfdriveState": FakeMessage(enabled=True),
      "turboSteerAssistState": FakeMessage(applied=True),
    }

  def __getitem__(self, service):
    return self.data[service]


def test_turbo_steer_override_active_requires_authoritative_applied_state():
  assert is_turbo_steer_override_active(FakeSubMaster(), started=True)


@pytest.mark.parametrize(
  ("collection", "service"),
  [
    ("seen", "selfdriveState"),
    ("alive", "selfdriveState"),
    ("valid", "selfdriveState"),
    ("seen", "turboSteerAssistState"),
    ("alive", "turboSteerAssistState"),
    ("valid", "turboSteerAssistState"),
  ],
)
def test_turbo_steer_override_active_fails_closed_on_missing_feedback(collection, service):
  sm = FakeSubMaster()
  getattr(sm, collection)[service] = False

  assert not is_turbo_steer_override_active(sm, started=True)


def test_turbo_steer_override_active_requires_onroad_engagement():
  sm = FakeSubMaster()
  assert not is_turbo_steer_override_active(sm, started=False)

  sm.data["selfdriveState"].enabled = False
  assert not is_turbo_steer_override_active(sm, started=True)


def test_turbo_steer_override_active_clears_when_not_applied():
  sm = FakeSubMaster()
  sm.data["turboSteerAssistState"].applied = False

  assert not is_turbo_steer_override_active(sm, started=True)
