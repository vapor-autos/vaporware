from msgq.visionipc import VisionStreamType

import pytest

from openpilot.selfdrive.ui.gcs_camera import GCS_STANDARD_UI_CAMERA_ENV, gcs_standard_ui_camera


class FakeParams:
  def __init__(self, gcs: bool):
    self.gcs = gcs

  def get_bool(self, key: str) -> bool:
    assert key == "GCS"
    return self.gcs


def test_gcs_standard_ui_camera_defaults_to_unlocked_road(monkeypatch):
  monkeypatch.delenv(GCS_STANDARD_UI_CAMERA_ENV, raising=False)

  stream, lock_stream = gcs_standard_ui_camera(params=FakeParams(gcs=True))

  assert stream == VisionStreamType.VISION_STREAM_ROAD
  assert not lock_stream


def test_gcs_standard_ui_camera_ignores_env_when_not_gcs(monkeypatch):
  monkeypatch.setenv(GCS_STANDARD_UI_CAMERA_ENV, "wideRoad")

  stream, lock_stream = gcs_standard_ui_camera(params=FakeParams(gcs=False))

  assert stream == VisionStreamType.VISION_STREAM_ROAD
  assert not lock_stream


def test_gcs_standard_ui_camera_selects_and_locks_wide_road(monkeypatch):
  monkeypatch.setenv(GCS_STANDARD_UI_CAMERA_ENV, "wideRoad")

  stream, lock_stream = gcs_standard_ui_camera(params=FakeParams(gcs=True))

  assert stream == VisionStreamType.VISION_STREAM_WIDE_ROAD
  assert lock_stream


def test_gcs_standard_ui_camera_rejects_unknown_camera(monkeypatch):
  monkeypatch.setenv(GCS_STANDARD_UI_CAMERA_ENV, "driver")

  with pytest.raises(ValueError, match=f"invalid {GCS_STANDARD_UI_CAMERA_ENV}"):
    gcs_standard_ui_camera(params=FakeParams(gcs=True))
