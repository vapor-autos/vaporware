import os
import subprocess
import sys


def test_teleop_monitor_renders_only_video_and_inset():
  # Isolated imports: do not construct UIState's live SubMaster during this test.
  script = r'''
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.cereal import messaging
messaging.SubMaster = Mock(side_effect=AssertionError("no live subscriptions"))
state = ModuleType("openpilot.selfdrive.ui.ui_state")
state.device = SimpleNamespace(awake=True)
state.ui_state = SimpleNamespace()
sys.modules[state.__name__] = state

from openpilot.tools.turbo import gcs_ui

wide, inset = Mock(), Mock()
with patch.object(gcs_ui.messaging, "SubMaster") as sub, \
     patch.object(gcs_ui, "CameraView", side_effect=[wide, inset]), \
     patch.object(gcs_ui.rl, "draw_rectangle_rounded_lines_ex", side_effect=AssertionError("no teleop borders")), \
     patch.object(gcs_ui.rl, "draw_rectangle_lines_ex", side_effect=AssertionError("no teleop borders")), \
     patch.object(gcs_ui.rl, "draw_text_ex", side_effect=AssertionError("no teleop intent text")):
  view = gcs_ui.GcsUi()
  sub.assert_called_once_with(["g29"])
  rect = gcs_ui.rl.Rectangle(0, 0, 1920, 1080)
  view._render(rect)
  wide.render.assert_called_once_with(rect)
  inset.render.assert_called_once()
  overlay = inset.render.call_args.args[0]
  assert overlay.width == 482 and overlay.height == 302
  assert overlay.x+overlay.width == 1900 and overlay.y+overlay.height == 1060
'''
  result = subprocess.run([sys.executable, "-c", script], env=dict(os.environ, SCALE="1"),
                          capture_output=True, text=True, timeout=20)
  assert result.returncode == 0, result.stdout + result.stderr
