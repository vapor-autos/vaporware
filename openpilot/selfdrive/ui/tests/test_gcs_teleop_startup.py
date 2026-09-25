import os
import subprocess
import sys

import pytest


# A fresh interpreter preserves the real startup import order. No cameras or
# live message subscriptions: only the launcher, fonts, widget and outlines render.
RENDER_SCRIPT = r'''
import os
import sys
import time
from enum import Enum
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pyray as rl

from openpilot.cereal import messaging
messaging.SubMaster = Mock(side_effect=AssertionError("render test must not subscribe"))
ui_state = ModuleType("openpilot.selfdrive.ui.ui_state")
ui_state.device = SimpleNamespace(awake=True)
ui_state.ui_state = SimpleNamespace()
ui_state.UIStatus = Enum("UIStatus", "DISENGAGED ENGAGED OVERRIDE")
sys.modules[ui_state.__name__] = ui_state

from openpilot.tools.turbo import gcs_teleop_ui
from openpilot.tools.turbo.gcs_window import MonitorGeometry
import openpilot.system.ui.lib.application as application

fake_gcs = ModuleType("openpilot.tools.turbo.gcs_ui")
rendered = []

def load_widget(name):
  if name != "GcsUi":
    raise AttributeError(name)
  from openpilot.system.ui.widgets import Widget, gui_app as widget_app
  from openpilot.system.ui.widgets.label import gui_app as label_app
  from openpilot.selfdrive.ui.turbo_intent import LANE_CHANGE_COLOR, TAKEOVER_COLOR, intent_border, draw_intent_border
  from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
  from openpilot.selfdrive.controls.lib.turbo_intent import STATE_SERVICE, REQUEST_SERVICE

  class BorderPreview(Widget):
    def _render(self, rect):
      services = (STATE_SERVICE, REQUEST_SERVICE)
      class SM(dict):
        pass
      sm = SM((s, getattr(messaging.new_message(s), s)) for s in services)
      sm.seen = dict.fromkeys(services, True)
      sm.valid = dict.fromkeys(services, True)
      sm.recv_time = dict.fromkeys(services, time.monotonic())
      scenario = ("left", "right", "completed", "override", "critical")[len(rendered)]
      sm[STATE_SERVICE].from_dict({"protocolVersion": 2, "status": "completed" if scenario == "completed" else "executing",
                                 "mode": "execute", "direction": "right" if scenario == "right" else "left"})
      sm[REQUEST_SERVICE].localStatus = "idle"
      assert label_app is widget_app is application.gui_app
      # Retain coverage for the original separate-application missing-font crash.
      assert label_app.font(application.FontWeight.MEDIUM).texture.id != 0
      visual = intent_border(sm, time.monotonic(), engaged=True, override=scenario == "override", critical=scenario == "critical")
      width, height = int(rect.width), int(rect.height)
      target = rl.load_render_texture(width, height)
      rl.begin_texture_mode(target)
      rl.clear_background(rl.Color(18, 24, 28, 255))
      base = rl.Color(218, 111, 37, 255) if scenario == "override" else rl.Color(22, 127, 64, 255)
      if visual.takeover:
        base = rl.Color(*TAKEOVER_COLOR)
      outline = rl.Rectangle(30, 30, width-60, height-60)
      owner = SimpleNamespace(camera_point_to_screen=lambda x, y: (x, y))
      # Exercise the actual projected model-box renderer, including corner arcs.
      points = [(width*.25, height*.3), (width*.75, height*.3),
                (width*.75, height*.7), (width*.25, height*.7)]
      with patch.object(rl, "draw_text_ex", side_effect=AssertionError("intent decoration must be text-free")):
        rl.draw_rectangle_rounded_lines_ex(outline, 0.12, 10, 20, base)
        draw_intent_border(outline, visual, 20)
        # Still clipped inside the camera: projected box drawing must preserve scissor state.
        rl.begin_scissor_mode(30, 30, width-60, height-60)
        AugmentedRoadView._draw_model_crop_poly(owner, points, base, visual.side)
        rl.end_scissor_mode()
      rl.end_texture_mode()
      image = rl.load_image_from_texture(target.texture)
      colors = rl.load_image_colors(image)
      rgba = np.frombuffer(rl.ffi.buffer(colors, width*height*4), dtype=np.uint8).reshape(height, width, 4)
      violet = np.all(rgba == LANE_CHANGE_COLOR, axis=2)
      if scenario in ("left", "right"):
        active = violet[:, :width//2] if scenario == "left" else violet[:, width//2:]
        inactive = violet[:, width//2:] if scenario == "left" else violet[:, :width//2]
        assert active.sum() > 500 and inactive.sum() == 0
        # Highlight reaches both the outer border and the inner model box.
        assert violet[:, :35].any() if scenario == "left" else violet[:, -35:].any()
        assert violet[height//4:3*height//4, width//5:4*width//5].any()
        # Never draw a vertical divider through the middle of the image.
        assert violet[height//4:3*height//4, width//2-1:width//2+1].sum() < 20
      else:
        assert not violet.any()
        assert np.all(rgba == (base.r, base.g, base.b, base.a), axis=2).sum() > 500
      rl.unload_image_colors(colors)
      output = os.getenv("INTENT_RENDER_OUTPUT_DIR")
      if output:
        rl.image_flip_vertical(image)
        assert rl.export_image(image, os.path.join(output, scenario + ".png"))
      rl.unload_image(image)
      rl.unload_render_texture(target)
      rendered.append(True)
      if len(rendered) == 5:
        application.gui_app.request_close()
  return BorderPreview

fake_gcs.__getattr__ = load_widget
monitor = MonitorGeometry(0, 0, 0, 800, 600) if sys.argv[1] == "monitor" else None
with patch.dict(sys.modules, {fake_gcs.__name__: fake_gcs}), \
     patch.object(gcs_teleop_ui, "monitor_geometry", return_value=monitor), \
     patch.object(gcs_teleop_ui, "place_window"):
  gcs_teleop_ui.main()
assert len(rendered) == 5
assert not messaging.SubMaster.called
'''


@pytest.mark.skipif(not os.getenv("DISPLAY"), reason="requires an X11 display (use Xvfb)")
@pytest.mark.parametrize("monitor", ["monitor", "fallback"])
def test_teleop_startup_renders_directional_border_without_text(monitor, tmp_path):
  env = dict(os.environ, SCALE="1", OFFSCREEN="1", LIBGL_ALWAYS_SOFTWARE="1", INTENT_RENDER_OUTPUT_DIR=str(tmp_path))
  result = subprocess.run([sys.executable, "-c", RENDER_SCRIPT, monitor], env=env,
                          capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stdout + result.stderr
