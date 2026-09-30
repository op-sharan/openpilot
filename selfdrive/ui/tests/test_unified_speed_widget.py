from types import SimpleNamespace
from dataclasses import replace

import pyray as rl
import pytest

from cereal import custom
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.starpilot import slc_speed_limit
from openpilot.selfdrive.ui.onroad.starpilot.unified_speed_presentation import UnifiedSpeedPresentation, resolve_unified_speed
from openpilot.selfdrive.ui.onroad.starpilot.widgets import unified_speed


def make_widget(mode="split", pending=False):
  widget = object.__new__(unified_speed.UnifiedSpeedWidget)
  widget._rect = rl.Rectangle(30, 75, 520, 250)
  widget._presentation = UnifiedSpeedPresentation(mode, "70", "65", "70", "+5", "mph", "Map Data", pending, "slc")
  widget._show_max = True
  widget._slc_state = None
  widget.hud_renderer = SimpleNamespace(is_cruise_set=True)
  return widget


@pytest.fixture
def header_icon_cache(monkeypatch):
  app = object.__new__(type(unified_speed.gui_app))
  app._scale = app._pixel_scale_x = app._pixel_scale_y = 1.0
  app._cached_render_textures = {}
  app._pending_render_textures = {}
  geometry, draws, allocations, scales = [], [], [], []
  monkeypatch.setattr(unified_speed, "gui_app", app)
  monkeypatch.setattr(unified_speed, "_draw_source_icon", lambda *args: geometry.append(args))
  monkeypatch.setattr(unified_speed, "measure_text_cached", lambda *args: rl.Vector2(100, 28))
  monkeypatch.setattr(rl, "draw_text_ex", lambda *args: None)
  monkeypatch.setattr(rl, "draw_texture_pro", lambda *args: draws.append(args))
  monkeypatch.setattr(rl, "rl_scalef", lambda *args: scales.append(args))
  for name in ("rl_push_matrix", "rl_pop_matrix", "begin_texture_mode", "end_texture_mode", "clear_background",
               "rl_set_blend_factors_separate", "begin_blend_mode", "end_blend_mode", "set_texture_filter", "set_texture_wrap"):
    monkeypatch.setattr(rl, name, lambda *args: None)

  def allocate(width, height):
    allocations.append((width, height))
    return SimpleNamespace(texture=SimpleNamespace(width=width, height=height))

  monkeypatch.setattr(rl, "load_render_texture", allocate)
  return app, geometry, draws, allocations, scales


def test_header_glyph_cache_is_shared_and_skips_geometry_after_first_frame(header_icon_cache):
  app, geometry, draws, allocations, _scales = header_icon_cache
  widgets = [make_widget(), make_widget()]
  for widget in widgets:
    widget._font_semi_bold = None
  widget = widgets[0]
  for label, icon in (("MAX SET", "speedometer"), ("SPEED LIMIT", "map")):
    widget._draw_header(widget.rect, label, icon, rl.WHITE)
  assert len(geometry) == 2
  assert allocations == []
  app._populate_render_texture_cache()
  assert len(geometry) == 4

  for frame in range(60):
    widget = widgets[frame % 2]
    bounds = rl.Rectangle(frame, frame, 260, 250)
    widget._draw_header(bounds, "MAX SET", "speedometer", rl.WHITE)
    widget._draw_header(bounds, f"LIMIT {frame}", "map", rl.GRAY)
  assert len(geometry) == 4
  assert len(draws) == 120
  assert len(allocations) == len(app._cached_render_textures) == 2
  assert app._pending_render_textures == {}


@pytest.mark.parametrize("scale,dpi,texture_size", [(0.5, 1.0, 68), (1.0, 2.0, 136), (1.25, 1.5, 128)])
def test_header_cache_resolution_preserves_logical_geometry(header_icon_cache, scale, dpi, texture_size):
  app, geometry, draws, allocations, scales = header_icon_cache
  app._scale, app._pixel_scale_x = scale, dpi
  for icon in ("speedometer", "map", "camera", "dashboard", "next"):
    unified_speed._draw_header_icon(icon, 10, 20)
  app._populate_render_texture_cache()
  assert len(app._cached_render_textures) == 5
  assert allocations == [(texture_size, texture_size)] * 5
  assert all(args[1:4] == (0, 0, 34) for args in geometry[5:])
  assert scales == [(texture_size / 34, texture_size / 34, 1.0)] * 5

  unified_speed._draw_header_icon("map", 200, 300)
  assert len(geometry) == 10
  source, destination = draws[-1][1:3]
  assert (source.width, source.height) == (texture_size, -texture_size)
  assert (destination.x, destination.y, destination.width, destination.height) == (200, 300, 34, 34)


def test_speed_limit_hit_target_is_right_half_in_both_layouts():
  for mode in ("split", "merged"):
    right = make_widget(mode)._speed_limit_bounds(rl.Rectangle(30, 75, 520, 250))
    assert (right.x, right.width) == (290, 260)


def test_confirmation_touch_only_accepts_on_speed_limit_side(monkeypatch):
  widget = make_widget(pending=True)
  writes = []
  monkeypatch.setattr(unified_speed, "Params", lambda memory: SimpleNamespace(put_bool=lambda key, value: writes.append((key, value))))
  widget._handle_mouse_press(rl.Vector2(100, 150))
  assert writes == []
  widget._handle_mouse_press(rl.Vector2(400, 150))
  assert writes == [("SpeedLimitAccepted", True)]


def test_merged_speed_limit_side_toggles_sources(monkeypatch):
  widget = make_widget("merged")
  writes = []
  params = SimpleNamespace(get_bool=lambda _key: False, put_bool=lambda key, value: writes.append((key, value)))
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(ui_params=params))
  widget._handle_mouse_press(rl.Vector2(100, 150))
  assert writes == []
  widget._handle_mouse_press(rl.Vector2(400, 150))
  assert writes == [("SpeedLimitSources", True)]


def test_diagnostic_sources_can_be_dismissed_from_max_only_card(monkeypatch):
  widget = make_widget("max_only")
  widget._slc_state = {}
  params = SimpleNamespace(get_bool=lambda _key: True, put_bool=lambda key, value: writes.append((key, value)))
  writes = []
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(ui_params=params))
  widget._handle_mouse_press(rl.Vector2(100, 150))
  assert writes == [("SpeedLimitSources", False)]


def test_right_border_overlay_is_clipped_to_speed_limit_side(monkeypatch):
  events = []
  monkeypatch.setattr(unified_speed.rl, "begin_scissor_mode", lambda *args: events.append(("begin", args)))
  monkeypatch.setattr(unified_speed.rl, "draw_rectangle_rounded_lines_ex", lambda *args: events.append(("outline", args)))
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: events.append(("divider", args)))
  monkeypatch.setattr(unified_speed.rl, "end_scissor_mode", lambda: events.append(("end",)))
  for mode, expected in (("split", ["begin", "outline", "end", "divider"]),
                         ("merged", ["begin", "outline", "end"])):
    events.clear()
    widget = make_widget(mode)
    rect = widget.rect
    right = widget._speed_limit_bounds(rect)
    widget._draw_speed_limit_border(rect, right, rl.Color(188, 132, 255, 200))
    assert events[0] == ("begin", (290, 75, 261, 251))
    assert [event[0] for event in events] == expected


def test_split_and_merged_draw_one_card_with_both_headers(monkeypatch):
  cards = []
  lines = []
  monkeypatch.setattr(unified_speed, "draw_control_card", lambda *args, **kwargs: cards.append(args[0]))
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.DISENGAGED,
                                                                 ui_params=SimpleNamespace(get_bool=lambda _key: False)))
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: lines.append(args))
  monkeypatch.setattr(unified_speed.rl, "draw_rectangle_rounded_lines_ex", lambda *args: None)
  for mode in ("split", "merged"):
    lines.clear()
    widget = make_widget(mode)
    headers = []
    values = []
    separators = []
    offsets = []
    monkeypatch.setattr(widget, "_draw_header", lambda _bounds, text, icon, _color, rows=headers: rows.append((text, icon)))
    monkeypatch.setattr(widget, "_draw_centered_text", lambda text, *args, rows=values, **kwargs: rows.append(text))
    monkeypatch.setattr(widget, "_draw_offset_pill", lambda bounds, text, y, rows=offsets: rows.append((bounds, text, y)))
    monkeypatch.setattr(widget, "_draw_merged_separator", lambda _rect, rows=separators: rows.append(True))
    monkeypatch.setattr(widget, "_draw_active_emphasis", lambda *args: None)
    widget._render(widget.rect)
    assert headers == [("MAX SET", "speedometer"), ("SPEED LIMIT", "map")]
    assert separators == ([True] if mode == "merged" else [])
    assert sum(line[0].x == line[1].x == 290 for line in lines) == (1 if mode == "split" else 0)
    assert values == (["70", "mph"] if mode == "merged" else ["70", "mph", "65", "mph"])
    assert offsets[0][0].x == 290
    assert offsets[0][2] == (136 if mode == "merged" else 250)
  assert len(cards) == 2


def test_merged_draws_effective_speed_once_and_skips_active_line(monkeypatch):
  widget = make_widget("merged")
  widget._presentation = replace(widget._presentation, max_speed_text="71", effective_speed_text="70", active_side="shared")
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.ENGAGED,
                                                                 ui_params=SimpleNamespace(get_bool=lambda _key: False)))
  monkeypatch.setattr(unified_speed, "draw_control_card", lambda *args, **kwargs: None)
  monkeypatch.setattr(unified_speed.rl, "draw_rectangle_rounded_lines_ex", lambda *args: None)
  lines = []
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: lines.append(args))
  monkeypatch.setattr(widget, "_draw_merged_separator", lambda _rect: None)
  monkeypatch.setattr(widget, "_draw_header", lambda *args: None)
  monkeypatch.setattr(widget, "_draw_offset_pill", lambda *args: None)
  values = []
  monkeypatch.setattr(widget, "_draw_centered_text", lambda text, *args, **kwargs: values.append(text))
  widget._render(widget.rect)
  assert values == ["70", "mph"]
  assert lines == []


def test_merged_separator_has_shallow_center_dip(monkeypatch):
  widget = make_widget("merged")
  segments = []
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: segments.append(("line", args)))
  monkeypatch.setattr(unified_speed.rl, "draw_spline_segment_bezier_cubic", lambda *args: segments.append(("curve", args)))
  widget._draw_merged_separator(widget.rect)
  assert [segment[0] for segment in segments] == ["line", "curve", "line", "curve", "line"]
  assert segments[0][1][0].y == widget.rect.y + 76
  assert segments[2][1][0].y == widget.rect.y + 88


def test_enabled_slc_stays_full_width_when_plan_is_stale(monkeypatch):
  widget = make_widget("split")
  widget._snapshot_frame = None
  widget.hud_renderer = SimpleNamespace(is_cruise_available=True, is_cruise_set=True, set_speed=70)
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(
    sm=SimpleNamespace(frame=1), starpilot_toggles={}, is_metric=False,
  ))
  monkeypatch.setattr(unified_speed, "_is_slc_enabled", lambda: True)
  monkeypatch.setattr(unified_speed, "_get_slc_state", lambda: None)
  assert widget.get_size() == (520.0, 250.0)
  assert widget.is_visible
  assert widget._presentation.posted_speed_text == "–"


def test_split_merged_transitions_keep_the_same_footprint(monkeypatch):
  widget = make_widget("split")
  monkeypatch.setattr(widget, "_refresh_snapshot", lambda: None)
  sizes = []
  for mode in ("split", "merged", "split", "merged"):
    widget._presentation = replace(widget._presentation, mode=mode)
    sizes.append(widget.get_size())
  assert sizes == [(520.0, 250.0)] * 4


@pytest.fixture
def slc_ui(monkeypatch):
  class Params(dict):
    def get_bool(self, key):
      return bool(self.get(key))

    def get(self, key, encoding=None):
      return super().get(key)

  class SubMaster(dict):
    recv_frame = {"starpilotPlan": 10}
    valid = {"starpilotCarState": True}

  plan = custom.StarPilotPlan.new_message(
    slcSpeedLimit=30 * CV.MPH_TO_MS, slcSpeedLimitOffset=0.0, slcSpeedLimitSource="Map Data",
    slcOverriddenSpeed=0.0, slcMapSpeedLimit=30 * CV.MPH_TO_MS, slcMapboxSpeedLimit=0.0,
    slcNextSpeedLimit=0.0, unconfirmedSlcSpeedLimit=0.0, speedLimitChanged=False,
  )
  sm = SubMaster(starpilotPlan=plan, starpilotCarState=SimpleNamespace(dashboardSpeedLimit=0.0))
  sm.recv_frame = sm.recv_frame.copy()
  params = Params(SpeedLimitController=True, ShowSpeedLimits=False)
  ui = SimpleNamespace(
    sm=sm, started_frame=10, is_metric=False, ui_params=params, starpilot_toggles={},
    params_memory=SimpleNamespace(get_float=lambda _key: 0.0),
  )
  monkeypatch.setattr(slc_speed_limit, "ui_state", ui)
  monkeypatch.setattr(slc_speed_limit, "starpilot_state", SimpleNamespace(car_state=SimpleNamespace(hasDashSpeedLimits=True)))
  monkeypatch.setattr(slc_speed_limit, "_tick_pulse", lambda *args: None)
  return ui


def test_slc_state_extraction_respects_feature_and_display_toggles(slc_ui):
  assert slc_speed_limit._is_slc_enabled()
  assert slc_speed_limit._get_slc_state()["slc_enabled"]
  slc_ui.starpilot_toggles["speed_limit_controller"] = False
  assert not slc_speed_limit._is_slc_enabled()
  assert slc_speed_limit._get_slc_state() is None
  slc_ui.ui_params["ShowSpeedLimits"] = True
  assert not slc_speed_limit._get_slc_state()["slc_enabled"]
  slc_ui.starpilot_toggles["speed_limit_controller"] = True
  slc_ui.sm.recv_frame["starpilotPlan"] = 9
  assert slc_speed_limit._get_slc_state() is None


@pytest.mark.parametrize("presented_source,expected_source,expected_speed", [
  ("", "Map Data", "30"),
  ("Map Data", "Map Data", "30"),
  ("None", "None", "–"),
  ("Previous Limit", "Previous Limit", "30"),
  ("Vision", "Vision", "30"),
])
def test_serialized_plan_source_defaults_and_explicit_values(slc_ui, presented_source, expected_source, expected_speed):
  message = slc_ui.sm["starpilotPlan"]
  if presented_source:
    message.slcPresentedSpeedLimitSource = presented_source
  # Replay decodes older plans with a present but empty Text attribute.
  with custom.StarPilotPlan.from_bytes(message.to_bytes()) as plan:
    slc_ui.sm["starpilotPlan"] = plan
    state = slc_speed_limit._get_slc_state()
    result = resolve_unified_speed(True, True, 35, state, True, False)
    assert result.source == expected_source
    assert result.posted_speed_text == expected_speed
    assert result.mode == "split"


def test_legacy_replay_limit_and_offset_merge_with_max_set(slc_ui):
  message = slc_ui.sm["starpilotPlan"]
  message.slcSpeedLimitOffset = 5 * CV.MPH_TO_MS
  with custom.StarPilotPlan.from_bytes(message.to_bytes()) as plan:
    slc_ui.sm["starpilotPlan"] = plan
    result = resolve_unified_speed(True, True, 35, slc_speed_limit._get_slc_state(), True, False)
    assert (result.source, result.posted_speed_text, result.effective_speed_text) == ("Map Data", "30", "35")
    assert (result.mode, result.offset_text) == ("merged", "+5")


@pytest.mark.parametrize("presented_source,limiting,max_speed,enabled,overridden,expected_side,line_x", [
  ("", False, 40, True, False, "slc", 308),
  ("", False, 34, True, False, "max", 48),
  ("", False, 35, True, False, "shared", None),
  ("", False, 40, False, False, "max", 48),
  ("", False, 40, True, True, "none", None),
  ("Map Data", False, 40, True, False, "max", 48),
  ("Map Data", True, 40, True, False, "slc", 308),
])
def test_active_underline_with_legacy_and_current_plans(slc_ui, monkeypatch, presented_source, limiting,
                                                       max_speed, enabled, overridden, expected_side, line_x):
  message = slc_ui.sm["starpilotPlan"]
  message.slcSpeedLimitOffset = 5 * CV.MPH_TO_MS
  message.slcPresentedSpeedLimitSource = presented_source
  message.slcIsLimitingMaxSet = limiting
  message.slcOverriddenSpeed = 40 * CV.MPH_TO_MS if overridden else 0.0
  slc_ui.starpilot_toggles["speed_limit_controller"] = enabled
  slc_ui.ui_params["ShowSpeedLimits"] = True
  with custom.StarPilotPlan.from_bytes(message.to_bytes()) as plan:
    slc_ui.sm["starpilotPlan"] = plan
    presentation = resolve_unified_speed(True, True, max_speed, slc_speed_limit._get_slc_state(), enabled, False)
  assert presentation.active_side == expected_side

  widget = make_widget(presentation.mode)
  widget._presentation = presentation
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.ENGAGED))
  lines = []
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: lines.append(args))
  widget._draw_active_emphasis(widget.rect)
  if line_x is None:
    assert lines == []
  else:
    assert len(lines) == 1
    assert (lines[0][0].x, lines[0][0].y, lines[0][1].x) == (line_x, 140, line_x + 224)
    assert lines[0][3] == unified_speed.UNIFIED_ACCENT


def test_legacy_plan_without_active_source_does_not_use_diagnostic_map_limit(slc_ui):
  message = slc_ui.sm["starpilotPlan"]
  message.slcSpeedLimitSource = "None"
  with custom.StarPilotPlan.from_bytes(message.to_bytes()) as plan:
    slc_ui.sm["starpilotPlan"] = plan
    state = slc_speed_limit._get_slc_state()
    assert round(state["map_sl"]) == 30
    result = resolve_unified_speed(True, True, 35, state, True, False)
    assert (result.source, result.posted_speed_text, result.mode) == ("None", "–", "split")


def test_legacy_pending_candidate_remains_visible_without_active_source(slc_ui):
  message = slc_ui.sm["starpilotPlan"]
  message.slcSpeedLimitSource = "None"
  message.unconfirmedSlcSpeedLimit = 45 * CV.MPH_TO_MS
  message.speedLimitChanged = True
  with custom.StarPilotPlan.from_bytes(message.to_bytes()) as plan:
    slc_ui.sm["starpilotPlan"] = plan
    result = resolve_unified_speed(True, True, 35, slc_speed_limit._get_slc_state(), True, False)
    assert (result.posted_speed_text, result.mode, result.confirmation_pending) == ("45", "split", True)


def test_header_colors_preserve_engaged_disengaged_and_override_semantics(monkeypatch):
  widget = make_widget()
  colors = unified_speed.COLORS
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.ENGAGED))
  assert widget._max_header_color("max", True) == colors.ENGAGED
  assert widget._max_header_color("slc", True) == colors.GREY
  assert widget._limit_header_color("slc", False) == colors.ENGAGED

  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.DISENGAGED))
  assert widget._max_header_color("max", True) == colors.DISENGAGED
  assert widget._limit_header_color("slc", False) == colors.DISENGAGED

  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.OVERRIDE))
  assert widget._max_header_color("max", True) == colors.DISENGAGED
  assert widget._limit_header_color("slc", False) == colors.DISENGAGED

  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.ENGAGED))
  assert widget._limit_header_color("none", True) == colors.DISENGAGED
