from types import SimpleNamespace

import pyray as rl

from openpilot.selfdrive.ui.onroad.starpilot.unified_speed_presentation import UnifiedSpeedPresentation
from openpilot.selfdrive.ui.onroad.starpilot.widgets import unified_speed


def make_widget(mode="split", pending=False):
  widget = object.__new__(unified_speed.UnifiedSpeedWidget)
  widget._rect = rl.Rectangle(30, 75, 520, 250)
  widget._presentation = UnifiedSpeedPresentation(mode, "70", "65", "70", "+5", "mph", "Map Data", pending, "slc")
  widget._show_max = True
  widget._slc_state = None
  widget.hud_renderer = SimpleNamespace(is_cruise_set=True)
  return widget


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


def test_diagnostic_sources_can_be_dismissed_from_max_only_card(monkeypatch):
  widget = make_widget("max_only")
  widget._slc_state = {}
  params = SimpleNamespace(get_bool=lambda _key: True, put_bool=lambda key, value: writes.append((key, value)))
  writes = []
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(ui_params=params))
  widget._handle_mouse_press(rl.Vector2(100, 150))
  assert writes == [("SpeedLimitSources", False)]


def test_right_border_overlay_is_clipped_to_speed_limit_side(monkeypatch):
  widget = make_widget()
  events = []
  monkeypatch.setattr(unified_speed.rl, "begin_scissor_mode", lambda *args: events.append(("begin", args)))
  monkeypatch.setattr(unified_speed.rl, "draw_rectangle_rounded_lines_ex", lambda *args: events.append(("outline", args)))
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: events.append(("divider", args)))
  monkeypatch.setattr(unified_speed.rl, "end_scissor_mode", lambda: events.append(("end",)))
  rect = widget.rect
  right = widget._speed_limit_bounds(rect)
  widget._draw_speed_limit_border(rect, right, rl.Color(188, 132, 255, 200))
  assert events[0] == ("begin", (290, 75, 261, 251))
  assert [event[0] for event in events] == ["begin", "outline", "end", "divider"]


def test_split_and_merged_draw_one_card_with_both_headers(monkeypatch):
  cards = []
  monkeypatch.setattr(unified_speed, "draw_control_card", lambda *args, **kwargs: cards.append(args[0]))
  monkeypatch.setattr(unified_speed, "ui_state", SimpleNamespace(status=unified_speed.UIStatus.DISENGAGED,
                                                                 ui_params=SimpleNamespace(get_bool=lambda _key: False)))
  monkeypatch.setattr(unified_speed.rl, "draw_line_ex", lambda *args: None)
  for mode in ("split", "merged"):
    widget = make_widget(mode)
    headers = []
    monkeypatch.setattr(widget, "_draw_header", lambda _bounds, text, icon, _color, rows=headers: rows.append((text, icon)))
    monkeypatch.setattr(widget, "_draw_centered_text", lambda *args, **kwargs: None)
    monkeypatch.setattr(widget, "_draw_offset_pill", lambda *args: None)
    monkeypatch.setattr(widget, "_draw_active_emphasis", lambda *args: None)
    widget._render(widget.rect)
    assert headers == [("MAX SET", "dashboard"), ("SPEED LIMIT", "map")]
  assert len(cards) == 2


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
