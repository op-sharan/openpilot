"""One Big UI card for Max Set and the accepted speed limit."""

from __future__ import annotations

import math

import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.hud_renderer import COLORS
from openpilot.selfdrive.ui.onroad.starpilot.slc_speed_limit import (
  _draw_source_icon, _draw_sources_bubble, _get_slc_state, _is_slc_enabled, _speed_limit_pulse_color, source_icon_key,
)
from openpilot.selfdrive.ui.onroad.starpilot.unified_speed_presentation import (
  UnifiedSpeedPresentation, resolve_unified_speed,
)
from openpilot.selfdrive.ui.onroad.starpilot.widget_style import (
  CONTROL_BG, CONTROL_ROUNDNESS, CONTROL_SEGMENTS, draw_control_card, roundness_for,
)
from openpilot.selfdrive.ui.onroad.starpilot.widgets.base import LayoutWidget
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached


UNIFIED_WIDTH = 520
UNIFIED_HEIGHT = 250
SINGLE_WIDTH = 250
MERGED_SEPARATOR_Y = 76
HEADER_ICON_SIZE = 34
HEADER_FONT_SIZE = 28
VALUE_FONT_SIZE = 96
UNIT_FONT_SIZE = 28
OFFSET_FONT_SIZE = 22
OFFSET_PILL_HEIGHT = 30
CONFIRMATION_COLOR = rl.Color(188, 132, 255, 255)
UNIFIED_ACCENT = rl.Color(160, 96, 230, 230)
OFFSET_COLOR = rl.Color(UNIFIED_ACCENT.r, UNIFIED_ACCENT.g, UNIFIED_ACCENT.b, 255)


class UnifiedSpeedWidget(LayoutWidget):
  TOUCH_SLOP = 20

  def __init__(self, hud_renderer):
    super().__init__("unified_speed", priority=1)
    self.hud_renderer = hud_renderer
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._slc_state: dict | None = None
    self._slc_enabled = False
    self._presentation: UnifiedSpeedPresentation | None = None
    self._show_max = False
    self._snapshot_frame: int | None = None

  def _refresh_snapshot(self) -> None:
    frame = getattr(ui_state.sm, "frame", None)
    if frame is not None and frame == self._snapshot_frame:
      return
    self._snapshot_frame = frame
    self._slc_enabled = _is_slc_enabled()
    self._slc_state = _get_slc_state()
    self._show_max = (
      self.hud_renderer.is_cruise_available and
      not ui_state.starpilot_toggles.get("hide_max_speed", False)
    )
    self._presentation = resolve_unified_speed(
      self._show_max, self.hud_renderer.is_cruise_set, self.hud_renderer.set_speed,
      self._slc_state, self._slc_enabled, ui_state.is_metric,
    )

  @property
  def is_visible(self) -> bool:
    self._refresh_snapshot()
    return self._show_max or self._presentation.mode != "max_only"

  def get_size(self) -> tuple[float, float]:
    self._refresh_snapshot()
    width = UNIFIED_WIDTH if self._presentation.mode in ("split", "merged") else SINGLE_WIDTH
    return float(width), float(UNIFIED_HEIGHT)

  @property
  def _hit_rect(self) -> rl.Rectangle:
    rect = self.rect
    return rl.Rectangle(
      rect.x, rect.y - self.TOUCH_SLOP,
      rect.width + self.TOUCH_SLOP, rect.height + 2 * self.TOUCH_SLOP,
    )

  def _speed_limit_bounds(self, rect: rl.Rectangle) -> rl.Rectangle | None:
    mode = self._presentation.mode
    if mode in ("split", "merged"):
      return rl.Rectangle(rect.x + rect.width / 2, rect.y, rect.width / 2, rect.height)
    if mode == "limit_only":
      return rect
    return None

  def _draw_centered_text(self, text: str, bounds: rl.Rectangle, y: float,
                          font_size: int, color: rl.Color, *, bold: bool = False) -> None:
    font = self._font_bold if bold else self._font_semi_bold
    text_size = measure_text_cached(font, text, font_size)
    text_x = bounds.x + (bounds.width - text_size.x) / 2
    rl.draw_text_ex(font, text, rl.Vector2(text_x, y), font_size, 0, color)

  def _draw_header(self, bounds: rl.Rectangle, text: str, icon_key: str | None, label_color: rl.Color) -> None:
    text = tr(text)
    font_size = HEADER_FONT_SIZE
    icon_width = HEADER_ICON_SIZE + 9 if icon_key else 0
    while font_size > 16 and measure_text_cached(self._font_semi_bold, text, font_size).x + icon_width > bounds.width - 24:
      font_size -= 1
    text_size = measure_text_cached(self._font_semi_bold, text, font_size)
    group_width = icon_width + text_size.x
    group_x = bounds.x + (bounds.width - group_width) / 2
    icon_y = bounds.y + 20
    if icon_key:
      _draw_source_icon(icon_key, group_x, icon_y, HEADER_ICON_SIZE, rl.WHITE)
    rl.draw_text_ex(
      self._font_semi_bold, text,
      rl.Vector2(group_x + icon_width, icon_y + (HEADER_ICON_SIZE - text_size.y) / 2),
      font_size, 0, label_color,
    )

  def _draw_offset_pill(self, bounds: rl.Rectangle, text: str, y: float) -> None:
    text_size = measure_text_cached(self._font_semi_bold, text, OFFSET_FONT_SIZE)
    width = max(56.0, text_size.x + 20.0)
    pill = rl.Rectangle(bounds.x + (bounds.width - width) / 2, y, width, OFFSET_PILL_HEIGHT)
    rl.draw_rectangle_rounded(pill, roundness_for(pill, 17), 8, rl.Color(32, 20, 45, 255))
    rl.draw_rectangle_rounded_lines_ex(pill, roundness_for(pill, 17), 8, 2, OFFSET_COLOR)
    self._draw_centered_text(text, pill, y + (pill.height - text_size.y) / 2, OFFSET_FONT_SIZE, OFFSET_COLOR)

  @staticmethod
  def _max_header_color(active_side: str, cruise_set: bool) -> rl.Color:
    if cruise_set and ui_state.status == UIStatus.ENGAGED and active_side in ("max", "shared"):
      return COLORS.ENGAGED
    if cruise_set and ui_state.status in (UIStatus.DISENGAGED, UIStatus.OVERRIDE):
      return COLORS.DISENGAGED
    return COLORS.GREY

  @staticmethod
  def _limit_header_color(active_side: str, overridden: bool) -> rl.Color:
    if overridden or ui_state.status in (UIStatus.DISENGAGED, UIStatus.OVERRIDE):
      return COLORS.DISENGAGED
    if ui_state.status == UIStatus.ENGAGED and active_side in ("slc", "shared"):
      return COLORS.ENGAGED
    return COLORS.GREY

  def _draw_active_emphasis(self, rect: rl.Rectangle) -> None:
    presentation = self._presentation
    if presentation.mode == "merged" or ui_state.status != UIStatus.ENGAGED or presentation.active_side == "none":
      return
    if presentation.mode in ("max_only", "limit_only"):
      bounds = rect
    elif presentation.active_side == "slc":
      bounds = self._speed_limit_bounds(rect)
    elif presentation.active_side == "max":
      bounds = rl.Rectangle(rect.x, rect.y, rect.width / 2, rect.height)
    else:
      bounds = rect
    rl.draw_line_ex(
      rl.Vector2(bounds.x + 18, rect.y + 65),
      rl.Vector2(bounds.x + bounds.width - 18, rect.y + 65),
      3, UNIFIED_ACCENT,
    )

  def _draw_merged_separator(self, rect: rl.Rectangle) -> None:
    center = rect.x + rect.width / 2
    shelf_y = rect.y + MERGED_SEPARATOR_Y
    valley_y = shelf_y + 12
    color = rl.Color(UNIFIED_ACCENT.r, UNIFIED_ACCENT.g, UNIFIED_ACCENT.b, 170)
    rl.draw_line_ex(rl.Vector2(rect.x + 18, shelf_y), rl.Vector2(center - 34, shelf_y), 2, color)
    rl.draw_spline_segment_bezier_cubic(
      rl.Vector2(center - 34, shelf_y), rl.Vector2(center - 19, shelf_y),
      rl.Vector2(center - 23, valley_y), rl.Vector2(center - 7, valley_y), 2, color,
    )
    rl.draw_line_ex(rl.Vector2(center - 7, valley_y), rl.Vector2(center + 7, valley_y), 2, color)
    rl.draw_spline_segment_bezier_cubic(
      rl.Vector2(center + 7, valley_y), rl.Vector2(center + 23, valley_y),
      rl.Vector2(center + 19, shelf_y), rl.Vector2(center + 34, shelf_y), 2, color,
    )
    rl.draw_line_ex(rl.Vector2(center + 34, shelf_y), rl.Vector2(rect.x + rect.width - 18, shelf_y), 2, color)

  def _draw_speed_limit_border(self, rect: rl.Rectangle, right: rl.Rectangle, color: rl.Color) -> None:
    # Clip the shared rounded outline so only the Speed Limit side changes.
    rl.begin_scissor_mode(int(right.x), int(rect.y), int(right.width + 1), int(rect.height + 1))
    try:
      rl.draw_rectangle_rounded_lines_ex(rect, CONTROL_ROUNDNESS, CONTROL_SEGMENTS, 3, color)
    finally:
      rl.end_scissor_mode()
    if self._presentation.mode == "split":
      rl.draw_line_ex(rl.Vector2(right.x, rect.y + 8), rl.Vector2(right.x, rect.y + rect.height - 8), 3, color)

  def _render(self, rect: rl.Rectangle) -> None:
    presentation = self._presentation
    state = self._slc_state
    rl.draw_rectangle_rounded_lines_ex(
      rect, CONTROL_ROUNDNESS, CONTROL_SEGMENTS, 7,
      rl.Color(UNIFIED_ACCENT.r, UNIFIED_ACCENT.g, UNIFIED_ACCENT.b, 55),
    )
    draw_control_card(rect, fill=CONTROL_BG, border=UNIFIED_ACCENT, border_width=2)
    if presentation.mode == "split":
      divider_x = rect.x + rect.width / 2
      rl.draw_line_ex(
        rl.Vector2(divider_x, rect.y + 8), rl.Vector2(divider_x, rect.y + rect.height - 8),
        2, rl.Color(UNIFIED_ACCENT.r, UNIFIED_ACCENT.g, UNIFIED_ACCENT.b, 110),
      )
    elif presentation.mode == "merged":
      self._draw_merged_separator(rect)

    self._draw_active_emphasis(rect)
    max_bounds = rl.Rectangle(rect.x, rect.y, rect.width / 2, rect.height) if presentation.mode in ("split", "merged") else rect
    limit_bounds = self._speed_limit_bounds(rect)
    if self._show_max or presentation.confirmation_pending:
      max_color = COLORS.DARK_GREY if not self.hud_renderer.is_cruise_set else COLORS.WHITE
      max_label_color = self._max_header_color(presentation.active_side, self.hud_renderer.is_cruise_set)
      self._draw_header(max_bounds, "MAX SET", "speedometer", max_label_color)
      if presentation.mode != "merged":
        self._draw_centered_text(presentation.max_speed_text, max_bounds, rect.y + 75, VALUE_FONT_SIZE, max_color, bold=True)
        self._draw_centered_text(tr(presentation.unit_text), max_bounds, rect.y + 204, UNIT_FONT_SIZE, COLORS.WHITE_TRANSLUCENT)

    if limit_bounds is not None:
      icon_key = source_icon_key(presentation.source)
      overridden = bool(state and state['slc_overridden_speed'])
      label_color = self._limit_header_color(presentation.active_side, overridden)
      self._draw_header(limit_bounds, "SPEED LIMIT", icon_key, label_color)
      if presentation.mode != "merged":
        self._draw_centered_text(presentation.posted_speed_text, limit_bounds, rect.y + 75, VALUE_FONT_SIZE, COLORS.WHITE, bold=True)
        if presentation.confirmation_pending:
          self._draw_centered_text(tr("PENDING"), limit_bounds, rect.y + 175, 25, CONFIRMATION_COLOR)
        elif presentation.offset_text is not None:
          self._draw_offset_pill(limit_bounds, presentation.offset_text, rect.y + 175)
        self._draw_centered_text(tr(presentation.unit_text), limit_bounds, rect.y + 204, UNIT_FONT_SIZE, COLORS.WHITE_TRANSLUCENT)

    if presentation.mode == "merged":
      self._draw_centered_text(presentation.effective_speed_text, rect, rect.y + 98, VALUE_FONT_SIZE, COLORS.WHITE, bold=True)
      self._draw_centered_text(tr(presentation.unit_text), rect, rect.y + 204, UNIT_FONT_SIZE, COLORS.WHITE_TRANSLUCENT)
      if presentation.offset_text is not None:
        self._draw_offset_pill(
          limit_bounds, presentation.offset_text, rect.y + MERGED_SEPARATOR_Y - OFFSET_PILL_HEIGHT / 2,
        )

    if presentation.confirmation_pending and limit_bounds is not None:
      intensity = (1.0 + math.sin(2.0 * math.pi * rl.get_time())) / 2.0
      alpha = round(100 + 155 * intensity)
      pulse = rl.Color(CONFIRMATION_COLOR.r, CONFIRMATION_COLOR.g, CONFIRMATION_COLOR.b, alpha)
      self._draw_speed_limit_border(rect, limit_bounds, pulse)
    else:
      if limit_bounds is not None and state is not None:
        vision_color = _speed_limit_pulse_color(UNIFIED_ACCENT, UNIFIED_ACCENT.a)
        if (vision_color.r, vision_color.g, vision_color.b) != (UNIFIED_ACCENT.r, UNIFIED_ACCENT.g, UNIFIED_ACCENT.b):
          self._draw_speed_limit_border(rect, limit_bounds, vision_color)
      if state is not None and ui_state.ui_params.get_bool("SpeedLimitSources"):
        _draw_sources_bubble(state, rect)

  def _handle_mouse_press(self, mouse_pos) -> None:
    right = self._speed_limit_bounds(self.rect)
    if right is None and self._slc_state is not None:
      # The detailed source panel remains dismissible when no limit is valid.
      right = self.rect
    if right is None:
      return
    target = rl.Rectangle(right.x, right.y - self.TOUCH_SLOP,
                          right.width + self.TOUCH_SLOP, right.height + 2 * self.TOUCH_SLOP)
    if not rl.check_collision_point_rec(mouse_pos, target):
      return
    if self._presentation.confirmation_pending:
      Params(memory=True).put_bool("SpeedLimitAccepted", True)
      return
    params = ui_state.ui_params
    params.put_bool("SpeedLimitSources", not params.get_bool("SpeedLimitSources"))
