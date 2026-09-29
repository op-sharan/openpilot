"""Displayed Max Set and posted-limit values for the Big UI speed card."""

from dataclasses import dataclass


@dataclass(frozen=True)
class UnifiedSpeedPresentation:
  mode: str
  max_speed_text: str
  posted_speed_text: str
  effective_speed_text: str
  offset_text: str | None
  unit_text: str
  source: str
  confirmation_pending: bool
  active_side: str


def resolve_unified_speed(show_max: bool, cruise_set: bool, max_speed: float,
                          slc_state: dict | None, is_metric: bool) -> UnifiedSpeedPresentation:
  """Compare the rounded values the driver sees; ignore override speed for layout."""
  unit = "km/h" if is_metric else "mph"
  max_text = str(round(max_speed)) if cruise_set else "–"
  if slc_state is None:
    return UnifiedSpeedPresentation("max_only", max_text, "–", "–", None, unit, "None", False, "max" if cruise_set else "none")

  conversion = slc_state['speed_conversion']
  accepted = slc_state['accepted_speed_limit_ms']
  effective = slc_state['effective_target_ms']
  candidate = slc_state['unconfirmed_speed_limit']
  pending = bool(slc_state['speed_limit_changed'] and slc_state['unconfirmed_valid'])
  source = slc_state['presented_source']
  has_limit = (source not in ("", "None") and accepted > 1) or pending
  if not has_limit:
    return UnifiedSpeedPresentation("max_only", max_text, "–", "–", None, unit, "None", False, "max" if cruise_set else "none")

  posted_text = str(round(candidate)) if pending else str(round(accepted * conversion))
  effective_text = str(round(effective * conversion)) if effective > 0 else "–"
  offset_display = round(slc_state['offset_ms'] * conversion)
  offset_text = f"{offset_display:+d}" if offset_display else None
  merged = bool(show_max and cruise_set and slc_state['slc_enabled'] and not pending and max_text == effective_text)
  mode = "split" if pending else "merged" if merged else "split" if show_max else "limit_only"
  active_side = "none" if slc_state['slc_overridden_speed'] else "shared" if merged else (
    "slc" if slc_state['slc_is_limiting_max_set'] else "max" if (show_max or pending) and cruise_set else "none"
  )
  return UnifiedSpeedPresentation(mode, max_text, posted_text, effective_text, offset_text, unit, source, pending, active_side)
