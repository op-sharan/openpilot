"""Onroad GNSS health readout for diagnosing RF desense from the eGPU's USB-C link.

Under desense the receiver keeps tracking satellites (the count can even rise) but decodes satellite
time from none of them, so the decode rate is the signal to watch, not the satellite count.
"""
import time

import pyray as rl

from cereal import log
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

WIDTH = 320
HEIGHT = 150
MARGIN = 30
PADDING = 14
TITLE_SIZE = 26
ROW_SIZE = 30

_BG = rl.Color(0, 0, 0, 180)
_LABEL = rl.Color(255, 255, 255, 140)
_GOOD = rl.Color(34, 197, 94, 255)
_WARN = rl.Color(234, 179, 8, 255)
_BAD = rl.Color(239, 68, 68, 255)

# Under ~40% has never produced a fix in logged drives.
SAT_TIME_GOOD = 60.0
SAT_TIME_WARN = 25.0

# Modem's raw carrier-noise units, ~2550 on a clean fix. It can stay high while desense blocks decoding,
# so it isn't a health signal on its own.
CNO_GOOD = 2000.0
CNO_WARN = 800.0

# "R"'s diagonal leg looks left of its edge next to a digit, even though both rows end at the same x.
SATS_OPTICAL_NUDGE = 3

# gpsLocation arrives at ~1Hz.
FIX_STALE_S = 2.0


def _grade(value: float, good: float, warn: float) -> rl.Color:
  if value >= good:
    return _GOOD
  if value >= warn:
    return _WARN
  return _BAD


class GnssHealth:
  def __init__(self):
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._sat_time_pct = 0.0
    self._cno = 0.0
    self._gps_sv = 0
    self._glonass_sv = 0
    self._has_fix = False
    self._phone_fix = False

  def _update(self) -> None:
    sm = ui_state.sm

    # qcomgpsd devices publish gpsLocation; gpsLocationExternal is only ublox/car GPS. qcomgpsd usually
    # signals "no fix" by publishing nothing at all, so a stale message has to count as no fix.
    self._has_fix = False
    self._phone_fix = False
    for service in ("gpsLocation", "gpsLocationExternal"):
      if sm.valid.get(service, False) and sm.recv_frame[service] > 0:
        fresh = time.monotonic() - sm.recv_time[service] < FIX_STALE_S
        self._has_fix = fresh and sm[service].hasFix
        self._phone_fix = self._has_fix and sm[service].source == log.GpsLocationData.SensorSource.android
        break

    if not sm.valid.get("qcomGnss", False):
      return

    # sm["qcomGnss"] is only the last message of a burst, usually not the measurementReport.
    for msg in sm.drained.get("qcomGnss", []):
      if msg.which() != "qcomGnss":
        continue
      gnss = msg.qcomGnss
      if gnss.which() != "measurementReport":
        continue

      report = gnss.measurementReport
      svs = list(report.sv)
      source = str(report.source)

      # The modem never sets satelliteTimeIsKnown on GLONASS satellites, so decode rate is GPS-only.
      is_glonass = "glonass" in source
      if is_glonass:
        self._glonass_sv = len(svs)
      else:
        self._gps_sv = len(svs)

      if svs and not is_glonass:
        known = sum(1 for sv in svs if sv.measurementStatus.satelliteTimeIsKnown)
        self._sat_time_pct = 100.0 * known / len(svs)

      if svs:
        noise = [sv.carrierNoise for sv in svs if sv.carrierNoise > 0]
        if noise:
          self._cno = sum(noise) / len(noise)

  def render(self, bounds: rl.Rectangle) -> None:
    self._update()

    x = bounds.x + bounds.width - WIDTH - MARGIN
    y = bounds.y + bounds.height - HEIGHT - MARGIN
    rect = rl.Rectangle(x, y, WIDTH, HEIGHT)
    rl.draw_rectangle_rounded(rect, 0.12, 10, _BG)

    tx = int(x + PADDING)
    right = int(x + WIDTH - PADDING)
    ty = int(y + PADDING)

    rl.draw_text_ex(self._font, "GNSS", rl.Vector2(tx, ty), TITLE_SIZE, 0, _LABEL)
    fix_text = "PHONE FIX" if self._phone_fix else ("FIX" if self._has_fix else "NO FIX")
    fix_color = _WARN if self._phone_fix else (_GOOD if self._has_fix else _BAD)
    fix_width = measure_text_cached(self._font, fix_text, TITLE_SIZE).x
    rl.draw_text_ex(self._font, fix_text, rl.Vector2(right - fix_width, ty), TITLE_SIZE, 0, fix_color)

    ty += 34
    self._draw_row(tx, right, ty, "Decode %", f"{self._sat_time_pct:.0f}%",
                   _grade(self._sat_time_pct, SAT_TIME_GOOD, SAT_TIME_WARN))

    ty += 36
    self._draw_row(tx, right, ty, "Radio Signal", f"{self._cno:.0f}",
                   _grade(self._cno, CNO_GOOD, CNO_WARN))

    ty += 36
    self._draw_row(tx, right + SATS_OPTICAL_NUDGE, ty, "Tracked Sats",
                   f"{self._gps_sv}G {self._glonass_sv}R", rl.WHITE)

  def _draw_row(self, tx: int, right: int, ty: int, label: str, value: str, value_color: rl.Color) -> None:
    rl.draw_text_ex(self._font, label, rl.Vector2(tx, ty), ROW_SIZE, 0, _LABEL)
    value_width = measure_text_cached(self._font, value, ROW_SIZE).x
    rl.draw_text_ex(self._font, value, rl.Vector2(right - value_width, ty), ROW_SIZE, 0, value_color)
