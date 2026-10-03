from collections import deque

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

# Highway weave is a loop through the model: it reacts to the car's own sway and openpilot delivers that
# request ~1:1 with ~0.4 s lag. Scaling only the fast part of the request lowers that loop's gain without
# adding lag (a low-pass adds lag and did not help). Near MIN_GAIN the output is mostly the slow baseline,
# i.e. a low-pass again (~35 deg lag at 0.5 Hz for 0.2, ~53 deg for 0.1).
BASELINE_TAU = 1.5                        # s
SPEED_OFF = 30.0 * CV.MPH_TO_MS
SPEED_ON = 40.0 * CV.MPH_TO_MS
# Inside a curve the gate measures distance from the curve's own level (a slow average of the request)
# instead of from zero, so a steady curve is smoothed like a straight. Until the level builds up it is the
# original zero-referenced gate, which keeps curve entries as before; in the curve the thresholds tighten so
# the request leaving the level (exit, tightening) switches smoothing off before the slow baseline can hold
# the car in the curve.
CURVE_LEVEL_TAU = 3.0                     # s
LAT_ACCEL_ON = 0.25                       # m/s^2
LAT_ACCEL_OFF = 0.6                       # m/s^2
CURVE_DEVIATION_ON = 0.12                 # m/s^2
CURVE_DEVIATION_OFF = 0.30                # m/s^2
CURVE_LEVEL_BLEND = (0.15, 0.35)          # m/s^2 of curve level: straight thresholds -> curve thresholds
TIGHT_CURVE_ON = 1.5                      # m/s^2
TIGHT_CURVE_OFF = 2.0                     # m/s^2
# held longer than the weave's peak spacing so the weave can't modulate its own gain
ENVELOPE_HOLD = 2.0                       # s
ENVELOPE_RELEASE_RATE = 0.3               # m/s^2 per second
BYPASS_FADE_RATE = 2.0                    # per second
MIN_GAIN = 0.1


def _smoothstep(x: float, lo: float, hi: float) -> float:
  t = min(max((x - lo) / (hi - lo), 0.0), 1.0)
  return t * t * (3.0 - 2.0 * t)


class _Envelope:
  def __init__(self):
    self.frame = 0
    self.peaks: deque[tuple[int, float]] = deque()
    self.value = 0.0

  def update(self, x: float) -> float:
    self.frame += 1
    while self.peaks and self.peaks[-1][1] <= x:
      self.peaks.pop()
    self.peaks.append((self.frame, x))
    while self.peaks[0][0] <= self.frame - ENVELOPE_HOLD / DT_CTRL:
      self.peaks.popleft()
    self.value = max(self.peaks[0][1], self.value - ENVELOPE_RELEASE_RATE * DT_CTRL)
    return self.value


class HighwayCorrectionGain:
  def __init__(self):
    self.alpha = DT_CTRL / (BASELINE_TAU + DT_CTRL)
    self.alpha_curve = DT_CTRL / (CURVE_LEVEL_TAU + DT_CTRL)
    self.reset()

  def reset(self, curvature: float = 0.0) -> None:
    self.baseline = curvature
    self.curve_level = curvature
    self.envelope = _Envelope()
    self.bypass_weight = 0.0
    self.weight = 0.0

  def update(self, curvature: float, v_ego: float, lat_active: bool, gain: float, bypass: bool) -> float:
    if not lat_active:
      self.reset(curvature)
      return curvature

    # tracked even when bypassed so fading in never steps the output
    self.baseline += self.alpha * (curvature - self.baseline)
    self.curve_level += self.alpha_curve * (curvature - self.curve_level)

    v2 = v_ego ** 2
    level = abs(self.curve_level) * v2
    in_curve = _smoothstep(level, *CURVE_LEVEL_BLEND)
    on = LAT_ACCEL_ON + in_curve * (CURVE_DEVIATION_ON - LAT_ACCEL_ON)
    off = LAT_ACCEL_OFF + in_curve * (CURVE_DEVIATION_OFF - LAT_ACCEL_OFF)
    envelope = self.envelope.update(abs(curvature - in_curve * self.curve_level) * v2)
    gate = (1.0 - _smoothstep(envelope, on, off)) * (1.0 - _smoothstep(level, TIGHT_CURVE_ON, TIGHT_CURVE_OFF))

    step = BYPASS_FADE_RATE * DT_CTRL
    self.bypass_weight += min(max((0.0 if bypass else 1.0) - self.bypass_weight, -step), step)

    gain = min(max(gain, MIN_GAIN), 1.0)
    self.weight = self.bypass_weight * _smoothstep(v_ego, SPEED_OFF, SPEED_ON) * gate
    k = 1.0 - self.weight * (1.0 - gain)
    if k >= 1.0:
      return curvature
    return self.baseline + k * (curvature - self.baseline)
