from collections import deque

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

# Highway weave is a loop through the model: it reacts to the car's own sway and openpilot delivers that
# request ~1:1 with ~0.4 s lag. Scaling only the fast part of the request lowers that loop's gain without
# adding lag (a low-pass adds lag and did not help). Below MIN_GAIN the output is mostly the slow baseline,
# i.e. a low-pass again.
BASELINE_TAU = 1.5                        # s
SPEED_OFF = 30.0 * CV.MPH_TO_MS
SPEED_ON = 40.0 * CV.MPH_TO_MS
LAT_ACCEL_ON = 0.25                       # m/s^2
LAT_ACCEL_OFF = 0.6                       # m/s^2
# held longer than the weave's peak spacing so the weave can't modulate its own gain
ENVELOPE_HOLD = 2.0                       # s
ENVELOPE_RELEASE_RATE = 0.3               # m/s^2 per second
BYPASS_FADE_RATE = 2.0                    # per second
MIN_GAIN = 0.3


def _smoothstep(x: float, lo: float, hi: float) -> float:
  t = min(max((x - lo) / (hi - lo), 0.0), 1.0)
  return t * t * (3.0 - 2.0 * t)


class HighwayCorrectionGain:
  def __init__(self):
    self.alpha = DT_CTRL / (BASELINE_TAU + DT_CTRL)
    self.reset()

  def reset(self, curvature: float = 0.0) -> None:
    self.baseline = curvature
    self.envelope = 0.0
    self.frame = 0
    self.peaks: deque[tuple[int, float]] = deque()
    self.bypass_weight = 0.0
    self.weight = 0.0

  def update(self, curvature: float, v_ego: float, lat_active: bool, gain: float, bypass: bool) -> float:
    if not lat_active:
      self.reset(curvature)
      return curvature

    # tracked even when bypassed so fading in never steps the output
    self.baseline += self.alpha * (curvature - self.baseline)

    lat_accel = abs(curvature) * v_ego ** 2
    self.frame += 1
    while self.peaks and self.peaks[-1][1] <= lat_accel:
      self.peaks.pop()
    self.peaks.append((self.frame, lat_accel))
    while self.peaks[0][0] <= self.frame - ENVELOPE_HOLD / DT_CTRL:
      self.peaks.popleft()
    self.envelope = max(self.peaks[0][1], self.envelope - ENVELOPE_RELEASE_RATE * DT_CTRL)

    step = BYPASS_FADE_RATE * DT_CTRL
    self.bypass_weight += min(max((0.0 if bypass else 1.0) - self.bypass_weight, -step), step)

    gain = min(max(gain, MIN_GAIN), 1.0)
    self.weight = (self.bypass_weight * _smoothstep(v_ego, SPEED_OFF, SPEED_ON) *
                   (1.0 - _smoothstep(self.envelope, LAT_ACCEL_ON, LAT_ACCEL_OFF)))
    k = 1.0 - self.weight * (1.0 - gain)
    if k >= 1.0:
      return curvature
    return self.baseline + k * (curvature - self.baseline)
