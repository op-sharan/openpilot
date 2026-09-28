from collections import deque

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL

# Straight-highway gain on the FAST part of the desired curvature (user setting "HighwayCorrectionGain").
#
# The Ioniq 6 highway weave is a lightly damped loop through the driving model, not a controller or EPS
# problem (drives 00000b2c..00000b52):
#   - openpilot delivers 0.97x of the model's requested lateral accel, ~0.4 s late; Kp 0.4-1.4 and the
#     EPS damping byte changed nothing.
#   - with Hyundai LFA steering (model open loop) the model's request wobbles half as much and follows the
#     car's own motion (gain ~0.94 at 0.4-0.8 Hz): road kicks -> car sways -> model asks to follow the
#     sway -> openpilot delivers it. LFA delivers only ~1/1.36 of what the model asks and its weave is
#     flat vs road roughness (0.09) while openpilot's grows with it (0.18 -> 0.25).
#
# So this lowers the loop gain at the weave frequency instead of filtering: the output is
#   baseline + k * (curvature - baseline),  k = 1 - weight * (1 - gain)
# with baseline a slow (1.5 s) low-pass. At 0.35-0.8 Hz the baseline has little content, so the result is
# ~gain with only ~5 deg of phase lag, and DC / slow lane keeping passes unchanged. The earlier tau 0.3 s
# low-pass smoother (drive 00000b27) did nothing because it added lag to the same loop it meant to damp.
#
# Gating copies that smoother: only above ~45 mph, only on near-straight road (sliding-max envelope of the
# raw commanded lateral accel so the weave cannot modulate its own gain), faded out for blinkers,
# overrides, turn holds, lane changes and maneuver plans. gain = 1.0 is an exact pass-through.
BASELINE_TAU = 1.5                        # s
SPEED_OFF = 40.0 * CV.MPH_TO_MS
SPEED_ON = 50.0 * CV.MPH_TO_MS
LAT_ACCEL_ON = 0.25                       # m/s^2, envelope below this: full effect
LAT_ACCEL_OFF = 0.6                       # m/s^2, envelope above this: no effect
ENVELOPE_HOLD = 2.0                       # s, sliding-max window (outlasts 0.35 Hz weave peak spacing)
ENVELOPE_RELEASE_RATE = 0.3               # m/s^2 per second, after the hold
BYPASS_FADE_RATE = 2.0                    # per second: blinker/override fade in/out over 0.5 s
MIN_GAIN = 0.4


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
    self.peaks: deque[tuple[int, float]] = deque()  # monotonic (frame, lat accel) for the sliding max
    self.bypass_weight = 0.0
    self.weight = 0.0

  def update(self, curvature: float, v_ego: float, lat_active: bool, gain: float, bypass: bool) -> float:
    if not lat_active:
      self.reset(curvature)
      return curvature

    # The baseline always tracks the raw command, so fading in or changing the gain never steps the output.
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
