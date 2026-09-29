import math

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib import latcontrol_vehicle_tunes as tunes


@pytest.fixture(autouse=True)
def _reset_toggles():
  yield
  tunes.set_lateral_test_toggles(True, False)


def _fade(v_ego):
  return tunes.get_ioniq_6_friction_center_fade_scale(0.0, v_ego)


def test_defaults_match_production():
  assert tunes._IONIQ_6_WEAVE_TUNE and not tunes._HKG_HIGHWAY_FRICTION_THRESHOLD


def test_weave_tune_off_restores_legacy_values():
  v = 30.0
  tunes.set_lateral_test_toggles(True, False)
  tuned = (_fade(v), tunes.get_ioniq_6_center_taper_scale(0.2, v), tunes.get_ioniq_6_highway_output_taper_scale(0.12, v))
  tunes.set_lateral_test_toggles(False, False)
  legacy = (_fade(v), tunes.get_ioniq_6_center_taper_scale(0.2, v), tunes.get_ioniq_6_highway_output_taper_scale(0.12, v))
  assert all(not math.isclose(a, b) for a, b in zip(tuned, legacy, strict=True))

  speed_weight = tunes._ioniq_6_sigmoid((v - tunes.IONIQ_6_FRICTION_CENTER_FADE_SPEED) / tunes.IONIQ_6_FRICTION_CENTER_FADE_SPEED_WIDTH)
  center_weight = tunes._ioniq_6_sigmoid(tunes.IONIQ_6_FRICTION_CENTER_FADE_LAT / tunes.IONIQ_6_LEGACY_FRICTION_CENTER_FADE_LAT_WIDTH)
  assert math.isclose(legacy[0], 1.0 - tunes.IONIQ_6_LEGACY_FRICTION_CENTER_FADE_MAX * speed_weight * center_weight)


def test_hkg_highway_friction_threshold():
  mph = CV.MPH_TO_MS
  assert math.isclose(tunes.get_hkg_canfd_base_friction_threshold(70 * mph), tunes.HKG_CANFD_BASE_FRICTION_THRESHOLD)

  tunes.set_lateral_test_toggles(True, True)
  assert math.isclose(tunes.get_hkg_canfd_base_friction_threshold(20 * mph), tunes.HKG_CANFD_BASE_FRICTION_THRESHOLD)
  assert math.isclose(tunes.get_hkg_canfd_base_friction_threshold(20.0), 0.585)
  assert math.isclose(tunes.get_hkg_canfd_base_friction_threshold(70 * mph), tunes.HKG_CANFD_HIGHWAY_FRICTION_THRESHOLD)
  assert math.isclose(tunes.get_ioniq_6_friction_threshold(70 * mph), tunes.HKG_CANFD_HIGHWAY_FRICTION_THRESHOLD)
