"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.constants import CV

GENERIC_MIN_SET_SPEED = {True: 30, False: 20}  # km/h / mph


def _brand_minimum_set_speed_kph(CP) -> int | None:
  if CP is None:
    return None
  if CP.brand == "volkswagen":
    from opendbc.sunnypilot.car.volkswagen.icbm import minimum_set_speed_kph
    return minimum_set_speed_kph(CP)
  return None


def get_minimum_set_speed(is_metric: bool, CP=None) -> int:
  """Lowest set speed ICBM will request, in the display unit. Brands may allow less than the generic floor."""
  brand_min_kph = _brand_minimum_set_speed_kph(CP)
  if brand_min_kph is not None:
    return brand_min_kph if is_metric else max(1, round(brand_min_kph * CV.KPH_TO_MPH))
  return GENERIC_MIN_SET_SPEED[is_metric]
