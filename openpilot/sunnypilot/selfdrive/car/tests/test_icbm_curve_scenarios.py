"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

SCC-Map, its camera gate and ICBM together, closed loop: the map's target as the planner publishes it is what ICBM
walks the car's set speed to. A phantom (the map's slow point on a road the camera sees straight) still starts a
slowdown while its point is far, and the gate ends it once the point is near; a real curve is followed all the way;
a Jetlink hand-back on the way to a real curve leaves the map's target in charge.
"""
import json
import math
import platform

from opendbc.car.structs import car
from opendbc.car.volkswagen.values import VolkswagenFlags
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.car.tests.test_icbm_hold_set_speed import _Params, _cc, _cs
from openpilot.sunnypilot.selfdrive.car.tests.test_icbm_restore import Car
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.camera_gate import CameraGate, TRUST_FRAMES
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import R, SmartCruiseControlMap

T = [10. * (i / 32) ** 2 for i in range(33)]
V_EGO = 15.  # m/s, held: the point's distance is what this is about
SET_KPH = 60
POINT_V = 20 / 3.6  # the map's slow point: a junction turn's speed
MODEL_EVERY = 5  # ICBM at 100 Hz, the planner at 20 Hz


def plan(lat_acc):
  return [lat_acc / V_EGO] * len(T), [V_EGO] * len(T), T


class TestIcbmMapScenarios(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

  def drive(self, gate, camera_lat, start_distance, big=lambda f: False, frames=2400):
    """The car drives at V_EGO toward the map's point; returns the lowest set speed, the final one, the vetoed frames."""
    scc_m = SmartCruiseControlMap()
    scc_m.camera_gate = CameraGate() if gate else None
    CP = car.CarParams(brand="volkswagen", pcmCruise=True, flags=int(VolkswagenFlags.MQB_EVO_GEN2))
    icbm = IntelligentCruiseButtonManagement(CP, custom.CarParamsSP(pcmCruiseSpeed=False), params=_Params(True))
    car_ = Car(SET_KPH)
    point_lon = (start_distance / R) * (180. / math.pi)
    self.mem_params.put("MapTargetVelocities", json.dumps([{"latitude": 0., "longitude": point_lon, "velocity": POINT_V}]), block=True)
    lowest, vetoed, map_kph = SET_KPH, 0, None
    # the camera has run long enough to be trusted before the drive starts
    for _ in range(TRUST_FRAMES + 1):
      if scc_m.camera_gate is not None:
        scc_m.camera_gate.update(True, big(0), *plan(0.), V_EGO, None, 0.)
    for f in range(frames):
      travelled = V_EGO * f * 0.01
      if f % MODEL_EVERY == 0:
        lon = (travelled / R) * (180. / math.pi)
        self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0., "longitude": lon}), block=True)
        if travelled > start_distance:
          self.mem_params.put("MapTargetVelocities", "[]", block=True)  # passed: mapd's path no longer has it
        scc_m.update(True, False, V_EGO, 0., SET_KPH * CV.KPH_TO_MS, (True, big(f), *plan(camera_lat(travelled))))
        vetoed += scc_m.camera_veto
        map_kph = None if scc_m.output_v_target >= V_CRUISE_UNSET else scc_m.output_v_target * CV.MS_TO_KPH
      lp = custom.LongitudinalPlanSP()
      lp.vTarget = V_CRUISE_UNSET
      lp.smartCruiseControl.vision.vTarget = V_CRUISE_UNSET
      lp.smartCruiseControl.map.vTarget = V_CRUISE_UNSET if map_kph is None else map_kph * CV.KPH_TO_MS
      lp.speedLimit.assist.vTarget = V_CRUISE_UNSET
      icbm.run(_cs(SET_KPH, v_ego_kph=V_EGO * CV.MS_TO_KPH, cluster_kph=car_.cluster), _cc(), lp, is_metric=True)
      car_.step(icbm)
      lowest = min(lowest, car_.cluster)
    return lowest, car_.cluster, vetoed

  def test_a_phantom_is_cut_short(self):
    """The camera sees a straight road all the way: without the gate the car walks down toward the turn's speed until it
    passes the point; with it the slowdown ends once the point is in the near field, and the set speed comes back."""
    straight = lambda d: 0.1  # noqa: E731
    low_plain, end_plain, _ = self.drive(False, straight, 100.)
    low_gated, end_gated, vetoed = self.drive(True, straight, 100.)
    assert vetoed > 0
    assert low_gated >= low_plain + 10, (low_plain, low_gated)
    assert low_gated < SET_KPH  # what the gate cannot do: the far point still starts the slowdown
    assert end_gated == SET_KPH

  def test_a_real_curve_is_followed_to_the_turn_speed(self):
    """The camera sees the bend once it is near: the map's target stands and the car is walked down as without the gate."""
    curve = lambda d: 2.0 if d > 40. else 0.1  # noqa: E731
    low_plain, _, _ = self.drive(False, curve, 100.)
    low_gated, _, vetoed = self.drive(True, curve, 100.)
    assert vetoed == 0
    assert low_gated == low_plain

  def test_a_hand_back_on_the_way_keeps_the_map_in_charge(self):
    """A Jetlink hand-back as the point comes near: the big model saw the bend; the built-in model starts from a reset
    history and reads the road straight for a while, but it is not trusted yet, so the map's real curve is not vetoed."""
    handed_back_at = 45.  # m travelled (3 s): the big model until then
    reads = lambda d: 2.0 if d < handed_back_at else (0.1 if d < 75. else 2.0)  # noqa: E731
    big = lambda f: f < 300  # noqa: E731
    low_plain, _, _ = self.drive(False, reads, 100.)
    low_gated, _, vetoed = self.drive(True, reads, 100., big=big)
    assert vetoed == 0
    assert low_gated == low_plain

  def test_known_limit_a_trusted_misread_vetoes_until_the_bend_is_seen(self):
    """What the gate cannot remove: a trusted camera that reads a real curve as straight inside its near field vetoes the
    map's target for as long as it does. The veto lifts the moment the camera sees the bend and the car is walked down
    again; the cost is the slowdown lost meanwhile."""
    sees_late = lambda d: 0.1 if d < 70. else 2.0  # noqa: E731  (the point is at 100 m)
    low_plain, _, _ = self.drive(False, sees_late, 100.)
    low_gated, _, vetoed = self.drive(True, sees_late, 100.)
    assert vetoed > 0
    assert low_gated > low_plain  # the slowdown lost while the camera misread
    assert low_gated < SET_KPH    # but the car was walked down again once it saw the bend
