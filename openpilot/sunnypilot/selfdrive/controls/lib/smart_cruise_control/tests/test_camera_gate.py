"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

SCC-Map's near-field camera gate (camera_gate.py): a map target is vetoed only once its point is inside the camera's
reliable near field and the camera sees a straight road there, and never on a camera it cannot trust. The thresholds
are checked against the owner's logged map events: every straight-road one read under 0.55 m/s^2, every real curve
over 0.94.
"""
import json
import math
import platform

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.camera_gate import (CameraGate, near_field_lat_acc, NEAR_T, NEAR_T_BIG,
                                                                                         RELEASE_LAT_ACC, TRUST_FRAMES, VETO_LAT_ACC,
                                                                                         VETO_PERSIST_FRAMES)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import R, SmartCruiseControlMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl

T = [10. * (i / 32) ** 2 for i in range(33)]  # the model's plan times, ModelConstants.T_IDXS' shape
POINT = (46.75, 23.516)


def plan(lat_acc, v=15., curve_from_t=0.):
  """A plan at speed v predicting `lat_acc` from `curve_from_t` seconds on (yaw rate = lat_acc / v), straight before."""
  rate = [lat_acc / v if t >= curve_from_t else 0. for t in T]
  return rate, [v] * len(T), T


class TestNearField(OpenpilotTestCase):
  def test_reads_the_highest_predicted_lateral_accel_inside_the_horizon(self):
    rate, vel, t = plan(1.5, curve_from_t=2.)
    assert abs(near_field_lat_acc(rate, vel, t, 3.) - 1.5) < 1e-6
    assert near_field_lat_acc(rate, vel, t, 1.9) == 0.  # the bend starts past the horizon

  def test_unreadable_plans_never_read_as_straight(self):
    assert near_field_lat_acc([], [], [], 3.) == math.inf
    assert near_field_lat_acc([0.1, float('nan')], [15., 15.], [0., 0.5], 3.) == math.inf
    assert near_field_lat_acc(None, [15.], [0.], 3.) == math.inf
    assert near_field_lat_acc([0.01], [15.], [5.], 3.) == math.inf  # nothing inside the horizon

  def test_mismatched_lengths_read_the_common_part(self):
    assert abs(near_field_lat_acc([0.1, 0.1, 0.1], [10., 10.], [0., 1., 2.], 3.) - 1.0) < 1e-6


class TestCameraGate(OpenpilotTestCase):
  def setup_method(self):
    self.gate = CameraGate()

  def trust(self, big=False, frames=TRUST_FRAMES):
    for _ in range(frames):
      self.gate.update(True, big, *plan(0.), 15., None, 0.)

  def veto(self, lat_acc, distance=20., v=15., big=False, model_ok=True, point=POINT, n=VETO_PERSIST_FRAMES, gate=None):
    """`n` model frames of the same read; whether the last one vetoes."""
    gate = gate or self.gate
    out = False
    for _ in range(n):
      out = gate.update(model_ok, big, *plan(lat_acc, v), v, point, distance)
    return out

  def trusted_gate(self, big=False):
    gate = CameraGate()
    for _ in range(TRUST_FRAMES):
      gate.update(True, big, *plan(0.), 15., None, 0.)
    return gate

  def test_straight_road_near_the_point_is_vetoed(self):
    self.trust()
    assert self.veto(0.3)  # 20 m at 15 m/s: 1.3 s, inside the near field

  def test_a_short_misread_never_vetoes(self):
    """A straight read must last VETO_PERSIST_FRAMES: a frame or two of misread never cancels a map curve."""
    self.trust()
    for _ in range(3):
      assert not self.veto(0.1, n=VETO_PERSIST_FRAMES - 1)
      assert not self.veto(1.5, n=1)  # the bend seen again restarts the count
    assert self.veto(0.1)

  def test_a_point_still_far_is_the_map_s(self):
    """A fast curve the camera cannot see yet is what the map is for: never vetoed before the near field."""
    self.trust()
    assert not self.veto(0.0, distance=15. * (NEAR_T + 0.5), n=200)
    assert self.veto(0.0, distance=15. * (NEAR_T - 0.5))

  def test_the_big_model_s_near_field_is_longer(self):
    self.trust(big=True)
    assert self.veto(0.0, distance=15. * (NEAR_T_BIG - 0.3), big=True)
    assert not self.veto(0.0, distance=15. * (NEAR_T_BIG - 0.3), gate=self.trusted_gate(big=False), n=200)  # small: too far

  def test_a_real_curve_is_never_vetoed(self):
    self.trust()
    for lat in (0.94, 1.22, 1.6, 2.3, 4.0):  # the owner's real map curves read 0.94 and up
      assert not self.veto(lat, n=200), lat

  def test_the_logged_straight_road_events_are_vetoed(self):
    for lat in (0.05, 0.12, 0.28, 0.32, 0.52, 0.53, 0.55):  # the owner's straight-road map events
      assert self.veto(lat, gate=self.trusted_gate()), lat

  def test_thresholds_and_hysteresis(self):
    self.trust()
    assert self.veto(VETO_LAT_ACC - 0.01)
    assert self.veto(0.9, n=1)                         # vetoed: holds until the camera sees RELEASE_LAT_ACC
    assert not self.veto(RELEASE_LAT_ACC + 0.01, n=1)  # released at once
    assert not self.veto(0.9, n=200)                   # and back under VETO_LAT_ACC only to veto again
    assert self.veto(VETO_LAT_ACC - 0.01)
    assert not self.veto(VETO_LAT_ACC + 0.01, gate=self.trusted_gate(), n=200)

  def test_never_before_the_camera_is_trusted(self):
    """modeld just started: the model's history is still filling. No veto until TRUST_FRAMES on it."""
    reads = [self.gate.update(True, False, *plan(0.), 15., POINT, 20.) for _ in range(TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)]
    first = reads.index(True)
    assert TRUST_FRAMES <= first <= TRUST_FRAMES + VETO_PERSIST_FRAMES

  def test_a_jetlink_switch_resets_the_trust(self):
    """A swap to the big model or a hand-back to the built-in one starts a model from a reset history: a real map curve
    must not be vetoed on its first reads."""
    self.trust(big=True)
    assert self.veto(0.0, big=True)
    reads = [self.gate.update(True, False, *plan(0.), 15., POINT, 20.) for _ in range(TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)]
    assert not reads[0], "the hand-back lifts a standing veto at once"
    assert not any(reads[:TRUST_FRAMES])
    assert reads[-1]  # trusted again on the built-in model

  def test_a_missing_or_invalid_model_resets_the_trust(self):
    self.trust()
    assert not self.veto(0.0, model_ok=False, n=1)
    assert not self.veto(0.0, n=TRUST_FRAMES - 1)  # back, but untrusted again

  def test_a_new_point_starts_unvetoed(self):
    self.trust()
    assert self.veto(0.7)
    # a new point: neither the old one's veto nor its count carries over
    assert not self.gate.update(True, False, *plan(0.7), 15., (46.76, 23.52), 20.)

  def test_degenerate_inputs_never_veto(self):
    self.trust()
    assert not self.veto(0.0, point=None)
    assert not self.veto(0.0, distance=-1.)
    assert not self.veto(0.0, distance=math.inf)
    standing = [self.gate.update(True, False, *plan(0.), 0., POINT, 0.5) for _ in range(VETO_PERSIST_FRAMES)]
    assert standing[-1]  # standing next to it: the speed is clipped, no division by zero
    assert not self.gate.update(True, False, [], [], [], 15., POINT, 20.)  # unreadable plan: lifts the veto


class TestMapWithCameraGate(OpenpilotTestCase):
  """Through SCC-Map itself: the mapd target, the distance to its point, and the output the planner and ICBM read."""

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

  def place(self, distance, velocity):
    lon = (distance / R) * (180.0 / math.pi)
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0.0, "longitude": 0.0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([{"latitude": 0.0, "longitude": lon, "velocity": velocity}]), block=True)

  def run_map(self, scc_m, camera, frames, v_ego=15.):
    for _ in range(frames):
      scc_m.update(True, False, v_ego, 0., 25., camera)

  def test_a_near_phantom_is_vetoed_and_logged(self):
    scc_m = SmartCruiseControlMap()
    scc_m.camera_gate = CameraGate()
    self.place(20., 5.)
    self.run_map(scc_m, (True, False, *plan(0.2)), TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)
    assert scc_m.is_active and scc_m.camera_veto
    assert scc_m.output_v_target == V_CRUISE_UNSET
    assert abs(scc_m.target_distance - 20.) < 0.5

  def test_a_far_point_keeps_its_target(self):
    scc_m = SmartCruiseControlMap()
    scc_m.camera_gate = CameraGate()
    self.place(90., 5.)
    self.run_map(scc_m, (True, False, *plan(0.2)), TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)
    assert scc_m.is_active and not scc_m.camera_veto
    assert abs(scc_m.output_v_target - max(5., MIN_V)) < 1e-6

  def test_a_near_real_curve_keeps_its_target(self):
    scc_m = SmartCruiseControlMap()
    scc_m.camera_gate = CameraGate()
    self.place(20., 5.)
    self.run_map(scc_m, (True, False, *plan(2.0)), TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)
    assert scc_m.is_active and not scc_m.camera_veto and scc_m.output_v_target < V_CRUISE_UNSET

  def test_without_the_gate_nothing_changes(self):
    """openpilot longitudinal cars, and no camera data: the map target is applied as before."""
    for gate, camera in ((None, (True, False, *plan(0.2))), (CameraGate(), None)):
      scc_m = SmartCruiseControlMap()
      scc_m.camera_gate = gate
      self.place(20., 5.)
      self.run_map(scc_m, camera, TRUST_FRAMES + VETO_PERSIST_FRAMES + 5)
      assert scc_m.is_active and not scc_m.camera_veto and scc_m.output_v_target < V_CRUISE_UNSET


class TestSmartCruiseControlCamera(OpenpilotTestCase):
  def test_the_camera_comes_off_model_v2(self):
    msg = messaging.new_message('modelV2')
    rate, vel, t = plan(1.2)
    msg.modelV2.orientationRate.z, msg.modelV2.orientationRate.t, msg.modelV2.velocity.x = rate, t, vel
    msg.modelV2.big = True
    class SM:
      alive = {'modelV2': True}
      valid = {'modelV2': True}

      def __getitem__(self, k):
        return msg.modelV2
    model_ok, big, rz, vx, tt = SmartCruiseControl.camera(SM())
    assert model_ok and big and abs(near_field_lat_acc(rz, vx, tt, NEAR_T) - 1.2) < 1e-3

  def test_the_gate_only_on_stock_acc_with_icbm(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    for op_long, pcm, pcm_speed, gated in ((False, True, False, True),    # stock ACC + ICBM: the Tiguan
                                           (False, True, True, False),    # stock ACC, ICBM off
                                           (True, True, False, False),    # openpilot longitudinal
                                           (True, False, True, False)):
      CP = car.CarParams(openpilotLongitudinalControl=op_long, pcmCruise=pcm)
      CP_SP = custom.CarParamsSP(pcmCruiseSpeed=pcm_speed)
      planner = LongitudinalPlannerSP(CP, CP_SP, None)
      assert (planner.scc.map.camera_gate is not None) == gated, (op_long, pcm, pcm_speed)
