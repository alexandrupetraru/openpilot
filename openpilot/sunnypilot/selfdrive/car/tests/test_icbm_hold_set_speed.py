"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.structs import car
from opendbc.car.volkswagen.values import VolkswagenFlags
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CAR_SYNC_FRAMES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (IntelligentCruiseButtonManagement,
                                                                                                DRIVER_ADJUST_FRAMES, RESTORE_DELAY_FRAMES,
                                                                                                STANDSTILL_SPEED, INACTIVE_TIMER)
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState

UNSET = float(V_CRUISE_UNSET)  # inactive limiters publish this sentinel (m/s)
FIRST_PRESS_FRAMES = int(INACTIVE_TIMER / DT_CTRL) + 20  # the state machine arms itself in preActive before the first press


class _Params:
  def __init__(self, hold=True):
    self.hold = hold

  def get(self, key, return_default=False):
    assert key == "IcbmHoldSetSpeed"
    return self.hold


def _lp_sp(v_target_kph=UNSET, vision_kph=None, map_kph=None, assist_kph=None):
  lp = custom.LongitudinalPlanSP()
  lp.vTarget = v_target_kph if v_target_kph == UNSET else v_target_kph * CV.KPH_TO_MS
  lp.smartCruiseControl.vision.vTarget = UNSET if vision_kph is None else vision_kph * CV.KPH_TO_MS
  lp.smartCruiseControl.map.vTarget = UNSET if map_kph is None else map_kph * CV.KPH_TO_MS
  lp.speedLimit.assist.vTarget = UNSET if assist_kph is None else assist_kph * CV.KPH_TO_MS
  return lp


def _cs(set_kph, buttons=(), v_ego_kph=None, cluster_kph=None):
  """set_kph: openpilot's own set speed (CS.vCruise); cluster_kph: what the car's ACC currently has (moved by ICBM)."""
  v_ego = (set_kph if v_ego_kph is None else v_ego_kph) * CV.KPH_TO_MS
  cluster = (set_kph if cluster_kph is None else cluster_kph) * CV.KPH_TO_MS
  CS = car.CarState(vEgo=v_ego, vEgoRaw=v_ego, vCruise=set_kph, vCruiseCluster=set_kph,
                    cruiseState={"available": True, "enabled": True, "speed": cluster, "speedCluster": cluster})
  CS.buttonEvents = list(buttons)
  return CS


def _cc(enabled=True):
  return car.CarControl(enabled=enabled)


class TestIcbmHoldSetSpeed(OpenpilotTestCase):
  """Hold Set Speed: ICBM keeps the driver's set speed and only lowers it for the limiters (SCC / SLA)."""

  def setup_method(self):
    self.CP = car.CarParams(brand="volkswagen", pcmCruise=True, flags=int(VolkswagenFlags.MQB_EVO_GEN2))
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)

  def _icbm(self, hold=True):
    return IntelligentCruiseButtonManagement(self.CP, self.CP_SP, params=_Params(hold))

  def _run(self, icbm, set_kph, lp_sp, n=1, buttons=(), force_decel=False, v_ego_kph=None, cluster_kph=None):
    for _ in range(n):
      icbm.run(_cs(set_kph, buttons, v_ego_kph, cluster_kph), _cc(), lp_sp, is_metric=True, force_decel=force_decel)
      buttons = ()
    return icbm

  def test_planned_speed_is_ignored_in_hold_mode(self):
    """The MPC plans a ramp below the set speed after every '+'; that must not turn into '-' presses anymore."""
    icbm = self._run(self._icbm(), 40, _lp_sp(v_target_kph=25), n=50)
    assert icbm.v_target == 40 and icbm.state == State.holding and icbm.cruise_button == SendButtonState.none

  def test_legacy_mode_still_follows_the_plan(self):
    icbm = self._run(self._icbm(hold=False), 40, _lp_sp(v_target_kph=25), n=FIRST_PRESS_FRAMES)
    assert icbm.v_target == 25 and icbm.state == State.decreasing and icbm.cruise_button == SendButtonState.decrease

  def test_limiters_lower_the_target_and_the_set_speed_comes_back(self):
    icbm = self._run(self._icbm(), 60, _lp_sp(vision_kph=45), n=FIRST_PRESS_FRAMES)
    assert icbm.v_target == 45 and icbm.state == State.decreasing
    icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), n=50, cluster_kph=45)  # the car followed the presses down
    assert icbm.state == State.holding
    icbm = self._run(icbm, 60, _lp_sp(), n=RESTORE_DELAY_FRAMES - 10, cluster_kph=45)  # curve over: nothing lower than the set speed
    assert icbm.v_target == 60 and icbm.cruise_button == SendButtonState.none  # but not raised back yet
    icbm = self._run(icbm, 60, _lp_sp(), n=20, cluster_kph=45)
    assert icbm.state == State.increasing and icbm.cruise_button == SendButtonState.increase

  def test_speed_limit_cap_and_release(self):
    """70 set, 50 zone -> 50; limit rises to 90 -> back to the driver's 70, not 90."""
    icbm = self._run(self._icbm(), 70, _lp_sp(assist_kph=50), n=FIRST_PRESS_FRAMES)
    assert icbm.v_target == 50 and icbm.state == State.decreasing
    icbm = self._run(icbm, 70, _lp_sp(assist_kph=90), n=RESTORE_DELAY_FRAMES + 20)
    assert icbm.v_target == 70 and icbm.state == State.holding

  def test_lowest_limiter_wins(self):
    icbm = self._run(self._icbm(), 90, _lp_sp(vision_kph=70, map_kph=50, assist_kph=80), n=50)
    assert icbm.v_target == 50

  def test_unset_limiter_values_do_not_pull_the_speed_down(self):
    """Before the first longitudinalPlanSP arrives its fields are 0: that is 'no limit', never 'slow to the floor'."""
    icbm = self._run(self._icbm(), 50, _lp_sp(vision_kph=0, map_kph=0, assist_kph=0), n=50)
    assert icbm.v_target == 50 and icbm.cruise_button == SendButtonState.none

  def test_force_decel_drives_the_target_to_the_floor(self):
    icbm = self._run(self._icbm(), 50, _lp_sp(), n=FIRST_PRESS_FRAMES, force_decel=True)
    assert icbm.v_target == 0 and icbm.v_cruise_min == 20 and icbm.state == State.decreasing

  def test_never_asks_for_more_than_the_driver_set(self):
    icbm = self._run(self._icbm(), 60, _lp_sp(vision_kph=120, map_kph=130), n=50)
    assert icbm.v_target == 60 and icbm.cruise_button == SendButtonState.none

  def test_pauses_after_a_driver_press(self):
    """The car applies its own step to the stalk press and openpilot adopts it; ICBM must not fight either."""
    icbm = self._run(self._icbm(), 60, _lp_sp(vision_kph=45), n=FIRST_PRESS_FRAMES)
    assert icbm.is_ready and icbm.cruise_button == SendButtonState.decrease
    icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), buttons=[ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    assert not icbm.is_ready and icbm.cruise_button == SendButtonState.none and icbm.driver_adjust_frames == DRIVER_ADJUST_FRAMES == CAR_SYNC_FRAMES
    icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), n=10, buttons=[ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    assert not icbm.is_ready and icbm.driver_adjust_frames == DRIVER_ADJUST_FRAMES - 9  # the release re-arms the pause
    icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), n=DRIVER_ADJUST_FRAMES - 10)
    assert not icbm.is_ready
    icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), n=FIRST_PRESS_FRAMES)  # re-armed: preActive timer runs again
    assert icbm.is_ready and icbm.cruise_button == SendButtonState.decrease

  def test_lowering_is_never_delayed_raising_waits(self):
    icbm = self._run(self._icbm(), 60, _lp_sp(vision_kph=45), n=FIRST_PRESS_FRAMES)
    assert icbm.cruise_button == SendButtonState.decrease  # lowering starts as soon as the state machine is armed
    for _ in range(3):  # a limiter that flickers on/off never produces a '+' press
      icbm = self._run(icbm, 60, _lp_sp(), n=RESTORE_DELAY_FRAMES // 2, cluster_kph=45)
      assert icbm.cruise_button == SendButtonState.none
      icbm = self._run(icbm, 60, _lp_sp(vision_kph=45), n=5, cluster_kph=45)
      assert icbm.cruise_button == SendButtonState.none
    icbm = self._run(icbm, 60, _lp_sp(), n=RESTORE_DELAY_FRAMES + 10, cluster_kph=45)
    assert icbm.cruise_button == SendButtonState.increase

  def test_no_presses_near_standstill(self):
    """'+' is RESUME on VW: a press while the ACC holds at a stop would drive the car off."""
    icbm = self._run(self._icbm(), 60, _lp_sp(), n=RESTORE_DELAY_FRAMES + 20, v_ego_kph=30)
    for v in (0, 3, STANDSTILL_SPEED * CV.MS_TO_KPH - 1):
      icbm = self._run(icbm, 60, _lp_sp(), n=20, v_ego_kph=v)
      assert not icbm.is_ready and icbm.cruise_button == SendButtonState.none
    icbm = self._run(icbm, 60, _lp_sp(), n=RESTORE_DELAY_FRAMES + 60, v_ego_kph=12)
    assert icbm.is_ready
    CS = _cs(60, v_ego_kph=30)
    CS.cruiseState.standstill = True
    icbm.run(CS, _cc(), _lp_sp(), is_metric=True)
    assert not icbm.is_ready

  def test_toggle_is_read_live(self):
    icbm = self._icbm(hold=True)
    icbm.params.hold = False
    icbm.read_params()
    assert not icbm.hold_set_speed


class TestIcbmMinimumSetSpeed(OpenpilotTestCase):
  def test_generic_floor(self):
    assert get_minimum_set_speed(True) == 30 and get_minimum_set_speed(False) == 20
    assert get_minimum_set_speed(True, car.CarParams(brand="volkswagen")) == 30  # MQB, no Gen 2 flag

  def test_mqb_evo_gen2_floor(self):
    CP = car.CarParams(brand="volkswagen", flags=int(VolkswagenFlags.MQB_EVO_GEN2))
    assert get_minimum_set_speed(True, CP) == 20 and get_minimum_set_speed(False, CP) == 12

  def test_other_brands_untouched(self):
    assert get_minimum_set_speed(True, car.CarParams(brand="hyundai", flags=int(VolkswagenFlags.MQB_EVO_GEN2))) == 30
