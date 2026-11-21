"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ICBM's restore (raising the set speed back after a limiter), closed loop: a car whose set speed (the cluster) answers
each ICBM press by one unit, at most one press every 0.21 s as the VW interface sends them. What is pinned:
  - a target that keeps rising (a curve exit, a staircase of small steps) is followed without a new wait at every step;
  - a limiter that clears for less than the restore delay never produces a '+' press (flicker), and a drop of the
    target, even one still above the car's set speed, restarts the wait; one-unit jitter does not;
  - lowering is never delayed; the car is never asked for more than the driver set;
  - the Jetlink hand-back hold, the standstill gate and the driver-press pause all win over a restore that is due.
"""
from opendbc.car.structs import car
from opendbc.car.volkswagen.values import VolkswagenFlags
from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement, DRIVER_ADJUST_FRAMES, RESTORE_DELAY_FRAMES, RESTORE_DROP_TOLERANCE, INACTIVE_TIMER)
from openpilot.sunnypilot.selfdrive.car.tests.test_icbm_hold_set_speed import _Params, _cc, _cs, _lp_sp

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState

PRESS_FRAMES = 21  # the VW interface sends a press at most every 0.2 s (icbm.py: strictly more than 20 frames)
ARM_FRAMES = int(INACTIVE_TIMER / DT_CTRL) + 5


class Car:
  """The car's set speed, moved one unit by each ICBM press."""

  def __init__(self, cluster):
    self.cluster = cluster
    self.frame = 0
    self.last_press = -PRESS_FRAMES
    self.ups = self.downs = 0

  def step(self, icbm):
    self.frame += 1
    if icbm.cruise_button != SendButtonState.none and self.frame - self.last_press >= PRESS_FRAMES:
      if icbm.cruise_button == SendButtonState.increase:
        self.cluster += 1
        self.ups += 1
      else:
        self.cluster -= 1
        self.downs += 1
      self.last_press = self.frame


class TestIcbmRestore(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(brand="volkswagen", pcmCruise=True, flags=int(VolkswagenFlags.MQB_EVO_GEN2))
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.icbm = IntelligentCruiseButtonManagement(self.CP, self.CP_SP, params=_Params(True))

  def drive(self, car_, set_kph, lp, frames, hold_raise=False, v_ego_kph=None, buttons_at=None):
    """`lp`: a LongitudinalPlanSP, or a function of the frame giving one. Returns (target, cluster, state) per frame."""
    trace = []
    for f in range(frames):
      buttons = buttons_at.get(f, ()) if buttons_at else ()
      self.icbm.run(_cs(set_kph, buttons, v_ego_kph, car_.cluster), _cc(), lp(f) if callable(lp) else lp, is_metric=True,
                    hold_raise=hold_raise(f) if callable(hold_raise) else hold_raise)
      car_.step(self.icbm)
      trace.append((self.icbm.v_target, car_.cluster, self.icbm.state))
    return trace

  def settle_at(self, set_kph, limiter_kph):
    """Engaged under a limiter, the car brought down to it."""
    car_ = Car(set_kph)
    self.drive(car_, set_kph, _lp_sp(vision_kph=limiter_kph), ARM_FRAMES + PRESS_FRAMES * (set_kph - limiter_kph + 3))
    assert car_.cluster == limiter_kph and self.icbm.state == State.holding
    car_.ups = car_.downs = 0  # count only what follows
    return car_

  @staticmethod
  def waiting(trace):
    """Frames the car sat under a higher target with ICBM not raising."""
    return sum(1 for tgt, cluster, state in trace if tgt > cluster and state != State.increasing)

  def test_restore_delay(self):
    assert RESTORE_DELAY_FRAMES == int(1.5 / DT_CTRL) and RESTORE_DROP_TOLERANCE == 2

  def test_rising_target_is_followed_without_a_wait_per_step(self):
    """Leaving a curve the target rises a unit at a time: one wait, then every step is followed at the press rate."""
    car_ = self.settle_at(60, 40)
    steps = 15
    hold = 2 * PRESS_FRAMES  # each step stays long enough for the car to catch up before the next
    trace = self.drive(car_, 60, lambda f: _lp_sp(vision_kph=min(40 + 1 + f // hold, 40 + steps)), hold * steps + RESTORE_DELAY_FRAMES)
    assert car_.cluster == 40 + steps
    # the one wait is the restore delay at the first step; the old rule waited again at every one of the 15
    assert self.waiting(trace) <= RESTORE_DELAY_FRAMES + PRESS_FRAMES
    assert car_.downs == 0

  def test_flicker_shorter_than_the_delay_never_raises(self):
    car_ = self.settle_at(60, 45)
    for _ in range(4):
      self.drive(car_, 60, _lp_sp(), RESTORE_DELAY_FRAMES - 15)  # the limiter clears...
      self.drive(car_, 60, _lp_sp(vision_kph=45), 10)            # ...and is back
    assert car_.ups == 0 and car_.cluster == 45

  def test_a_clear_longer_than_the_delay_raises(self):
    car_ = self.settle_at(60, 45)
    trace = self.drive(car_, 60, _lp_sp(), RESTORE_DELAY_FRAMES + PRESS_FRAMES * 20)
    assert car_.ups > 0 and car_.downs == 0
    first_up = next(i for i, (tgt, cluster, state) in enumerate(trace) if cluster > 45)
    # the frame the wait is served on presses: RESTORE_DELAY_FRAMES frames after the clear, counting the first
    assert RESTORE_DELAY_FRAMES - 1 <= first_up <= RESTORE_DELAY_FRAMES + PRESS_FRAMES

  def test_a_drop_above_the_set_speed_restarts_the_wait(self):
    """Still above the car's set speed, a limiter that comes in lower is new information: wait again."""
    car_ = self.settle_at(60, 40)
    self.drive(car_, 60, _lp_sp(vision_kph=50), RESTORE_DELAY_FRAMES - 30)
    assert car_.ups == 0
    trace = self.drive(car_, 60, _lp_sp(vision_kph=46), RESTORE_DELAY_FRAMES + PRESS_FRAMES)
    first_up = next(i for i, (tgt, cluster, state) in enumerate(trace) if cluster > 40)
    assert first_up >= RESTORE_DELAY_FRAMES - 1

  def test_one_unit_jitter_does_not_restart_the_wait(self):
    car_ = self.settle_at(60, 40)
    trace = self.drive(car_, 60, lambda f: _lp_sp(vision_kph=50 - (f % 2)), RESTORE_DELAY_FRAMES + PRESS_FRAMES * 3)
    first_up = next(i for i, (tgt, cluster, state) in enumerate(trace) if cluster > 40)
    assert first_up <= RESTORE_DELAY_FRAMES + PRESS_FRAMES

  def test_lowering_is_never_delayed_while_a_raise_waits(self):
    car_ = self.settle_at(60, 45)
    self.drive(car_, 60, _lp_sp(), RESTORE_DELAY_FRAMES // 2)
    trace = self.drive(car_, 60, _lp_sp(vision_kph=40), PRESS_FRAMES * 2)
    assert car_.cluster < 45 and trace[0][2] in (State.preActive, State.decreasing, State.holding)

  def test_never_above_the_driver_set_speed(self):
    car_ = self.settle_at(60, 40)
    trace = self.drive(car_, 60, lambda f: _lp_sp(vision_kph=40 + f // 10), 2000)
    assert max(cluster for _, cluster, _ in trace) == 60 and car_.cluster == 60

  def test_jetlink_hand_back_holds_a_due_restore(self):
    """The hold wins over a restore that is due, lowering still works during it, and the restore goes ahead (its wait
    already served) once the warning is over."""
    car_ = self.settle_at(60, 40)
    hold = int(5. / DT_CTRL)
    trace = self.drive(car_, 60, _lp_sp(vision_kph=55), hold, hold_raise=True)
    assert car_.ups == 0 and car_.cluster == 40 and all(state != State.increasing for _, _, state in trace)
    trace = self.drive(car_, 60, _lp_sp(vision_kph=55), PRESS_FRAMES * 3)
    assert car_.ups >= 2

  def test_jetlink_hand_back_still_lowers(self):
    car_ = self.settle_at(60, 50)
    self.drive(car_, 60, _lp_sp(vision_kph=42), PRESS_FRAMES * 12, hold_raise=True)
    assert car_.cluster == 42 and car_.ups == 0

  def test_standstill_wins_over_a_due_restore(self):
    car_ = self.settle_at(60, 40)
    self.drive(car_, 60, _lp_sp(), RESTORE_DELAY_FRAMES + 50, v_ego_kph=5)
    assert car_.ups == 0 and car_.cluster == 40

  def test_driver_press_pause_wins_over_a_due_restore(self):
    car_ = self.settle_at(60, 40)
    press = {0: [ButtonEvent(type=ButtonType.decelCruise, pressed=True)], 5: [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]}
    self.drive(car_, 60, _lp_sp(), RESTORE_DELAY_FRAMES + 10)  # the restore is due...
    trace = self.drive(car_, 60, _lp_sp(), DRIVER_ADJUST_FRAMES, buttons_at=press)  # ...but the driver pressed
    assert all(state == State.inactive for _, _, state in trace[1:DRIVER_ADJUST_FRAMES - 10])
