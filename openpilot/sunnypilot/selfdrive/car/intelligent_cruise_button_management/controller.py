"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers, CAR_SYNC_FRAMES, V_CRUISE_MAX, V_CRUISE_UNSET

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ButtonType = car.CarState.ButtonEvent.Type
DRIVER_CRUISE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.setCruise, ButtonType.resumeCruise)
# After the driver touches the stalk the car applies its own step and openpilot adopts the result
# (see VCruiseHelperSP.sync_v_cruise_with_car); ICBM stays quiet for the same window so it never fights that.
DRIVER_ADJUST_FRAMES = CAR_SYNC_FRAMES

# Never press below walking speed: on VW the '+' press is RESUME, which makes an ACC that is holding at a stop
# drive off. The car's own standstill flag is not reliable on every platform, so gate on speed as well.
STANDSTILL_SPEED = 2.0  # m/s, ~7 km/h
# Hold mode: wait before raising the set speed back up after a limiter clears, so a curve / limit that flickers
# for a second does not turn into a burst of -/+ presses. Lowering is never delayed. 1.5 s: replayed closed loop on
# 113 min of the owner's drives it cut the pick-up waiting by 10 s with no more flicker bursts than 2.0 s (1.0 s
# doubled them)
RESTORE_DELAY_FRAMES = int(1.5 / DT_CTRL)
# The wait counts from the last lowering demand (the target under the car's set speed) or the last real drop of the
# target: at least this far (display units) under its peak since. Not from the moment the set speed catches up with a
# target that is still rising: leaving a curve the target rises in small steps, and a wait at every step held the car
# under a target it had already been allowed to follow (logged: a quarter of every pick-up spent waiting).
RESTORE_DROP_TOLERANCE = 2

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params | None = None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.params = params or Params()
    # Hold Set Speed: track the driver's set speed lowered only by the limiters (SCC / SLA), instead of the
    # planner's MPC speed. Fewer button presses, the stock ACC keeps its own acceleration and lead following,
    # and the car is never asked for more than the driver set.
    self.hold_set_speed = True
    self.driver_adjust_frames = 0
    self.raise_frames = 0
    self.v_target_peak = 0  # the highest target since the last drop, for RESTORE_DROP_TOLERANCE
    # set by selfdrived for the seconds after the Jetlink big model is lost while in control: lower, never raise
    self.raise_hold = False
    self.read_params()

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER.copy()  # module-level dict; must not be aliased

  def read_params(self) -> None:
    self.hold_set_speed = bool(self.params.get("IcbmHoldSetSpeed", return_default=True))

  def limiter_target_ms(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP, force_decel: bool) -> float:
    # Same selection as LongitudinalPlannerSP.update_targets (min of set speed and limiter outputs), without the MPC.
    if force_decel:
      return 0.
    if not (0 < CS.vCruise < V_CRUISE_UNSET):
      return CS.cruiseState.speedCluster  # nothing valid to track: hold whatever the car has
    scc = LP_SP.smartCruiseControl
    # inactive limiters publish the V_CRUISE_UNSET sentinel; a 0 (message not received yet) must never pull the set speed down
    limits = [v for v in (scc.vision.vTarget, scc.map.vTarget, LP_SP.speedLimit.assist.vTarget) if v > 0]
    return min([min(CS.vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS] + limits)

  @property
  def restore_allowed(self) -> bool:
    if self.raise_hold:
      return False
    return not self.hold_set_speed or self.raise_frames >= RESTORE_DELAY_FRAMES

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP, force_decel: bool = False) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    v_target_ms = self.limiter_target_ms(CS, LP_SP, force_decel) if self.hold_set_speed else LP_SP.vTarget
    self.v_target_ms_last = apply_hysteresis(v_target_ms, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric, self.CP)
    self.update_restore_timer()
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_restore_timer(self) -> None:
    """raise_frames: how long a raise has been wanted since the last lowering demand or drop of the target. Caught up
    with the target, it holds its count, so the next step of a rising target is followed without a new wait."""
    dropped = self.v_target <= self.v_target_peak - RESTORE_DROP_TOLERANCE
    self.v_target_peak = self.v_target if dropped else max(self.v_target_peak, self.v_target)
    if dropped or self.v_target < self.v_cruise_cluster:
      self.raise_frames = 0
    elif self.v_target > self.v_cruise_cluster:
      self.raise_frames += 1

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster and self.restore_allowed:
              self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster or self.raise_hold:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    if any(b.type.raw in DRIVER_CRUISE_BUTTONS for b in CS.buttonEvents):
      self.driver_adjust_frames = DRIVER_ADJUST_FRAMES
    elif self.driver_adjust_frames > 0:
      self.driver_adjust_frames -= 1

    moving = CS.vEgo >= STANDSTILL_SPEED and not CS.cruiseState.standstill

    self.is_ready = ready and moving and not button_pressed and self.driver_adjust_frames == 0

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool, force_decel: bool = False,
          hold_raise: bool = False) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric
    self.raise_hold = hold_raise

    self.update_calculations(CS, LP_SP, force_decel)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
