import itertools
import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized_class
from openpilot.cereal import log
from openpilot.selfdrive.car.cruise import (
  CRUISE_BUTTON_STUCK_FRAMES, CRUISE_LONG_PRESS, IMPERIAL_INCREMENT, PREDICTIVE_TYPE_CURVE, PREDICTIVE_TYPE_SPEED_LIMIT, VCruiseHelper,
  V_CRUISE_INITIAL, V_CRUISE_MAX, V_CRUISE_MIN,
)
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.cruise_ext import ICBM_PRESS_HOLDOFF_FRAMES
from opendbc.car.structs import car, CarStateIC
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed(OpenpilotTestCase):
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    self.assertAlmostEqual(simulation_steady_state, cruise_speed, delta=.01, msg=f'Did not reach {self.speed} m/s')


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestVCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=self.pcm_cruise_speed)
    self.CS_IC = CarStateIC()
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), self.CS_IC, enabled=False, is_metric=False)

  def enable(self, v_ego, experimental_mode, dynamic_experimental_control):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode, dynamic_experimental_control)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=False)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=enabled, is_metric=False)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert (not pressed) == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=False)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts pressing set while enabled with gas pressed sets
    the speed to the maximum of vEgo and current cruise speed.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # first decrement speed, then perform gas pressed logic
      expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego * CV.MS_TO_KPH)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=False)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for dynamic_experimental_control in (True, False):
        for v_ego in np.linspace(0, 100, 101):
          self.reset_cruise_speed_state()
          assert not self.v_cruise_helper.v_cruise_initialized

          self.enable(float(v_ego), experimental_mode, dynamic_experimental_control)
          assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
          assert self.v_cruise_helper.v_cruise_initialized

  def test_missed_release_does_not_repeat_forever(self):
    """
    A press whose release event is never seen must stop synthesising long-press speed changes.
    """
    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=True)]
    self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=True)
    CS.buttonEvents = []

    # while the press is plausibly still held, long-press repeats are expected
    v_start = self.v_cruise_helper.v_cruise_kph
    for _ in range(CRUISE_LONG_PRESS * 2):
      self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=True)
    assert self.v_cruise_helper.v_cruise_kph > v_start
    assert self.v_cruise_helper.button_timers[ButtonType.accelCruise] > 0

    # past the stuck threshold the timer is dropped and the set speed is left alone
    for _ in range(CRUISE_BUTTON_STUCK_FRAMES):
      self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=True)
    assert self.v_cruise_helper.button_timers[ButtonType.accelCruise] == 0
    v_settled = self.v_cruise_helper.v_cruise_kph
    for _ in range(CRUISE_LONG_PRESS * 4):
      self.v_cruise_helper.update_v_cruise(CS, self.CS_IC, enabled=True, is_metric=True)
    assert self.v_cruise_helper.v_cruise_kph == v_settled

  def test_curve_prediction_is_a_cap_not_a_setpoint(self):
    self.v_cruise_helper.v_cruise_kph = 70
    self.CS_IC.cruiseSpeedLimit = 130 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicative = 80 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicativeType = PREDICTIVE_TYPE_CURVE

    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 70

    # An interface mismatch with a missing type must fail safe as a cap.
    self.v_cruise_helper._clear_curve_speed_cap()
    self.CS_IC.cruiseSpeedLimitPredicativeType = 0
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 70

  def test_curve_cap_opens_only_to_pre_curve_setpoint(self):
    self.v_cruise_helper.v_cruise_kph = 100
    self.CS_IC.cruiseSpeedLimit = 130 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicativeType = PREDICTIVE_TYPE_CURVE

    self.CS_IC.cruiseSpeedLimitPredicative = 80 * CV.KPH_TO_MS
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 80

    self.CS_IC.cruiseSpeedLimitPredicative = 90 * CV.KPH_TO_MS
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 90

    self.CS_IC.cruiseSpeedLimitPredicative = 0
    self.CS_IC.cruiseSpeedLimitPredicativeType = 0
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 100

  def test_predicative_speed_limit_keeps_setpoint_semantics(self):
    self.v_cruise_helper.v_cruise_kph = 70
    self.CS_IC.cruiseSpeedLimit = 130 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicative = 80 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicativeType = PREDICTIVE_TYPE_SPEED_LIMIT

    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 80

  def test_equal_value_speed_limit_replaces_curve_cap_semantics(self):
    self.v_cruise_helper.v_cruise_kph = 70
    self.CS_IC.cruiseSpeedLimit = 130 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicative = 80 * CV.KPH_TO_MS
    self.CS_IC.cruiseSpeedLimitPredicativeType = PREDICTIVE_TYPE_CURVE
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 70

    # The numeric target is unchanged, but the source is now a speed limit and
    # must recover normal setpoint semantics.
    self.CS_IC.cruiseSpeedLimitPredicativeType = PREDICTIVE_TYPE_SPEED_LIMIT
    self.v_cruise_helper._update_v_speed_limit(None, self.CS_IC, True, True, True)
    assert self.v_cruise_helper.v_cruise_kph == 80


@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(True, False)])
class TestVCruiseHelperIcbm(OpenpilotTestCase):
  """Stock cruise owns the set speed; openpilot only nudges it through the buttons (Intelligent Cruise Button Management)."""

  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=self.pcm_cruise_speed)
    self.CS_IC = CarStateIC()
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

  def _cs(self, car_set_kph, buttons=()):
    v = car_set_kph * CV.KPH_TO_MS
    CS = car.CarState(cruiseState={"available": True, "enabled": True, "speed": v, "speedCluster": v})
    CS.buttonEvents = list(buttons)
    return CS

  def _step(self, car_set_kph, buttons=(), enabled=True, n=1):
    for _ in range(n):
      self.v_cruise_helper.update_v_cruise(self._cs(car_set_kph, buttons), self.CS_IC, enabled=enabled, is_metric=True)

  def _engage(self, car_set_kph):
    self._step(car_set_kph, enabled=False, n=2)   # mirrors the car while not engaged
    self._step(car_set_kph, n=3)                  # latches openpilot's own set speed on engage
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == car_set_kph

  def test_driver_press_adopts_the_cars_result(self):
    """The car applies its own step to a stalk press (this VW rounds '+' to the next 10 km/h); openpilot must not guess a delta."""
    self._engage(20)
    self._step(20, [ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    self._step(31)
    self._step(40)  # car reacts before the release, in two steps
    self._step(40, [ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    self._step(40, n=5)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 40             # not 21, and not clipped up to the ICBM floor

  def test_icbm_own_presses_are_not_adopted_during_sync(self):
    """While the sync window is open, a step the car makes in reaction to ICBM's own press is not the driver's choice."""
    self._engage(20)
    self._step(20, [ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    self._step(40)
    self._step(40, [ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    self._step(40, n=5)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 40
    send = custom.IntelligentCruiseButtonManagement.SendButtonState.decrease
    self.v_cruise_helper.update_v_cruise(self._cs(40), self.CS_IC, enabled=True, is_metric=True, icbm_send_button=send)
    for _ in range(10):
      self.v_cruise_helper.update_v_cruise(self._cs(39), self.CS_IC, enabled=True, is_metric=True)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 40             # ICBM's own -1 was not adopted as a new driver choice
    # It is a hold-off, not a window close: if a driver press ever races with an ICBM press, the car's reaction
    # to the driver must still be learned once the hold-off ends (ICBM itself pauses for the whole window).
    for _ in range(ICBM_PRESS_HOLDOFF_FRAMES):
      self.v_cruise_helper.update_v_cruise(self._cs(39), self.CS_IC, enabled=True, is_metric=True)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 39

  def test_speed_limit_assist_does_not_rewrite_the_set_speed(self):
    """With stock ACC the limit is a cap applied by ICBM; the driver's set speed stays the ceiling."""
    self._engage(70)
    LP_SP = custom.LongitudinalPlanSP()
    LP_SP.speedLimit.resolver.speedLimitValid = True
    LP_SP.speedLimit.resolver.speedLimitFinalLast = 50 * CV.KPH_TO_MS
    LP_SP.speedLimit.assist.state = custom.LongitudinalPlanSP.SpeedLimit.AssistState.active
    for _ in range(20):
      self.v_cruise_helper.update_speed_limit_assist(True, LP_SP)
      self._step(70)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 70

  def test_set_and_resume_presses_also_sync(self):
    self._engage(50)
    self._step(50, [ButtonEvent(type=ButtonType.setCruise, pressed=True)])
    self._step(32)
    self._step(32, [ButtonEvent(type=ButtonType.setCruise, pressed=False)])
    self._step(32, n=5)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 32

  def test_car_changes_without_a_press_do_not_move_openpilot(self):
    """Without a driver press openpilot stays authoritative, so ICBM can restore the set speed the car dropped by itself."""
    self._engage(40)
    self._step(40, n=CRUISE_LONG_PRESS * 4)  # well past any sync window
    self._step(35, n=10)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == 40

  def test_sync_respects_cruise_max(self):
    self._engage(140)
    self._step(140, [ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    self._step(150, [ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    self._step(150, n=5)
    assert round(self.v_cruise_helper.v_cruise_kph, 1) == V_CRUISE_MAX

