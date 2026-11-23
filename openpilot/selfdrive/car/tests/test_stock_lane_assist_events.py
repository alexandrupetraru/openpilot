"""
The car's own Lane Assist while openpilot steers (VW MQB Evo Gen 2). On the 2026-11-23 night drive it judged the car to
be on a line on a wet road, its request reached the EPS over Ethernet, and the EPS unwound the wheel out of a curve
while still reporting openpilot's HCA as active. These events tell the driver: active -> turn it off, steering -> take
control.
"""
from opendbc.car.structs import car
from openpilot.cereal import custom, log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.car_events import CarEvents, STOCK_LANE_ASSIST_ONSET_FRAMES
from openpilot.selfdrive.selfdrived.events import EVENTS, ET

EventName = log.OnroadEvent.EventName
OURS = (EventName.stockLaneAssistActive, EventName.stockLaneAssistIntervening)


class TestStockLaneAssistEvents(OpenpilotTestCase):
  def setup_method(self):
    CP = car.CarParams.new_message(brand='volkswagen', carFingerprint='VOLKSWAGEN_TIGUAN_MK3', pcmCruise=True)
    self.events = CarEvents(CP)
    self.CS = car.CarState.new_message()

  def step(self, lat_active, active, intervening=False):
    CC = car.CarControl.new_message(latActive=lat_active)
    CS_IC = custom.CarStateIC.new_message(stockLaneAssistActive=active, stockLaneAssistIntervening=intervening)
    return [e for e in self.events.update(self.CS, self.CS, CC, CS_IC).names if e in OURS]

  def test_active_is_flagged_after_the_onset(self):
    for _ in range(STOCK_LANE_ASSIST_ONSET_FRAMES - 1):
      assert self.step(True, True) == []
    assert self.step(True, True) == [EventName.stockLaneAssistActive]
    assert self.step(True, True) == [EventName.stockLaneAssistActive]

  def test_a_flicker_restarts_the_onset(self):
    for _ in range(STOCK_LANE_ASSIST_ONSET_FRAMES - 1):
      self.step(True, True)
    assert self.step(True, False) == []  # back to standby around its switch-on speed
    for _ in range(STOCK_LANE_ASSIST_ONSET_FRAMES - 1):
      assert self.step(True, True) == []

  def test_steering_now_is_flagged_at_once_and_alone(self):
    assert self.step(True, True, intervening=True) == [EventName.stockLaneAssistIntervening]
    for _ in range(2 * STOCK_LANE_ASSIST_ONSET_FRAMES):
      assert self.step(True, True, intervening=True) == [EventName.stockLaneAssistIntervening]
    # done reacting, still active: the reminder, its onset already served
    assert self.step(True, True) == [EventName.stockLaneAssistActive]

  def test_nothing_while_openpilot_does_not_steer(self):
    for _ in range(2 * STOCK_LANE_ASSIST_ONSET_FRAMES):
      assert self.step(False, True) == []
      assert self.step(False, True, intervening=True) == []
    # and openpilot taking over the wheel starts the onset afresh
    for _ in range(STOCK_LANE_ASSIST_ONSET_FRAMES - 1):
      assert self.step(True, True) == []

  def test_alerts_show_while_steering(self):
    """WARNING alerts: shown while openpilot or MADS steers, which is when the events are raised. Steering now outranks
    the reminder and the steering-limit alert, and asks for the wheel."""
    active = EVENTS[EventName.stockLaneAssistActive][ET.WARNING]
    steering = EVENTS[EventName.stockLaneAssistIntervening][ET.WARNING]
    saturated = EVENTS[EventName.steerSaturated][ET.WARNING]
    assert steering.priority > active.priority and steering.priority > saturated.priority
    assert steering.visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    assert steering.audible_alert == log.SelfdriveState.AudibleAlert.warningSoft
