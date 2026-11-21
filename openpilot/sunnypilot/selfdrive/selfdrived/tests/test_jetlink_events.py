"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

selfdrived's Jetlink events over jetlink's own swap-event logic (jetlink/openpilot/events.py): what the driver is
told and what is held back when the big model joins or hands back. The safety properties pinned here:
  - losing the big model while in control is a warning, never a disengagement;
  - the swap's no-entry second is a NO_ENTRY only, and a MADS enable in it waits in paused;
  - only the services a switch delays are held, and only for the settling second;
  - ICBM never raises the set speed while the warning shows.
"""
import unittest
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import ET, Events, EventName
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.jetlink_events import JetlinkEvents, SWITCH_SERVICES

EventNameSP = custom.OnroadEventSP.EventName

try:
  from jetlink.openpilot.events import SwapEvents, HANDBACK_S, OFFER_S, SWITCHING_S
  HAVE_JETLINK = True
except ImportError:
  HAVE_JETLINK = False


class _Tracker:
  """jetlink's SwapEvents with the state the runtime record would give, as hooks.SwapTracker drives it."""

  def __init__(self):
    self.events = SwapEvents(DT_CTRL)
    self.state = 'joining'

  @property
  def settling(self):
    return self.events.settling

  def update(self, fresh, big, in_control):
    return self.events.update(fresh, big, in_control, self.state)


class _Model:
  def __init__(self, big):
    self.big = big


class _SM:
  """What JetlinkEvents reads of selfdrived's SubMaster: modelV2's health and big flag, and the checks per service."""

  def __init__(self, services=('modelV2', 'carState', 'deviceMotion', 'pandaStates')):
    self.services = list(services)
    self.ok = dict.fromkeys(self.services, True)
    self.big = False
    self.seen = dict.fromkeys(self.services, True)

  @property
  def alive(self):
    return {'modelV2': self.ok['modelV2']}

  @property
  def valid(self):
    return {'modelV2': self.ok['modelV2']}

  def __getitem__(self, s):
    assert s == 'modelV2'
    return _Model(self.big)

  def all_checks(self, service_list=None):
    return all(self.ok[s] for s in (service_list or self.services))


@unittest.skipUnless(HAVE_JETLINK, "no jetlink checkout")
class TestJetlinkEvents(OpenpilotTestCase):
  def setup_method(self):
    self.je = JetlinkEvents()
    self.je.swap = _Tracker()
    self.sm = _SM()

  def _tick(self, in_control, n=1):
    for _ in range(n):
      events, events_sp = Events(), EventsSP()
      self.je.update(self.sm, in_control, events, events_sp)
    return events, events_sp

  def _drive_big(self, in_control=True, seconds=3.):
    """The big model swapped in while disengaged, then the driver engaged."""
    self.je.swap.state = 'running'
    self.sm.big = True
    self._tick(False, int(SWITCHING_S / DT_CTRL) + 5)
    return self._tick(in_control, int(seconds / DT_CTRL))

  def test_offer_only_while_in_control(self):
    """Ready while steering: the driver is offered the switch (shown OFFER_S). Ready with nothing in control: no offer,
    jetlink swaps at once."""
    self.je.swap.state = 'ready'
    _, events_sp = self._tick(True)
    assert events_sp.has(EventNameSP.bigModelAvailable)
    idle = JetlinkEvents()
    idle.swap = _Tracker()
    idle.swap.state = 'ready'
    for _ in range(int(OFFER_S / DT_CTRL) + 5):
      events, events_sp = Events(), EventsSP()
      idle.update(self.sm, False, events, events_sp)
      assert not events_sp.has(EventNameSP.bigModelAvailable)

  def test_swap_is_a_no_entry_then_a_chime(self):
    self.je.swap.state = 'running'
    self.sm.big = True
    events, _ = self._tick(False)
    assert events.has(EventName.bigModelLoading) and self.je.no_entry
    assert events.contains(ET.NO_ENTRY) and not events.contains(ET.SOFT_DISABLE) and not events.contains(ET.IMMEDIATE_DISABLE)
    chimed = False
    for _ in range(int(SWITCHING_S / DT_CTRL) + 5):
      events, events_sp = self._tick(False)
      chimed |= events_sp.has(EventNameSP.bigModelReady)
    assert chimed and not events.has(EventName.bigModelLoading) and not self.je.no_entry

  def test_link_lost_while_in_control_is_a_warning_only(self):
    self._drive_big()
    # the link drops: modelV2.big goes false while openpilot steers
    self.sm.big = False
    events, events_sp = self._tick(True)
    assert events_sp.has(EventNameSP.bigModelLinkLost) and self.je.link_lost
    for evs in (events, events_sp):
      assert not evs.contains(ET.SOFT_DISABLE) and not evs.contains(ET.IMMEDIATE_DISABLE) and not evs.contains(ET.NO_ENTRY)
    assert events_sp.contains(ET.WARNING)
    # held for HANDBACK_S, then gone
    ticks = 1
    while self.je.link_lost and ticks < 2 * HANDBACK_S / DT_CTRL:
      self._tick(True)
      ticks += 1
    assert abs(ticks * DT_CTRL - HANDBACK_S) < 0.05

  def test_settling_holds_only_the_switch_services_for_a_second(self):
    self._drive_big()
    self.sm.big = False
    self._tick(True)
    assert self.je.settling
    # the switch's dropped frame: modelV2 and what follows it late
    self.sm.ok['modelV2'] = self.sm.ok['deviceMotion'] = False
    assert self.je.comm_ok(self.sm)
    # anything else failing is never held
    self.sm.ok['carState'] = False
    assert not self.je.comm_ok(self.sm)
    self.sm.ok['carState'] = True
    # and not past the settling second: a dead model still soft-disables
    self.sm.ok['modelV2'] = True
    self._tick(True, int(SWITCHING_S / DT_CTRL) + 5)
    self.sm.ok['modelV2'] = False
    assert not self.je.settling and not self.je.comm_ok(self.sm)

  def test_switch_services_are_model_downstream_only(self):
    for service in ('carState', 'carControl', 'controlsState', 'pandaStates', 'carOutput', 'deviceState',
                    'driverMonitoringState', 'managerState', 'radarState', 'carStateIC'):
      assert service not in SWITCH_SERVICES

  def test_no_jetlink_no_events(self):
    je = JetlinkEvents()
    je.swap = None
    events, events_sp = Events(), EventsSP()
    je.update(self.sm, True, events, events_sp)
    assert len(events) == 0 and len(events_sp) == 0 and not je.settling and not je.comm_ok(self.sm)


@unittest.skipUnless(HAVE_JETLINK, "no jetlink checkout")
class TestJetlinkScenarios(OpenpilotTestCase):
  """The swap events over a drive: who is warned when, and what ICBM may do meanwhile."""

  def setup_method(self):
    self.je = JetlinkEvents()
    self.je.swap = _Tracker()
    self.sm = _SM()

  def tick(self, in_control, n=1):
    seen = set()
    for _ in range(n):
      events, events_sp = Events(), EventsSP()
      self.je.update(self.sm, in_control, events, events_sp)
      seen |= {('sp', e) for e in events_sp.names} | {('op', e) for e in events.names}
    return seen

  def swap_in(self):
    """The big model joins while nothing is in control, then the driver engages after the no-entry second."""
    self.je.swap.state = 'running'
    self.sm.big = True
    self.tick(False, int(SWITCHING_S / DT_CTRL) + 5)

  def hand_back(self, in_control):
    self.sm.big = False
    self.je.swap.state = 'retrying'
    return self.tick(in_control)

  def test_lost_under_mads_only_warns(self):
    """MADS lateral with cruise off is in control: the warning is just as needed."""
    self.swap_in()
    self.tick(True, 100)
    assert ('sp', EventNameSP.bigModelLinkLost) in self.hand_back(in_control=True)

  def test_lost_while_disengaged_does_not_warn(self):
    self.swap_in()
    assert ('sp', EventNameSP.bigModelLinkLost) not in self.hand_back(in_control=False)
    assert not self.je.link_lost

  def test_every_hand_back_warns(self):
    for _ in range(3):
      self.swap_in()
      self.tick(True, 200)
      assert ('sp', EventNameSP.bigModelLinkLost) in self.hand_back(in_control=True)
      self.tick(True, int(HANDBACK_S / DT_CTRL) + 5)
      self.tick(False, 10)

  def test_no_offer_over_the_warning(self):
    """A link that is ready again within the warning is not offered over it: once the warning is over."""
    self.swap_in()
    self.tick(True, 100)
    self.hand_back(in_control=True)
    self.je.swap.state = 'ready'
    during = self.tick(True, int(HANDBACK_S / DT_CTRL) - 20)
    assert ('sp', EventNameSP.bigModelAvailable) not in during
    after = self.tick(True, 50)
    assert ('sp', EventNameSP.bigModelAvailable) in after

  def test_the_real_hooks_with_the_link_off_raise_nothing(self):
    je = JetlinkEvents()  # jetlink_adapter's own tracker, the setting off in this test's params store
    assert je.swap is not None
    for _ in range(200):
      events, events_sp = Events(), EventsSP()
      je.update(self.sm, True, events, events_sp)
      assert len(events) == 0 and len(events_sp) == 0
    assert not je.settling and not je.link_lost and not je.no_entry

  def test_icbm_holds_its_raise_over_the_warning(self):
    """End to end: the warning's flag is what selfdrived hands ICBM; the target rising under it is not followed until
    the warning is over, lowering is."""
    from opendbc.car.structs import car as car_structs
    from opendbc.car.volkswagen.values import VolkswagenFlags
    from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
    from openpilot.sunnypilot.selfdrive.car.tests.test_icbm_hold_set_speed import _Params, _cc, _cs, _lp_sp
    from openpilot.sunnypilot.selfdrive.car.tests.test_icbm_restore import Car, PRESS_FRAMES
    CP = car_structs.CarParams(brand="volkswagen", pcmCruise=True, flags=int(VolkswagenFlags.MQB_EVO_GEN2))
    icbm = IntelligentCruiseButtonManagement(CP, custom.CarParamsSP(pcmCruiseSpeed=False), params=_Params(True))
    car_ = Car(40)

    def step(lp, in_control=True):
      events, events_sp = Events(), EventsSP()
      self.je.update(self.sm, in_control, events, events_sp)
      icbm.run(_cs(60, (), None, car_.cluster), _cc(), lp, is_metric=True, hold_raise=self.je.link_lost)
      car_.step(icbm)

    self.swap_in()
    for _ in range(200):
      step(_lp_sp(vision_kph=40))
    self.sm.big = False  # handed back mid-curve
    self.je.swap.state = 'retrying'
    for _ in range(int(HANDBACK_S / DT_CTRL) - 5):
      step(_lp_sp(vision_kph=55))  # the built-in model reads the bend as gentler
    assert car_.ups == 0 and car_.cluster == 40
    for _ in range(PRESS_FRAMES * 4):
      step(_lp_sp(vision_kph=55))
    assert car_.ups >= 2  # the warning is over and the wait already served


class TestMadsJetlinkPause(OpenpilotTestCase):
  def _state_machine(self, no_entry):
    from openpilot.sunnypilot.mads.state import StateMachine
    selfdrive = SimpleNamespace(events=Events(), events_sp=EventsSP(), state_machine=SimpleNamespace(current_alert_types=[]),
                                jetlink_events=SimpleNamespace(no_entry=no_entry))
    return StateMachine(SimpleNamespace(selfdrive=selfdrive)), selfdrive

  def test_jetlink_no_entry_pauses_mads(self):
    sm, selfdrive = self._state_machine(no_entry=True)
    selfdrive.events.add(EventName.bigModelLoading)
    assert sm.check_contains_in_list()

  def test_native_big_model_load_still_refuses(self):
    sm, selfdrive = self._state_machine(no_entry=False)
    selfdrive.events.add(EventName.bigModelLoading)
    assert not sm.check_contains_in_list()


if __name__ == "__main__":
  unittest.main()
