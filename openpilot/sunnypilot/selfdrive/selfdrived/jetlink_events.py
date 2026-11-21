"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

selfdrived's side of Jetlink (openpilot/sunnypilot/jetlink_adapter): the onroad events of a big model that joins
mid-drive, read off jetlink's swap events (jetlink/openpilot/events.py), and what a model switch may hold back.

The big model swaps in only while nothing is in control (openpilot or MADS, MADS paused included), so the driver is
told once it is ready (bigModelAvailable) and re-engages to use it; for the second after a swap nothing engages
(bigModelLoading, NO_ENTRY) and a chime says when it can. If it falls behind or the link drops while in control, the
built-in model drives on from the same frame and the driver is warned (bigModelLinkLost, a WARNING): nothing
disengages.

A switch costs modeld a frame or two. That frame's lateness reaches selfdrived as a late modelV2 and everything
computed from it (a localizer, paramsd, lagd and planner hiccup), which would otherwise raise a soft disable on the
next tick. For the second after a switch, `settling` holds only those (SWITCH_SERVICES): zoompilot holds every
service's comm check for that second, this holds only what a switch can delay, so CAN, carState, controls and every
other service keep theirs. A model outage longer than the second is not held.
"""
from openpilot.cereal import custom
import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName

# what a model switch's dropped frame delays: modeld's output and the services computed from it
SWITCH_SERVICES = frozenset({'modelV2', 'modelDataV2SP', 'deviceMotion', 'vehicleParameters', 'lateralDelay',
                             'longitudinalPlan', 'longitudinalPlanSP', 'driverAssistance',
                             'lateralTorqueParameters', 'lateralCurvatureParameters'})


class JetlinkEvents:
  # the gadget owner is optional: its exit costs the big model, never engagement. selfdrived leaves it out of
  # processNotRunning
  OPTIONAL_PROCESSES = frozenset({jetlink_adapter.OWNER})

  def __init__(self):
    # jetlink's tracker behind the hooks' net; None without jetlink. Built once, link on or off: it binds nothing
    self.swap = jetlink_adapter.swap_events(DT_CTRL)
    # this tick's flags, for selfdrived's other readers (MADS, ICBM)
    self.link_lost = False
    self.no_entry = False

  @property
  def settling(self) -> bool:
    """Within a second of the big model swapping in or handing back."""
    return self.swap is not None and self.swap.settling

  def comm_ok(self, sm: messaging.SubMaster) -> bool:
    """While settling: are all services but the ones a switch delays healthy?"""
    others = [s for s in sm.services if s not in SWITCH_SERVICES]
    return bool(self.settling and others and sm.all_checks(others))

  def update(self, sm: messaging.SubMaster, in_control: bool, events: Events, events_sp: EventsSP) -> None:
    """`in_control`: openpilot or MADS is engaged, MADS even while its lateral is paused: the rule the adapter's
    swap gate waits on (Adapter.engaged)."""
    self.link_lost = self.no_entry = False
    if self.swap is None:
      return
    fresh = sm.seen['modelV2'] and sm.alive['modelV2'] and sm.valid['modelV2']
    big = bool(sm.alive['modelV2'] and sm.valid['modelV2'] and sm['modelV2'].big)
    # no state of this fork's own on a message: the tracker reads modeld's runtime record
    flags = self.swap.update(fresh, big, in_control)
    self.link_lost, self.no_entry = flags.link_lost, flags.no_entry
    if flags.offer:
      events_sp.add(EventNameSP.bigModelAvailable)
    if flags.link_lost:
      events_sp.add(EventNameSP.bigModelLinkLost)
    if flags.no_entry or flags.chime:
      # the chime waits for the end of the no-entry second, when it means the driver can engage
      events_sp.remove(EventNameSP.bigModelReady)
    if flags.no_entry:
      # upstream's no-entry for a big model that is not ready to drive: native, as the main state machine reads
      # only native events
      events.add(EventName.bigModelLoading)
    if flags.chime:
      events_sp.add(EventNameSP.bigModelReady)
