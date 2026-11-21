"""
Jetlink on this fork: the adapter jetlink runs over, and the hooks manager, modeld, selfdrived, hardwared and the
UI call.

jetlink (the jetlink_repo submodule, https://github.com/zoompilot/jetlink) runs openpilot's large driving model on
a computer or phone connected to the comma by USB: a Jetson, a Linux PC with an NVIDIA GPU, a Mac, an Android phone
(mode 'usb') or an iPhone/iPad (mode 'ios'). Its stock adapter (jetlink.openpilot.stock) knows stock openpilot's
paths; Adapter below is that with this fork's MADS and modeld_v2 constants.

Modelled on zoompilot's openpilot/sunnypilot/jetlink_adapter (MIT, Zeph Leggett), with these differences:
  - no model-manager catalog integration: the big model is picked in the Jetlink app or on the host, so nothing is
    fetched from jetlink's model index while the link is off;
  - no cable-as-chestnut guess: this base's modeld only runs a chestnut that names itself (chestnut_present()), so a
    Jetson or a phone on the USB-C port can never be mistaken for one;
  - the hook functions are written out, not unpacked from a dict's order;
  - JetlinkLink is not BACKUP (params_keys.h): a restored backup never switches the link on on another comma.

Every hook answers as the link off when jetlink is missing, too old, or fails: a jetlink bug never takes manager,
modeld, selfdrived, hardwared or the UI down (jetlink.openpilot.hooks).

manager imports this module to build its process list and runs it as jetlinkd, the resident USB gadget owner. So
the top level is the standard library only; everything else is imported where it is used. tests/test_adapter.py
holds the line.
"""
from __future__ import annotations

import functools
import os
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other is treated as jetlink being absent,
# with the reason as the offroad alert
API = 4

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index (jetlink.openpilot.MODES), written out so the UI builds the setting
# without a jetlink checkout
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in params_keys.h. No big-model slot or catalog: jetlink runs the
# model picked on the host, or its own default
_Keys = namedtuple('_Keys', 'link offroad progress spec charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='JetlinkProgress', spec='JetlinkSpec',
             charge_phone='JetlinkChargePhone')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT, else /data/params on a device and
  $HOME/.comma<OPENPILOT_PREFIX>/params elsewhere, then /<OPENPILOT_PREFIX, or d>. Per call, so it follows a
  test's prefix as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.environ.get('HOME', '') + '/.comma' + prefix + '/params'
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def link_on() -> bool:
  """Is the Jetlink setting on? Read as a file, as jetlink reads it, so it works with no jetlink at all."""
  try:
    return 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
  except (OSError, ValueError):
    return False


def link_mode() -> str:
  """The setting by name: 'off', 'usb' or 'ios'."""
  try:
    mode = int((_params_dir() / KEYS.link).read_bytes())
  except (OSError, ValueError):
    return MODES[0]
  return MODES[mode] if 0 <= mode < len(MODES) else MODES[0]


@functools.cache
def _adapter_class():
  from jetlink.openpilot.interface import Keys
  from jetlink.openpilot.stock import Stock

  class Adapter(Stock):
    """jetlink.openpilot.interface.Openpilot over this fork: stock openpilot but for what follows."""
    KEYS = Keys(**KEYS._asdict())
    # MADS's state rides on carControlSP. carState stays, so a dead card counts as in control
    IN_CONTROL_SERVICES = ('carState', 'carControl', 'carControlSP')

    def chestnut_present(self) -> bool:
      # modeld's own test, the one it runs the big model on: a chestnut whose firmware names itself
      from openpilot.selfdrive.modeld.helpers import chestnut_present
      return chestnut_present()

    def model_constants(self):
      # modeld_v2 (modeld_tinygrad) reads its ModelConstants off the model: MODEL_FREQ, DESIRE_LEN, T_IDXS
      from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
      return ModelConstants

    def engaged(self, sm) -> bool:
      # selfdrived's enabled or mads.enabled, as controlsd republishes them. MADS counts with its lateral paused
      # (a stop, the brake, a blinker): it steers again on its own, so the model must never swap under it
      return bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)

  return Adapter


def adapter():
  """The adapter, for jetlink's entry points that run as their own process: the provisioning run."""
  return _adapter_class()()


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  return _adapter_class().owner_config()


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  _adapter_class().main()


# -- the hooks -----------------------------------------------------------------------------------------------------

# this process's hooks, made on first use and kept, the link off included: Python does not cache a failed import.
# One per process, so that prepare() and attach() reach the same jetlink
_hooks = None
_making = threading.Lock()


def hooks():
  """jetlink.openpilot.hooks.Hooks over this adapter, or _Off."""
  global _hooks
  with _making:
    if _hooks is None:
      _hooks = _guard()
    return _hooks


def _guard():
  # inside a function, so a jetlink that fails to import lands here too
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Off()   # an empty jetlink_repo, which Python takes for a namespace package
    from jetlink.openpilot.hooks import Hooks
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Off()   # no checkout: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  try:
    return Hooks(API, adapter, link_on=link_on, log=_cloudlog)
  except Exception as e:
    return _unusable(str(e), e)   # another API, which it names


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


def _unusable(why: str, error: Exception) -> _Off:
  try:
    _cloudlog().error("jetlink: %s", why, exc_info=error)
  except Exception:
    pass
  return _Off(why)


class _Off:
  """Every hook with the link off, for a jetlink that is missing, too old or another API; reason() says why to
  someone who turned the link on. Written out hook by hook, each with the answer jetlink's own Off gives."""

  def __init__(self, why: str = ''):
    self.why = why

  def should_run(self, started, params, CP) -> bool:
    return False

  def status(self):
    return None

  def reason(self) -> str | None:
    return self.why if self.why and link_on() else None

  def prepare(self, chestnut: bool) -> bool:
    return False

  def attach(self, model, small, cam_w: int, cam_h: int):
    return model

  def handed_over(self, model) -> bool:
    return False

  def state(self, model) -> str:
    return 'none'

  def runtime(self):
    return None

  def shutdown_step(self, should_shutdown: bool, reason: str = '') -> bool:
    # a wanted power-off goes ahead: no jetlink must ever keep a comma on to drain the car's battery
    return should_shutdown

  def request_shutdown(self, reason: str = '') -> bool:
    return False

  def shutdown_pending(self) -> bool:
    return False

  def model_state(self, ref: str) -> str | None:
    return None

  def lists_big_models(self) -> bool:
    return False

  def big_catalog(self, selector: int) -> dict | None:
    return None

  def listed_ref(self, ref: str) -> str | None:
    return None

  def should_extend_catalog(self) -> bool:
    return False

  def extend_catalog(self, catalog: dict) -> dict:
    return catalog

  def swap_events(self, tick_s: float):
    return None


# the hook lines call these, each this process's hooks() at the call (jetlink.openpilot.hooks says what each does)

def should_run(started, params, CP) -> bool:
  """manager's should_run for jetlinkd."""
  return hooks().should_run(started, params, CP)


def status():
  """The UI's snapshot (jetlink.openpilot.status.Status), or None when jetlink cannot run at all."""
  return hooks().status()


def reason() -> str | None:
  """Why the link the user turned on cannot run, for the offroad alert; None with the link off."""
  return hooks().reason()


def prepare(chestnut: bool) -> bool:
  """modeld, before config_realtime_process."""
  return hooks().prepare(chestnut)


def attach(model, small, cam_w: int, cam_h: int):
  """modeld, once `small` is built: the model to run."""
  return hooks().attach(model, small, cam_w, cam_h)


def handed_over(model) -> bool:
  """modeld, after every run(): did it change which model drives?"""
  return hooks().handed_over(model)


def state(model) -> str:
  """The joining model's state, one of jetlink.openpilot.STATES."""
  return hooks().state(model)


def runtime():
  return hooks().runtime()


def shutdown_step(should_shutdown: bool, reason: str = '') -> bool:
  """hardwared, every loop: may the comma power off now?"""
  return hooks().shutdown_step(should_shutdown, reason)


def swap_events(tick_s: float):
  """selfdrived's swap-event tracker (jetlink.openpilot.hooks.SwapTracker), or None without jetlink."""
  return hooks().swap_events(tick_s)


def small_takes_over(joined, small, bufs, transforms, inputs):
  """modeld, when the joined model's run() raised: Jetlink falls back to the small model itself on a lost or lagging
  link, so this is an error that escaped its own net. modeld must not die of it: the small model runs this very
  frame (and modeld keeps it for the rest of the drive), and the joined model is let go off the frame loop, as its
  close() waits on its threads. An error from the small model itself is modeld's, as without Jetlink."""
  try:
    _cloudlog().exception("jetlink: the joined model failed outside its own fallback, the small model drives on")
  except Exception:
    pass
  threading.Thread(target=_close_quietly, args=(joined,), name="jetlink_close", daemon=True).start()
  return small.run(bufs, transforms, inputs, None)


def _close_quietly(joined) -> None:
  try:
    joined.close()
  except Exception:
    pass


# the hooks the fork's lines call, for tests/test_adapter.py's check_fallback
HOOK_NAMES = ('should_run', 'status', 'reason', 'prepare', 'attach', 'handed_over', 'state', 'runtime',
              'shutdown_step', 'swap_events')
