"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The Jetlink adapter against jetlink's own conformance kit (jetlink/openpilot/testing.py), and the link-off answers
every hook line relies on. The setting is written as a file, as jetlink reads it, into the test's own params store
(OpenpilotTestCase's prefix), so no test reaches the device's own params.
"""
import os
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot import jetlink_adapter

REPO = Path(__file__).resolve().parents[4]
MODELDS = ('openpilot/selfdrive/modeld/modeld.py', 'openpilot/sunnypilot/modeld_v2/modeld.py')

try:
  import jetlink
  HAVE_JETLINK = getattr(jetlink, '__file__', None) is not None
except ImportError:
  HAVE_JETLINK = False


def _set_link(value) -> None:
  # Params lays out the test's own store (a symlink on a comma) before a file is written into it
  from openpilot.common.params import Params
  Params()
  path = jetlink_adapter._params_dir() / jetlink_adapter.KEYS.link
  assert path.parent.is_dir(), path.parent
  if value is None:
    path.unlink(missing_ok=True)
  else:
    path.write_text(str(value))


class TestJetlinkSetting(OpenpilotTestCase):
  def test_link_on_reads_the_setting(self):
    assert os.environ.get('OPENPILOT_PREFIX'), "the test must run in its own params store"
    for value, on, mode in ((None, False, 'off'), (0, False, 'off'), (1, True, 'usb'), (2, True, 'ios'),
                            (3, False, 'off'), ('junk', False, 'off')):
      _set_link(value)
      assert jetlink_adapter.link_on() is on, value
      assert jetlink_adapter.link_mode() == mode, value

  def test_fallback_answers_as_the_link_off(self):
    """The fork's own hooks for a jetlink that is missing or too old: each answers as jetlink's Off does."""
    if not HAVE_JETLINK:
      raise unittest.SkipTest("no jetlink checkout")
    from jetlink.openpilot.testing import check_fallback
    _set_link(0)
    assert check_fallback(jetlink_adapter._Off('jetlink is missing'), jetlink_adapter.HOOK_NAMES, link_on=jetlink_adapter.link_on) == []
    _set_link(1)
    # the reason is told only to someone who turned the link on
    assert jetlink_adapter._Off('jetlink is missing').reason() == 'jetlink is missing'
    _set_link(0)
    assert jetlink_adapter._Off('jetlink is missing').reason() is None

  def test_no_link_host_is_ever_a_chestnut(self):
    """modeld runs its native big model only on a chestnut, and a chestnut that fails mid-drive is a soft disable.
    Nothing Jetlink puts on the port may ever look like one: not jetlink's own gadget, a Jetson's port or an iPhone."""
    from openpilot.common.hardware.usb import is_chestnut_usb_id
    for vid, pid in ((0x1209, 0x0001), (0x0955, 0x7020), (0x05AC, 0x12A8), (0x18D1, 0x4EE7)):
      assert not is_chestnut_usb_id(vid, pid), (hex(vid), hex(pid))

  def test_off_never_keeps_the_comma_on(self):
    off = jetlink_adapter._Off()
    assert off.shutdown_step(True, 'test') is True
    assert off.shutdown_step(False) is False

  def test_the_host_picks_the_model_while_the_link_runs(self):
    """Settings' Big Model list is not what runs with the link on: jetlink runs the computer's or phone's pick. A
    chestnut stands jetlink down, so its list counts again; so it does with the link off or unreadable."""
    for value, chestnut, picks in ((None, False, False), (0, False, False), ('junk', False, False),
                                   (1, False, True), (2, False, True), (1, True, False), (2, True, False)):
      _set_link(value)
      assert jetlink_adapter.host_picks_model(chestnut) is picks, (value, chestnut)

  def test_jetlink_is_handed_no_model_manager_pick(self):
    """What the Settings note rests on: none of the keys through which jetlink takes the model manager's pick
    (jetlink.openpilot.interface.Keys: big_model, catalog, select). Declaring one makes the comma's list a pick again,
    and host_picks_model and the note must change with it."""
    for key in ('big_model', 'catalog', 'select'):
      assert key not in jetlink_adapter.KEYS._fields, key


@unittest.skipUnless(HAVE_JETLINK, "no jetlink checkout")
class TestJetlinkAdapter(OpenpilotTestCase):
  def test_the_adapter(self):
    from jetlink.openpilot.testing import check_adapter
    assert check_adapter(jetlink_adapter, heavy=('openpilot.common.params', 'openpilot.common.swaglog', 'openpilot.cereal')) == []

  def test_the_modeld_seams(self):
    """Both modelds: prepare() before going realtime, attach() once the small model is built, its result run with
    frame_drop_ratio written, and handed_over() read."""
    from jetlink.openpilot.testing import check_seam
    for modeld in MODELDS:
      assert check_seam(REPO / modeld) == [], modeld

  def test_engaged_counts_mads_paused(self):
    """The swap gate: never swap the model while openpilot or MADS is in control, MADS paused included."""
    op = jetlink_adapter.adapter()

    def sm(enabled, mads):
      return {'carControl': SimpleNamespace(enabled=enabled), 'carControlSP': SimpleNamespace(mads=SimpleNamespace(enabled=mads))}
    assert op.engaged(sm(False, False)) is False
    assert op.engaged(sm(True, False)) is True
    assert op.engaged(sm(False, True)) is True
    assert 'carControlSP' in op.IN_CONTROL_SERVICES and 'carState' in op.IN_CONTROL_SERVICES

  def test_jetlink_sees_no_model_manager_pick(self):
    """Through jetlink's own interface: no big-model slot, catalog or select key, so it runs the host's pick or its
    default (host_picks_model)."""
    keys = jetlink_adapter.adapter().keys
    assert keys.big_model is None and keys.catalog is None and keys.select is None

  def test_hooks_with_the_link_off(self):
    """With the setting off every hook a hot path calls answers as no jetlink at all, without binding."""
    _set_link(0)
    hooks = jetlink_adapter.hooks()
    model, small = object(), object()
    assert hooks.prepare(False) is False
    assert hooks.attach(model, small, 1928, 1208) is model
    assert hooks.handed_over(model) is False
    assert hooks.state(model) == 'none'
    assert hooks.reason() is None
    assert hooks.should_run(False, None, None) is False
    # a power-off: no until one is wanted, then yes at once with nothing to ask, and from then on (it latches, as
    # DoShutdown is final)
    assert jetlink_adapter.shutdown_step(False) is False
    assert jetlink_adapter.shutdown_step(True, 'test') is True
    assert jetlink_adapter.shutdown_step(False) is True
    tracker = hooks.swap_events(0.01)
    flags = tracker.update(True, False, True)
    assert not any(flags) and not tracker.settling

  def test_keys_are_declared(self):
    """Every param the adapter names is declared, and the setting is not BACKUP (a restore must not turn it on)."""
    keys_h = (REPO / 'openpilot/common/params_keys.h').read_text()
    for key in (*jetlink_adapter.KEYS[:1], *jetlink_adapter.KEYS[2:], 'Offroad_JetlinkUnavailable', 'BigModelSmoothing'):
      assert f'{{"{key}", ' in keys_h, key
    line = next(ln for ln in keys_h.splitlines() if '{"JetlinkLink", ' in ln)
    assert 'BACKUP' not in line


class _Small:
  def __init__(self, fail=False):
    self.fail = fail
    self.calls = []

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    self.calls.append((bufs, transforms, inputs, after_enqueue))
    if self.fail:
      raise RuntimeError("small model failed")
    return {'plan': 'small'}


class _Joined:
  def __init__(self, close_fails=False):
    self.closed = threading.Event()
    self.close_fails = close_fails

  def close(self):
    self.closed.set()
    if self.close_fails:
      raise RuntimeError("close failed")


class TestSmallTakesOver(OpenpilotTestCase):
  """modeld's own net under Jetlink's: an error that escapes the joined model never takes modeld down."""

  def test_the_small_model_runs_the_same_frame(self):
    small, joined = _Small(), _Joined()
    bufs, tfms, inputs = {'img': 1}, {'img': 2}, {'desire_pulse': 3}
    assert jetlink_adapter.small_takes_over(joined, small, bufs, tfms, inputs) == {'plan': 'small'}
    # the frame's own inputs, and no after_enqueue: the joined model may already have called it
    assert small.calls == [(bufs, tfms, inputs, None)]
    assert joined.closed.wait(2.), "the joined model is let go"

  def test_a_close_that_fails_is_let_go(self):
    small, joined = _Small(), _Joined(close_fails=True)
    assert jetlink_adapter.small_takes_over(joined, small, {}, {}, {}) == {'plan': 'small'}
    assert joined.closed.wait(2.)

  def test_the_close_never_blocks_the_frame(self):
    class Slow(_Joined):
      def close(self):
        time.sleep(1.)
        super().close()
    start = time.monotonic()
    jetlink_adapter.small_takes_over(Slow(), _Small(), {}, {}, {})
    assert time.monotonic() - start < 0.2

  def test_the_small_model_s_own_error_is_modeld_s(self):
    with self.assertRaises(RuntimeError):
      jetlink_adapter.small_takes_over(_Joined(), _Small(fail=True), {}, {}, {})


if __name__ == "__main__":
  unittest.main()
