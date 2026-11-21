"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The driving warp Jetlink runs on the comma (compile_driving_warp.py): built here, read back through jetlink's own
loader (the checks that stand between a bad pickle and the car), run through jetlink's frame-loop Warp, and compared
byte for byte with modeld's own warp function on the same frames. On a comma this runs on the GPU (QCOM), as modeld
does; on a PC on the CPU.
"""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.common.test import OpenpilotTestCase

try:
  from jetlink.openpilot.warp import Warp, Warps
  HAVE_JETLINK = True
except ImportError:
  HAVE_JETLINK = False


@unittest.skipUnless(HAVE_JETLINK, "no jetlink checkout")
class TestDrivingWarp(OpenpilotTestCase):
  def test_the_warp_builds_loads_and_matches_modeld(self):
    from tinygrad.tensor import Tensor
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    from openpilot.selfdrive.modeld.compile_driving_warp import compile_driving_warp, make_driving_warp
    from openpilot.sunnypilot import jetlink_adapter

    cam_w, cam_h, model_w, model_h = jetlink_adapter.adapter().camera()
    assert (model_w, model_h) == tuple(MEDMODEL_INPUT_SIZE)
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / f"driving_warp_{cam_w}x{cam_h}_tinygrad.pkl"
      compile_driving_warp(cam_w, cam_h, model_w, model_h, str(path))

      op = SimpleNamespace(camera=lambda: (cam_w, cam_h, model_w, model_h), warp_path=lambda *args: path)
      warp = Warps(op).load(cam_w, cam_h, model_w, model_h)

      frame_size = get_nv12_info(cam_w, cam_h)[3]
      runtime = Warp(warp, frame_size)
      rng = np.random.default_rng(7)
      frames = [rng.integers(0, 256, frame_size, dtype=np.uint8) for _ in range(2)]
      # a plausible camera->model transform: mostly a scale, a little perspective
      tfms = [np.array([[2.5, 0.01, 300.], [0.02, 2.5, 200.], [0., 0., 1.]], dtype=np.float32),
              np.array([[3.0, -0.02, 150.], [0.01, 3.0, 90.], [0., 0.00001, 1.]], dtype=np.float32)]
      runtime.start(frames[0].ctypes.data, frames[1].ctypes.data, tfms[0], tfms[1])
      runtime.wait()
      got = np.frombuffer(runtime.output, dtype=np.uint8).reshape(2, 6, model_h // 2, model_w // 2).copy()

      fn, specs = make_driving_warp(cam_w, cam_h, model_w, model_h)
      copy_size = specs['input_frame'][0][1]
      ref = fn(input_frame=Tensor(np.stack([f[:copy_size] for f in frames])).realize(),
               M_inv=Tensor(np.stack(tfms)).realize()).numpy()
      np.testing.assert_array_equal(got, ref)
      assert got.any(), "the warp made an empty frame"


if __name__ == "__main__":
  unittest.main()
