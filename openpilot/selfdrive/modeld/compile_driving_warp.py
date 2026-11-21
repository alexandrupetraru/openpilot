#!/usr/bin/env python3
"""Compile the driving warp Jetlink runs on the comma.

Jetlink (jetlink_repo, jetlink/openpilot/warp.py) keeps the camera warp on the comma and sends the warped frame to the
large model on the host. It loads openpilot's driving warp pickle, driving_warp_{w}x{h}_tinygrad.pkl:
  {'run': TinyJit, 'input_specs': {name: (shape, dtype, device)}, 'metadata': {...}}
taking both camera frames as one (2, frame size) uint8 `input_frame` and their transforms as one (2, 3, 3) float32
`M_inv`, and making the (2, 6, model_h/2, model_w/2) uint8 frame the model reads. Upstream openpilot builds it with
tinygrad's examples/openpilot/compile_warp.py --layout yuv420 --frames 2, which this tinygrad does not ship; the warp
math is modeld's own (compile_modeld.make_frame_prepare), so the large model sees exactly the frame the small one does.

Checked here before it is written, and again after it is read back: the JIT's output equals the plain function's on
two different inputs (a JIT that captured nothing, or that ignores its inputs, fails), and its shape and input names
are what jetlink's loader requires.
"""
import argparse
import pickle

import numpy as np
from tinygrad.tensor import Tensor
from tinygrad.device import Device
from tinygrad.engine.jit import TinyJit

from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_frame_prepare, nv12_copy_size, _parse_size

FRAMES = 2  # the road camera and the wide camera, warped together
INPUT_NAMES = ['M_inv', 'input_frame']  # jetlink.openpilot.warp.WARP_INPUT_NAMES: sorted(kwargs) of the call


def make_driving_warp(cam_w: int, cam_h: int, model_w: int, model_h: int):
  stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
  # what modeld copies of each camera buffer, short of its trailing guard allocation
  frame_size = nv12_copy_size(stride, y_height, uv_height)
  frame_prepare = make_frame_prepare(NV12Frame(cam_w, cam_h, stride, y_height, uv_height, frame_size), model_w, model_h)

  def warp(input_frame, M_inv):
    return Tensor.stack(*(frame_prepare(input_frame[i], M_inv[i]) for i in range(FRAMES)))

  specs = {'input_frame': ((FRAMES, frame_size), np.dtype(np.uint8).str, Device.DEFAULT),
           'M_inv': ((FRAMES, 3, 3), np.dtype(np.float32).str, Device.DEFAULT)}
  return warp, specs


def make_inputs(specs, seed: int) -> dict[str, Tensor]:
  rng = np.random.default_rng(seed)
  (frame_shape, _, frame_dev), (tfm_shape, _, tfm_dev) = specs['input_frame'], specs['M_inv']
  frames = rng.integers(0, 256, frame_shape, dtype=np.uint8)
  tfm = (rng.standard_normal(tfm_shape) * 8).astype(np.float32)
  return {'input_frame': Tensor(frames, device=frame_dev).realize(), 'M_inv': Tensor(tfm, device=tfm_dev).realize()}


def check(jit, warp, specs, model_w: int, model_h: int) -> None:
  for seed in (42, 43):
    inputs = make_inputs(specs, seed)
    expected = warp(**inputs).numpy()
    got = jit(**inputs).numpy()
    np.testing.assert_array_equal(got, expected)
  assert expected.shape == (FRAMES, 6, model_h // 2, model_w // 2), expected.shape
  assert expected.dtype == np.uint8, expected.dtype
  captured = getattr(jit, 'captured', None)
  assert captured is not None, "the warp JIT never captured"
  assert list(captured.expected_names) == INPUT_NAMES, captured.expected_names


def compile_driving_warp(cam_w: int, cam_h: int, model_w: int, model_h: int, pkl_path: str) -> None:
  print(f"Compiling driving warp for {cam_w}x{cam_h} -> {model_w}x{model_h}, {FRAMES} frames...")
  warp, specs = make_driving_warp(cam_w, cam_h, model_w, model_h)
  jit = TinyJit(warp, prune=True)
  # TinyJit captures on its second call; the third replays the capture
  inputs = make_inputs(specs, 41)
  for _ in range(3):
    jit(**inputs).realize()
  Device.default.synchronize()
  check(jit, warp, specs, model_w, model_h)

  artifact = {'metadata': {'camera': (cam_w, cam_h), 'model': (model_w, model_h), 'frames': FRAMES}, 'run': jit, 'input_specs': specs}
  with open(pkl_path, "wb") as f:
    pickle.dump(artifact, f)

  with open(pkl_path, "rb") as f:
    loaded = pickle.load(f)
  check(loaded['run'], warp, loaded['input_specs'], model_w, model_h)
  print(f"  Saved to {pkl_path}")


if __name__ == "__main__":
  p = argparse.ArgumentParser()
  p.add_argument('--camera-resolution', type=_parse_size, required=True, help='camera resolution WxH')
  p.add_argument('--warp-to', type=_parse_size, required=True, help='model input WxH')
  p.add_argument('--output', required=True)
  args = p.parse_args()
  compile_driving_warp(*args.camera_resolution, *args.warp_to, args.output)
