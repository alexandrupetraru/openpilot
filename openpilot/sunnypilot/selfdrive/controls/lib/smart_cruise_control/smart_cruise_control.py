"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.camera_gate import CameraGate


class SmartCruiseControl:
  def __init__(self, camera_gate: bool = False):
    """camera_gate: stock ACC driven by ICBM, where the map's targets become the car's set speed (camera_gate.py)."""
    self.vision = SmartCruiseControlVision()
    self.map = SmartCruiseControlMap()
    if camera_gate:
      self.map.camera_gate = CameraGate()

  @staticmethod
  def camera(sm: messaging.SubMaster):
    model = sm['modelV2']
    model_ok = bool(sm.alive['modelV2'] and sm.valid['modelV2'])
    return model_ok, bool(model.big), model.orientationRate.z, model.velocity.x, model.orientationRate.t

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise: float) -> None:
    camera = self.camera(sm) if self.map.camera_gate is not None else None
    self.map.update(long_enabled, long_override, v_ego, a_ego, v_cruise, camera)
    self.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)
