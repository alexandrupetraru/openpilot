"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

SCC-Map's near-field camera gate, for stock ACC + ICBM cars.

The map (mapd's MapTargetVelocities) asks for a slower speed once its slowest point ahead comes inside braking
distance. On a junction or a wrong road match it asks for a turn's speed where the car goes straight: logged on a VW
Tiguan in Cluj, 6 of 11 map slowdowns were on roads driven straight (measured under 0.6 m/s^2), down to the 20 km/h
floor, three of them at the same spot across two drives. At every one the camera predicted under 0.53 m/s^2; at every
real map curve, over 0.96.

The camera's own read is only measured, not predicted, close to the car: the small model's curvature read holds to
about 3 s ahead, the big model's (Jetlink, a chestnut) to about 4 s (zoompilot docs/zoompilot/scc-curve-planning.md,
74 h of drives). So the gate vetoes a map target only once its point is inside that near field and the camera sees a
straight road there; a point further ahead is the map's, which is what it is for (a fast curve the camera cannot see
yet). A phantom therefore still starts a slowdown while its point is far, and the gate ends it once the point is near.

Never on a camera it cannot trust: modelV2 missing or invalid, or within TRUST_FRAMES of modeld starting or of the
model changing (a Jetlink swap or hand-back starts a model from a reset history). And only on a straight read that
lasts (VETO_PERSIST_FRAMES): a frame or two of misread never cancels a map curve; the veto lifts at once when the
camera sees a bend.

The limit it cannot remove: a trusted camera that reads a real curve as straight for that long inside its near field
vetoes the map until it sees the bend. That is the same read lateral control steers by at that range, and vision's own
curve slowdown runs on it too.
"""
import math

from openpilot.common.realtime import DT_MDL

NEAR_T = 3.0        # s: the small model's path is read as measured up to here
NEAR_T_BIG = 4.0    # s: the big model's 3-4 s reads as well as the small model's 2-3 s
VETO_LAT_ACC = 0.8  # m/s^2: a near-field read under this is a straight road
RELEASE_LAT_ACC = 1.0  # m/s^2: a veto lifts once the camera sees this much (hysteresis)
TRUST_FRAMES = int(5.0 / DT_MDL)  # model frames on the same model before the camera may veto anything
VETO_PERSIST_FRAMES = int(0.5 / DT_MDL)  # consecutive straight reads before a veto
MIN_SPEED = 1.0  # m/s, for the time to the point


def near_field_lat_acc(rate_z, vel_x, t, horizon: float) -> float:
  """The highest lateral acceleration the camera's plan predicts within `horizon` seconds (|yaw rate x speed|), or
  inf when the plan cannot be read, which never vetoes."""
  try:
    n = min(len(rate_z), len(vel_x), len(t))
    if n == 0:
      return math.inf
    lat = [abs(float(rate_z[i]) * float(vel_x[i])) for i in range(n) if float(t[i]) <= horizon]
  except (TypeError, ValueError):
    return math.inf
  if not lat or not all(math.isfinite(x) for x in lat):
    return math.inf
  return max(lat)


class CameraGate:
  def __init__(self):
    self.vetoed = False
    self.trusted_frames = 0
    self.straight_frames = 0
    self.big = None
    self.point = None
    self.near_lat_acc = math.inf

  def update(self, model_ok: bool, big: bool, rate_z, vel_x, t, v_ego: float, point, point_distance: float) -> bool:
    """One model frame. `point` is the map's limiting point (any hashable, None without one) and `point_distance`
    its distance in meters. True: the map's target for this point is vetoed this frame."""
    # how long the camera has run on the same model
    if not model_ok or big != self.big:
      self.trusted_frames = 0
    else:
      self.trusted_frames += 1
    self.big = big if model_ok else None

    if point != self.point:
      self.point = point
      self.vetoed = False
      self.straight_frames = 0

    horizon = NEAR_T_BIG if big else NEAR_T
    t_point = point_distance / max(v_ego, MIN_SPEED) if point is not None else math.inf
    if point is None or not model_ok or self.trusted_frames < TRUST_FRAMES or not (0. <= t_point <= horizon):
      self.vetoed = False
      self.straight_frames = 0
      self.near_lat_acc = math.inf
      return False

    self.near_lat_acc = near_field_lat_acc(rate_z, vel_x, t, horizon)
    if self.vetoed:
      # lifts at once when the camera sees a bend
      self.vetoed = self.near_lat_acc < RELEASE_LAT_ACC
      self.straight_frames = VETO_PERSIST_FRAMES if self.vetoed else 0
    else:
      self.straight_frames = self.straight_frames + 1 if self.near_lat_acc < VETO_LAT_ACC else 0
      self.vetoed = self.straight_frames >= VETO_PERSIST_FRAMES
    return self.vetoed
