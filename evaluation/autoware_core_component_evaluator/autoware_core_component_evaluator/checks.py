# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Per-message validity checks -- the invariants that a rate measurement cannot see.

These are the checks that catch the failures which are simultaneously the most damaging and the
least visible:

  * A trajectory containing NaN. Downstream arithmetic propagates it silently; the vehicle stops
    or veers and nothing logs an error. `finite` is the single highest-value check here.
  * An empty trajectory published at a perfect 10 Hz. Every timing check passes.
  * A trajectory whose points run backwards, or whose velocities are negative on a forward plan.
  * A covariance with negative diagonal entries, which makes every consumer's filter diverge.

Each check is (name, callable) -> None when the message is fine, or a short string naming what
was wrong. Checks are declared per interface in `expectation/*.yaml` under `checks:`, so a check
runs only where it means something.

They are deliberately cheap: they run on every message of every subscribed topic, and an
evaluator that perturbs the timing it is measuring is worse than no evaluator.
"""

from __future__ import annotations

import math
from typing import Any, Callable

# Bounds used by the generic bound checks. Chosen to be obviously-wrong thresholds rather than
# tight ones: this layer answers "is this message structurally insane", not "is this good
# driving". Behavioural bounds belong in expectation/*.yaml as metric gates.
MAX_PLAUSIBLE_SPEED_MPS = 100.0
MAX_PLAUSIBLE_ACCEL_MPS2 = 50.0
MAX_PLAUSIBLE_STEER_RAD = 1.6
MAX_PLAUSIBLE_POSITION_M = 1.0e6


def _finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _pose_finite(pose: Any) -> bool:
    p, q = pose.position, pose.orientation
    return _finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


# =================== Generic checks that apply to any message type ===================

def finite(message: Any) -> str | None:
    """Recursively reject NaN/Inf in any float field, to a bounded depth.

    Bounded because a PredictedObjects message with 60 objects x 20 predicted paths x 100 points
    is 120k floats, and walking all of them at 10 Hz is not free. The first offending field is
    named; that is enough to locate the producer.
    """
    return _walk_finite(message, depth=0)


def _walk_finite(value: Any, depth: int, path: str = "") -> str | None:
    if depth > 6:
        return None
    if isinstance(value, float):
        return None if math.isfinite(value) else f"{path or 'value'} is {value}"
    if isinstance(value, (int, bool, str, bytes)):
        return None
    if isinstance(value, (list, tuple)):
        for index, item in enumerate(value[:64]):
            if failure := _walk_finite(item, depth + 1, f"{path}[{index}]"):
                return failure
        return None
    fields = getattr(value, "get_fields_and_field_types", None)
    if fields is None:
        return None
    for name in fields():
        if failure := _walk_finite(getattr(value, name, None), depth + 1, f"{path}.{name}"):
            return failure
    return None


def nonempty(message: Any) -> str | None:
    """A message whose only payload is an empty sequence carries no information.

    Applied to trajectories, paths, and map binaries. `autoware_path_generator` publishing an
    empty path is the exact signature of an ego spawned off-lanelet, and it is otherwise
    indistinguishable from a broken integration.
    """
    for attribute in ("points", "data", "objects", "traffic_light_groups"):
        sequence = getattr(message, attribute, None)
        if sequence is not None:
            return None if len(sequence) > 0 else f"{attribute} is empty"
    return None


# =================== Planning checks ===================

def arc_length_monotonic(message: Any) -> str | None:
    """Consecutive trajectory points must advance. Zero-length segments are tolerated
    (a stopped plan repeats its point), reversals are not."""
    points = getattr(message, "points", [])
    previous = None
    for index, point in enumerate(points[:512]):
        pose = getattr(point, "pose", None) or getattr(getattr(point, "point", None), "pose", None)
        if pose is None:
            return None
        current = (pose.position.x, pose.position.y)
        if previous is not None:
            step = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if step > 1.0e3:
                return f"point {index} jumps {step:.1f} m from its predecessor"
        previous = current
    return None


def velocity_bounds(message: Any) -> str | None:
    """Longitudinal velocity on a plan must be finite, non-negative and plausible.

    A negative longitudinal velocity on a forward trajectory is not a slow plan; it is a sign
    error, and the controller will act on it.
    """
    for index, point in enumerate(getattr(message, "points", [])[:512]):
        target = getattr(point, "longitudinal_velocity_mps", None)
        if target is None:
            target = getattr(getattr(point, "point", None), "longitudinal_velocity_mps", None)
        if target is None:
            return None
        if not _finite(target):
            return f"point {index} velocity is {target}"
        if target < -1.0e-3:
            return f"point {index} velocity is negative ({target:.3f} m/s)"
        if target > MAX_PLAUSIBLE_SPEED_MPS:
            return f"point {index} velocity is implausible ({target:.1f} m/s)"
    return None


# =================== Control checks ===================

def control_bounds(message: Any) -> str | None:
    """Steering and acceleration commands within physically plausible bounds."""
    lateral, longitudinal = getattr(message, "lateral", None), getattr(message, "longitudinal", None)
    if lateral is None or longitudinal is None:
        return None

    steer = getattr(lateral, "steering_tire_angle", math.nan)
    accel = getattr(longitudinal, "acceleration", math.nan)
    if not _finite(steer, accel):
        return f"steering={steer} acceleration={accel}"
    if abs(steer) > MAX_PLAUSIBLE_STEER_RAD:
        return f"steering {steer:.3f} rad exceeds {MAX_PLAUSIBLE_STEER_RAD} rad"
    if abs(accel) > MAX_PLAUSIBLE_ACCEL_MPS2:
        return f"acceleration {accel:.2f} m/s^2 exceeds {MAX_PLAUSIBLE_ACCEL_MPS2} m/s^2"
    return None


# =================== Localization checks ===================

def pose_plausible(message: Any) -> str | None:
    """Pose finite, in bounds, with a normalised quaternion.

    An unnormalised quaternion is the classic symptom of a hand-built pose reaching the stack;
    consumers that assume normalisation produce a heading error that looks like a control bug.
    """
    pose = getattr(getattr(message, "pose", None), "pose", None) or getattr(message, "pose", None)
    if pose is None or not hasattr(pose, "position"):
        return None
    if not _pose_finite(pose):
        return "pose contains a non-finite value"
    if max(abs(pose.position.x), abs(pose.position.y)) > MAX_PLAUSIBLE_POSITION_M:
        return f"position ({pose.position.x:.1f}, {pose.position.y:.1f}) is out of range"

    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if abs(norm - 1.0) > 1.0e-2:
        return f"quaternion norm is {norm:.4f}, not 1"
    return None


def covariance_valid(message: Any) -> str | None:
    """Diagonal entries of any covariance must be finite and non-negative."""
    for owner in (getattr(message, "pose", None), getattr(message, "twist", None), message):
        covariance = getattr(owner, "covariance", None)
        if covariance is None:
            continue
        values = list(covariance)
        size = int(math.isqrt(len(values)))
        for i in range(size):
            diagonal = values[i * size + i]
            if not _finite(diagonal):
                return f"covariance diagonal {i} is {diagonal}"
            if diagonal < 0.0:
                return f"covariance diagonal {i} is negative ({diagonal:.3e})"
    return None


# =================== Perception checks ===================

def objects_plausible(message: Any) -> str | None:
    """Object dimensions positive, classification probabilities in [0, 1] and summing to <= 1.

    A zero-dimension object passes every finiteness check and then makes every downstream
    distance computation meaningless.
    """
    for index, obj in enumerate(getattr(message, "objects", [])[:64]):
        shape = getattr(obj, "shape", None)
        if shape is not None and hasattr(shape, "dimensions"):
            d = shape.dimensions
            if not _finite(d.x, d.y, d.z):
                return f"object {index} has a non-finite dimension"
            if min(d.x, d.y, d.z) <= 0.0:
                return f"object {index} has a non-positive dimension ({d.x:.2f}, {d.y:.2f}, {d.z:.2f})"

        total = 0.0
        for classification in getattr(obj, "classification", []):
            probability = getattr(classification, "probability", math.nan)
            if not _finite(probability) or not 0.0 <= probability <= 1.0:
                return f"object {index} classification probability is {probability}"
            total += probability
        if total > 1.0 + 1.0e-3:
            return f"object {index} classification probabilities sum to {total:.3f}"
    return None


# =================== Map checks ===================

def map_nonempty(message: Any) -> str | None:
    """A LaneletMapBin whose payload is empty means the map failed to load, and every
    downstream failure after that point is a consequence rather than a cause."""
    data = getattr(message, "data", None)
    if data is None:
        return None
    return None if len(data) > 0 else "map binary payload is empty"


# Check registry: name -> callable
CHECKS: dict[str, Callable[[Any], "str | None"]] = {
    "arc_length_monotonic": arc_length_monotonic,
    "control_bounds": control_bounds,
    "covariance_valid": covariance_valid,
    "finite": finite,
    "map_nonempty": map_nonempty,
    "nonempty": nonempty,
    "objects_plausible": objects_plausible,
    "pose_plausible": pose_plausible,
    "velocity_bounds": velocity_bounds,
}
