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
Derived metrics: the quantities that need more than one topic, or need ground truth.

WHY THESE ONES
----------------------------------
Per-interface statistics (rate, gap, jitter) answer "is the component alive and punctual". They
are necessary and they are cheap, but they are not evidence of quality -- a planner emitting a
constant, wrong trajectory at a flawless 10 Hz passes every one of them.

The metrics here answer "is the component correct", and several of them are only computable in
simulation:

  * ACCURACY AGAINST GROUND TRUTH: scenario_simulator publishes the exact pose of every entity on
    /simulation/entity/status. Comparing core's own /localization/kinematic_state against it gives
    a true absolute trajectory error -- not the self-consistency figure a covariance would give
    you. No unit test, and no on-vehicle log, can produce this number.

  * DETECTION RECALL: The simulator knows exactly how many objects exist, so
    perception's output can be scored rather than merely rate-checked.

  * TRACKING ERROR: Lateral deviation of the vehicle from the trajectory it was given, and
    longitudinal error against commanded speed, jointly measure whether control closes the loop.

  * LIFECYCLE TIMING: Time from launch to a usable trajectory, from route to plan, from engage to
    motion. These are the numbers that regress first when a component gets slower, and the ones a
    stakeholder actually recognises.

  * COMFORT: Peak acceleration and jerk, taken from the commands rather than from a differentiated
    pose, so they measure what control asked for rather than what the vehicle model did.

Every metric here degrades to NaN when its inputs were absent, and NaN is reported as
"not measured" rather than as a pass. A metric that silently reads 0.0 when its topic is dead is
worse than no metric.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

EGO_ENTITY_NAMES = ("ego", "Ego", "EGO")


def _yaw(orientation: Any) -> float:
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_difference(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _distance_to_polyline(x: float, y: float, points: list[tuple[float, float]]) -> float:
    """Shortest distance from a point to a polyline, segment-wise.

    Nearest-vertex would systematically overestimate on the sparse trajectories core emits
    (0.5 m spacing is common), which would make a lateral-deviation gate meaningless.
    """
    best = math.inf
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        dx, dy = x2 - x1, y2 - y1
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            best = min(best, math.hypot(x - x1, y - y1))
            continue
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / length_squared))
        best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return best


@dataclass
class Derived:
    """Accumulated cross-topic state, and the metrics computed from it."""

    started_s: float

    # Lifecycle timestamps, each recorded once at the first observation of the transition.
    localization_initialized_s: float | None = None
    route_set_s: float | None = None
    first_trajectory_s: float | None = None
    autonomous_s: float | None = None
    first_motion_s: float | None = None
    arrived_s: float | None = None

    # Ego odometry, as reported by core's localization.
    odometry_positions: list[tuple[float, float]] = field(default_factory=list)
    traveled_distance_m: float = 0.0
    max_speed_mps: float = 0.0
    max_pose_jump_m: float = 0.0
    _last_odometry: tuple[float, float, float, float] | None = None  # x, y, yaw, t

    # Latest trajectory, as a polyline plus its speed profile.
    trajectory_points: list[tuple[float, float]] = field(default_factory=list)
    trajectory_target_speed_mps: float = math.nan

    # Tracking error, accumulated while the vehicle is actually driving.
    max_lateral_deviation_m: float = 0.0
    speed_error_samples: list[float] = field(default_factory=list)

    # Command comfort, from /control/command/control_cmd.
    max_accel_mps2: float = 0.0
    max_jerk_mps3: float = 0.0
    _last_command: tuple[float, float] | None = None  # acceleration, t

    # Ground truth from the simulator.
    ground_truth_pose: tuple[float, float, float] | None = None  # x, y, yaw
    ground_truth_object_count: int = 0
    position_error_samples: list[float] = field(default_factory=list)
    heading_error_samples: list[float] = field(default_factory=list)

    # Perception, scored against the simulator's own object count.
    detected_object_counts: list[int] = field(default_factory=list)
    ground_truth_object_counts: list[int] = field(default_factory=list)

    # ------------------------------------------------------------------ ingest ----
    def on_odometry(self, message: Any, now_s: float) -> None:
        pose = message.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, _yaw(pose.orientation)
        if not all(map(math.isfinite, (x, y, yaw))):
            return

        speed = abs(message.twist.twist.linear.x)
        self.max_speed_mps = max(self.max_speed_mps, speed)
        if speed > 0.1 and self.first_motion_s is None:
            self.first_motion_s = now_s

        if self._last_odometry is not None:
            px, py, _, pt = self._last_odometry
            step = math.hypot(x - px, y - py)
            self.traveled_distance_m += step
            # A jump larger than what the elapsed time allows is a localization discontinuity,
            # not motion. 2x the max plausible speed leaves room for timing noise.
            elapsed = max(now_s - pt, 1.0e-3)
            if step / elapsed > 2.0 * max(self.max_speed_mps, 1.0):
                self.max_pose_jump_m = max(self.max_pose_jump_m, step)
        self._last_odometry = (x, y, yaw, now_s)
        self.odometry_positions.append((x, y))

        if self.trajectory_points and len(self.trajectory_points) > 1 and speed > 0.1:
            self.max_lateral_deviation_m = max(
                self.max_lateral_deviation_m, _distance_to_polyline(x, y, self.trajectory_points)
            )
            if math.isfinite(self.trajectory_target_speed_mps):
                self.speed_error_samples.append(abs(speed - self.trajectory_target_speed_mps))

        if self.ground_truth_pose is not None:
            gx, gy, gyaw = self.ground_truth_pose
            self.position_error_samples.append(math.hypot(x - gx, y - gy))
            self.heading_error_samples.append(_angle_difference(yaw, gyaw))

    def on_trajectory(self, message: Any, now_s: float) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in message.points]
        if not points:
            return
        if self.first_trajectory_s is None:
            self.first_trajectory_s = now_s
        self.trajectory_points = points
        # The speed the plan asks for near the vehicle, which is what control is tracking now.
        self.trajectory_target_speed_mps = float(message.points[0].longitudinal_velocity_mps)

    def on_control_command(self, message: Any, now_s: float) -> None:
        acceleration = float(message.longitudinal.acceleration)
        if not math.isfinite(acceleration):
            return
        self.max_accel_mps2 = max(self.max_accel_mps2, abs(acceleration))
        if self._last_command is not None:
            previous, t = self._last_command
            elapsed = now_s - t
            if elapsed > 1.0e-4:
                self.max_jerk_mps3 = max(self.max_jerk_mps3, abs(acceleration - previous) / elapsed)
        self._last_command = (acceleration, now_s)

    def on_localization_state(self, message: Any, now_s: float) -> None:
        # LocalizationInitializationState.INITIALIZED == 3
        if message.state == 3 and self.localization_initialized_s is None:
            self.localization_initialized_s = now_s

    def on_route_state(self, message: Any, now_s: float) -> None:
        # RouteState: UNSET=1, SET=2, ARRIVED=3, CHANGING=4
        if message.state >= 2 and self.route_set_s is None:
            self.route_set_s = now_s
        if message.state == 3 and self.arrived_s is None:
            self.arrived_s = now_s

    def on_operation_mode_state(self, message: Any, now_s: float) -> None:
        # OperationModeState.AUTONOMOUS == 2
        if message.mode == 2 and self.autonomous_s is None:
            self.autonomous_s = now_s

    def on_predicted_objects(self, message: Any, _now_s: float) -> None:
        self.detected_object_counts.append(len(message.objects))
        self.ground_truth_object_counts.append(self.ground_truth_object_count)

    def on_entity_status(self, message: Any, _now_s: float) -> None:
        """Simulator ground truth: every entity's exact pose, ego included."""
        others = 0
        for entity in message.data:
            if entity.name in EGO_ENTITY_NAMES:
                pose = entity.status.pose
                self.ground_truth_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            else:
                others += 1
        self.ground_truth_object_count = others

    # ------------------------------------------------------------------ metrics ----
    def _elapsed(self, moment: float | None) -> float:
        return math.nan if moment is None else moment - self.started_s

    def _between(self, start: float | None, end: float | None) -> float:
        return math.nan if start is None or end is None else end - start

    def values(self) -> dict[str, float]:
        def mean(samples: list[float]) -> float:
            return sum(samples) / len(samples) if samples else math.nan

        def rms(samples: list[float]) -> float:
            return math.sqrt(sum(s * s for s in samples) / len(samples)) if samples else math.nan

        recall = math.nan
        pairs = [
            (detected, truth)
            for detected, truth in zip(self.detected_object_counts, self.ground_truth_object_counts)
            if truth > 0
        ]
        if pairs:
            recall = mean([min(detected / truth, 1.0) for detected, truth in pairs])

        return {
            # Lifecycle.
            "time_to_localization_initialized_s": self._elapsed(self.localization_initialized_s),
            "time_to_route_set_s": self._elapsed(self.route_set_s),
            "time_to_first_trajectory_s": self._elapsed(self.first_trajectory_s),
            "time_from_route_to_trajectory_s": self._between(
                self.route_set_s, self.first_trajectory_s
            ),
            "time_to_autonomous_s": self._elapsed(self.autonomous_s),
            "time_from_autonomous_to_motion_s": self._between(self.autonomous_s, self.first_motion_s),
            "time_to_arrival_s": self._elapsed(self.arrived_s),
            "goal_reached": 1.0 if self.arrived_s is not None else 0.0,
            # Behaviour.
            "traveled_distance_m": self.traveled_distance_m,
            "max_speed_mps": self.max_speed_mps,
            "max_lateral_deviation_m": self.max_lateral_deviation_m
            if self.trajectory_points
            else math.nan,
            "mean_speed_error_mps": mean(self.speed_error_samples),
            # Comfort.
            "max_accel_mps2": self.max_accel_mps2 if self._last_command else math.nan,
            "max_jerk_mps3": self.max_jerk_mps3 if self._last_command else math.nan,
            # Localization quality, against simulator ground truth.
            "localization_ate_m": rms(self.position_error_samples),
            "localization_max_position_error_m": max(self.position_error_samples)
            if self.position_error_samples
            else math.nan,
            "localization_heading_error_rad": rms(self.heading_error_samples),
            "localization_max_pose_jump_m": self.max_pose_jump_m
            if self._last_odometry
            else math.nan,
            # Perception quality, against the simulator's own object count.
            "perception_object_recall": recall,
            "perception_mean_object_count": mean(
                [float(c) for c in self.detected_object_counts]
            ),
        }
