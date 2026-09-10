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
Guards on the contract itself.

These run without ROS and without a live stack, which is the point: an expectation file with a
typo in a metric name would otherwise only fail during a 3-minute scenario run, and then only as
a silently absent gate rather than as an error.
"""

from pathlib import Path
import re

import pytest

from autoware_core_component_evaluator.checks import CHECKS
from autoware_core_component_evaluator.contract import Contract

EXPECTATION_DIRECTORY = Path(__file__).resolve().parent.parent / "expectation"

# The scenarios that gate on these metrics. Two directories up from the package is the
# repository root; scenarios/ is a sibling of evaluation/.
SCENARIO_DIRECTORY = Path(__file__).resolve().parents[3] / "scenarios"

# A snapshot of autoware_core's manifest, so these tests run without a built core. It is used
# nowhere but here -- see test/README.md for why the evaluator must not fall back to it.
REFERENCE_MANIFEST = Path(__file__).resolve().parent / "interface_manifest.reference.json"

# openscenario_interpreter's UserDefinedValueCondition binds a topic matching
# `^(?:/[\w-]+)*/([\w]+)$`, so a metric whose name is not a bare word is unusable as a gate.
METRIC_NAME = re.compile(r"[A-Za-z0-9_]+")

# Statistic names the evaluator knows how to compute; see observation.Observation.statistic.
STATISTICS = {
    "rate_hz",
    "max_gap_s",
    "jitter_s",
    "latency_mean_ms",
    "latency_max_ms",
    "count",
    "observed",
    "publisher_count",
    "check_failures",
}

# Derived metric names; see derived.Derived.values.
DERIVED = {
    "time_to_localization_initialized_s",
    "time_to_route_set_s",
    "time_to_first_trajectory_s",
    "time_from_route_to_trajectory_s",
    "time_to_autonomous_s",
    "time_from_autonomous_to_motion_s",
    "time_to_arrival_s",
    "goal_reached",
    "traveled_distance_m",
    "max_speed_mps",
    "max_lateral_deviation_m",
    "mean_speed_error_mps",
    "max_accel_mps2",
    "max_jerk_mps3",
    "localization_ate_m",
    "localization_max_position_error_m",
    "localization_heading_error_rad",
    "localization_max_pose_jump_m",
    "perception_object_recall",
    "perception_mean_object_count",
}


@pytest.fixture(scope="module")
def contract() -> Contract:
    return Contract.load(EXPECTATION_DIRECTORY, REFERENCE_MANIFEST)


def test_expectations_without_a_manifest_are_rejected():
    """Loading with no manifest must fail loudly, not silently drop the structural half.

    An interface declared `from_manifest` and then loaded with no manifest available has no
    message type, and a contract that quietly skipped it would report a live component as
    conformant while checking nothing about it.
    """
    with pytest.raises(ValueError, match="not declared"):
        Contract.load(EXPECTATION_DIRECTORY, None)


def test_every_component_loads(contract):
    assert set(contract.components) == {
        "api",
        "control",
        "localization",
        "map",
        "perception",
        "planning",
        "sensing",
        "system",
        "vehicle",
    }


def test_metric_names_are_unique(contract):
    names = [metric.name for metric in contract.metrics()]
    duplicates = {name for name in names if names.count(name) > 1}
    assert not duplicates, f"duplicate metric names: {sorted(duplicates)}"


def test_scenario_metric_gates_name_a_real_metric(contract):
    """Every /metrics/<name> a scenario gates on must be a metric the evaluator publishes.

    A scenario naming a metric that does not exist is the worst failure mode this project has:
    UserDefinedValueCondition simply never becomes true, so the gate is silently absent. The
    scenario then runs to its timeout and reports failure, and nothing anywhere says the gate
    was the problem -- `core_smoke.yaml` gated on `D8_traveled_distance_m`, which had never
    existed, and so could not pass however well the stack drove.
    """
    known = {metric.name for metric in contract.metrics()}
    referenced = re.compile(r"/metrics/([A-Za-z0-9_]+)")

    unknown = {}
    for scenario in sorted(SCENARIO_DIRECTORY.glob("*.yaml")):
        for name in referenced.findall(scenario.read_text()):
            if name not in known:
                unknown.setdefault(scenario.name, []).append(name)

    assert not unknown, (
        "scenarios gate on metrics that do not exist: "
        + "; ".join(f"{f}: {sorted(set(n))}" for f, n in sorted(unknown.items()))
    )


def test_metric_names_are_usable_as_scenario_gates(contract):
    for metric in contract.metrics():
        assert METRIC_NAME.fullmatch(metric.name), (
            f"{metric.name} cannot appear in /metrics/<name>: "
            "UserDefinedValueCondition would not bind it"
        )


def test_metric_sources_exist(contract):
    interfaces = {interface.name for interface in contract.interfaces()}
    for metric in contract.metrics():
        if metric.derived is not None:
            assert metric.derived in DERIVED, f"{metric.name}: unknown derived {metric.derived}"
        else:
            assert metric.statistic in STATISTICS, (
                f"{metric.name}: unknown statistic {metric.statistic}"
            )
            assert metric.interface in interfaces, (
                f"{metric.name}: reads {metric.interface}, which no component declares"
            )


def test_declared_checks_exist(contract):
    for interface in contract.interfaces():
        for check in interface.checks:
            assert check in CHECKS, f"{interface.name}: unknown check {check}"


def test_topic_interfaces_have_a_type(contract):
    for interface in contract.topics():
        assert interface.type, f"{interface.name} has no message type and cannot be subscribed"


def test_rate_bounds_are_ordered(contract):
    for interface in contract.interfaces():
        assert interface.rate.min_hz <= interface.rate.max_hz, (
            f"{interface.name}: rate min {interface.rate.min_hz} > max {interface.rate.max_hz}"
        )


def test_gates_are_satisfiable(contract):
    for metric in contract.metrics():
        if metric.gate is None:
            continue
        if metric.gate.minimum is not None and metric.gate.maximum is not None:
            assert metric.gate.minimum <= metric.gate.maximum, (
                f"{metric.name}: gate min {metric.gate.minimum} > max {metric.gate.maximum}"
            )


def test_p0_is_the_baseline_profile(contract):
    # Every profile builds on P0, so anything required in P0 must remain required later --
    # a metric required only in P0 would silently stop being judged in P2.
    for metric in contract.metrics():
        if "P0" in metric.required_in:
            assert {"P2", "P3"} <= set(metric.required_in), (
                f"{metric.name} is required in P0 but not in every later profile"
            )


def test_reference_manifest_covers_declared_interfaces(contract):
    """Everything not carrying its own type must have been joined against the manifest."""
    for interface in contract.interfaces():
        assert interface.declared or interface.type, (
            f"{interface.name} is neither declared in the manifest nor typed in its expectation"
        )


def test_undeclared_interfaces_are_the_expected_ones(contract):
    """Pin the set of interfaces autoware_core has not promoted to a declared interface.

    Each is a candidate for an upstream autoware_component_interface_specs entry. This test is a
    ratchet: when core declares one, it fails and the list shrinks, which is the direction of
    travel we want to notice.
    """
    assert {interface.name for interface in contract.undeclared()} == {
        "/api/localization/initialization_state",
        "/api/operation_mode/state",
        "/api/routing/state",
        "/perception/object_recognition/detection/objects",
        "/perception/obstacle_segmentation/pointcloud",
        "/planning/scenario_planning/lane_driving/behavior_planning/path",
        "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
    }
