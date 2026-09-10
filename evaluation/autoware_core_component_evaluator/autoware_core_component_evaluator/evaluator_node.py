#!/usr/bin/env python3
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
Per-component evaluation of a running `autoware_core`.

WHAT THIS NODE DOES
-------------------
1. Loads the contract: autoware_core's own interface manifest joined with this package's
   performance expectations (see contract.py).
2. Subscribes to every topic interface the active profile requires, with the declared QoS.
3. Measures rate, worst gap, jitter and latency per interface, and runs the declared per-message
   validity checks.
4. Computes the derived, cross-topic metrics -- accuracy against simulator ground truth, tracking
   error, lifecycle timings, comfort (see derived.py).
5. Publishes the result three ways, because three different consumers need it:

     /simulation/evaluation/report        EvaluationReport, the full structured result
     /simulation/evaluation/metrics       autoware_internal_metric_msgs/MetricArray, AWF-native,
                                          so anything in the Autoware ecosystem that already
                                          consumes metrics gets these for free
     /metrics/<name>                      autoware_scenario_simulation_msgs/UserDefinedValue, one topic
                                          per metric

   The third is what makes this usable as a *gate* rather than a report.
   `openscenario_interpreter`'s UserDefinedValueCondition binds any topic matching
   `^(?:/[\\w-]+)*/([\\w]+)$` carrying that type, so every metric below can be written directly
   into a scenario's stop trigger with no interpreter change -- and a metric failure then lands
   in the same JUnit XML as a functional one.

6. Writes the whole report to JSON at shutdown, for tools/report.py and CI.

ON QoS
------
Subscriptions use the QoS the manifest declares, which matters: several core interfaces are
TRANSIENT_LOCAL (the map, the localization initialization state), and a VOLATILE subscriber
joining late silently receives nothing from them, which would be indistinguishable here from a
dead publisher.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Any

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from autoware_component_evaluation_msgs.msg import (
    ComponentReport,
    EvaluationReport,
    InterfaceObservation,
    Verdict,
)
from autoware_internal_metric_msgs.msg import Metric, MetricArray
from autoware_scenario_simulation_msgs.msg import UserDefinedValue, UserDefinedValueType

from .checks import CHECKS
from .contract import TOPIC, Contract
from .derived import Derived
from .observation import Observation, stamp_seconds

# Topics the derived metrics need, mapped to the Derived method that consumes them. These are
# subscribed regardless of the contract: they are inputs to the evaluation, not subjects of it.
# /simulation/entity/status is the simulator's ground truth, and is the reason accuracy metrics
# are possible at all.
DERIVED_SOURCES = {
    "/localization/kinematic_state": ("nav_msgs/msg/Odometry", "on_odometry"),
    "/planning/trajectory": ("autoware_planning_msgs/msg/Trajectory", "on_trajectory"),
    "/control/command/control_cmd": ("autoware_control_msgs/msg/Control", "on_control_command"),
    "/api/localization/initialization_state": (
        "autoware_adapi_v1_msgs/msg/LocalizationInitializationState",
        "on_localization_state",
    ),
    "/api/routing/state": ("autoware_adapi_v1_msgs/msg/RouteState", "on_route_state"),
    "/api/operation_mode/state": (
        "autoware_adapi_v1_msgs/msg/OperationModeState",
        "on_operation_mode_state",
    ),
    "/perception/object_recognition/objects": (
        "autoware_perception_msgs/msg/PredictedObjects",
        "on_predicted_objects",
    ),
    "/simulation/entity/status": (
        "traffic_simulator_msgs/msg/EntityStatusWithTrajectoryArray",
        "on_entity_status",
    ),
}

TRANSIENT_LOCAL_SOURCES = {
    "/api/localization/initialization_state",
    "/api/operation_mode/state",
    "/api/routing/state",
}


def qos_from(declared: dict, fallback_depth: int = 5) -> QoSProfile:
    """Build a QoSProfile from the manifest's QoS description."""
    profile = QoSProfile(depth=int(declared.get("depth", fallback_depth) or fallback_depth))
    profile.history = (
        HistoryPolicy.KEEP_ALL if declared.get("history") == "keep_all" else HistoryPolicy.KEEP_LAST
    )
    profile.reliability = (
        ReliabilityPolicy.BEST_EFFORT
        if declared.get("reliability") == "best_effort"
        else ReliabilityPolicy.RELIABLE
    )
    profile.durability = (
        DurabilityPolicy.TRANSIENT_LOCAL
        if declared.get("durability") == "transient_local"
        else DurabilityPolicy.VOLATILE
    )
    return profile


def manifest_path(explicit: str = "") -> Path | None:
    """autoware_core's declared component interfaces, if core is on the path.

    `interface_manifest.json` is a **source-tree** artifact in autoware_core: its CMakeLists
    generates and diffs the file under BUILD_TESTING but never installs it, and core's own
    manifest test calls the checked-in copy "the copy consumers read". So the installed share
    directory does not carry it, and looking only there finds nothing on a real workspace.

    Resolution order:

    1. an explicit path -- the `manifest_path` parameter, for a core checked out elsewhere;
    2. the installed copy, which is where it belongs and where this starts finding it the day
       upstream adds the file to INSTALL_TO_SHARE (one stat until then);
    3. the source tree that produced that install directory, reached from the install prefix
       of core's own package rather than guessed: <ws>/install/<pkg>/share/<pkg> -> <ws>/src.

    Deliberately *not* a fallback: `test/interface_manifest.reference.json` in this repository.
    A stale snapshot silently accepted as the contract is exactly what the join exists to
    prevent -- see test/README.md.
    """
    if explicit:
        candidate = Path(explicit)
        return candidate if candidate.is_file() else None

    try:
        share = Path(get_package_share_directory("autoware_component_interface_specs"))
    except PackageNotFoundError:
        return None

    candidate = share / "interface_manifest.json"
    if candidate.is_file():
        return candidate

    # <ws>/install/<pkg>/share/<pkg> is four levels below the workspace root.
    if len(share.parents) > 3:
        source = share.parents[3] / "src"
        # Bounded, single-level wildcards on purpose: `**` in pathlib refuses to descend into
        # symlinked directories, and bootstrap.sh links every repository into src by symlink.
        for pattern in (
            "*/common/autoware_component_interface_specs/interface_manifest.json",
            "*/*/autoware_component_interface_specs/interface_manifest.json",
            "*/autoware_component_interface_specs/interface_manifest.json",
        ):
            for found in sorted(source.glob(pattern)):
                return found
    return None


class ComponentEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("component_evaluator")

        self.profile = str(self.declare_parameter("profile", "P0").value)
        self.scenario = str(self.declare_parameter("scenario", "").value)
        self.report_path = Path(
            str(self.declare_parameter("report_path", "/tmp/core_component_report.json").value)
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 1.0).value)
        # Nothing is judged during warm-up. Autoware's first seconds are legitimately ragged --
        # nodes are still discovering each other -- and gating on that measures the launch
        # system, not the component.
        self.warmup_s = float(self.declare_parameter("warmup_s", 5.0).value)

        expectation_directory = self.declare_parameter("expectation_directory", "").value
        if not expectation_directory:
            expectation_directory = str(
                Path(get_package_share_directory("autoware_core_component_evaluator")) / "expectation"
            )

        # An explicit override for a core checked out outside this workspace. Empty means
        # "find it", which is what every normal run does.
        explicit_manifest = str(self.declare_parameter("manifest_path", "").value)
        self.contract = Contract.load(Path(expectation_directory), manifest_path(explicit_manifest))
        self.observations: dict[str, Observation] = {}
        self.derived = Derived(started_s=self._now())
        self._subscriptions: list[Any] = []
        self._user_defined_publishers: dict[str, Any] = {}
        self._window_start_s = self._now()

        self.report_publisher = self.create_publisher(
            EvaluationReport, "/simulation/evaluation/report", 1
        )
        self.metric_publisher = self.create_publisher(
            MetricArray, "/simulation/evaluation/metrics", 1
        )

        self._subscribe_contract()
        self._subscribe_derived_sources()
        self.create_timer(self.publish_period_s, self._publish)

        declared = sum(1 for i in self.contract.interfaces() if i.declared)
        self.get_logger().info(
            f"component_evaluator | profile={self.profile} | "
            f"{len(self.contract.components)} components, "
            f"{len(self.contract.interfaces())} interfaces "
            f"({declared} declared in autoware_core's manifest, "
            f"{len(self.contract.undeclared())} undeclared), "
            f"{len(self.contract.metrics())} metrics"
        )
        if self.contract.manifest_path is None:
            self.get_logger().warn(
                "autoware_component_interface_specs not found: every interface will be reported "
                "as undeclared and QoS conformance cannot be checked."
            )

    # ------------------------------------------------------------------ setup ----
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _subscribe_contract(self) -> None:
        for expectation in self.contract.topics():
            observation = Observation(
                name=expectation.name,
                domain=expectation.domain,
                type=expectation.type,
                kind=expectation.kind,
            )
            self.observations[expectation.name] = observation

            message_type = self._resolve(expectation.type, expectation.name)
            if message_type is None:
                continue

            checks = [(name, CHECKS[name]) for name in expectation.checks if name in CHECKS]
            for missing in set(expectation.checks) - set(CHECKS):
                self.get_logger().warn(
                    f"{expectation.name}: unknown check {missing!r}; see checks.py for the list"
                )

            self._subscriptions.append(
                self.create_subscription(
                    message_type,
                    expectation.name,
                    self._make_callback(observation, checks),
                    qos_from(expectation.qos),
                )
            )

        # Services cannot be observed by subscription; their availability is polled instead.
        self.create_timer(2.0, self._poll_graph)

    def _subscribe_derived_sources(self) -> None:
        for topic, (type_name, method) in DERIVED_SOURCES.items():
            message_type = self._resolve(type_name, topic)
            if message_type is None:
                continue
            handler = getattr(self.derived, method)
            qos = QoSProfile(depth=5)
            if topic in TRANSIENT_LOCAL_SOURCES:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self._subscriptions.append(
                self.create_subscription(
                    message_type, topic, self._make_derived_callback(handler, topic), qos
                )
            )

    def _resolve(self, type_name: str, interface: str) -> Any:
        if not type_name:
            self.get_logger().warn(f"{interface}: no message type known; not subscribed")
            return None
        try:
            return get_message(type_name)
        except (ImportError, ValueError, AttributeError) as error:
            # A missing type is a finding, not a crash: it means the stack under test does not
            # ship the package the contract expects, which is exactly what we want reported.
            self.get_logger().warn(f"{interface}: cannot import {type_name} ({error}); not subscribed")
            return None

    def _make_callback(self, observation: Observation, checks: list) -> Any:
        def callback(message: Any) -> None:
            now = self._now()
            observation.record(now, stamp_seconds(message))
            if now - self._window_start_s < self.warmup_s:
                return
            observation.checks_run += 1
            for name, check in checks:
                try:
                    if detail := check(message):
                        observation.record_check_failure(name, detail)
                except Exception as error:  # noqa: BLE001 - a broken check must not kill the run
                    observation.record_check_failure(name, f"check raised {type(error).__name__}: {error}")

        return callback

    def _make_derived_callback(self, handler: Any, topic: str) -> Any:
        def callback(message: Any) -> None:
            try:
                handler(message, self._now())
            except Exception as error:  # noqa: BLE001
                self.get_logger().warn(f"derived metric update failed for {topic}: {error}")

        return callback

    def _poll_graph(self) -> None:
        for name, observation in self.observations.items():
            try:
                endpoints = self.get_publishers_info_by_topic(name)
                observation.publisher_count = len(endpoints)
                observation.observed_types = {e.topic_type for e in endpoints}
                observation.subscriber_count = len(self.get_subscriptions_info_by_topic(name))
            except Exception:  # noqa: BLE001 - graph introspection is best-effort
                continue

    # ------------------------------------------------------------------ judging ----
    def _judge(self, expectation: Any, observation: Observation) -> tuple[int, list[str], dict]:
        now = self._now()
        window = max(now - self._window_start_s - self.warmup_s, 1.0e-6)

        statistics = {
            "rate_hz": observation.rate_hz(window),
            "max_gap_s": observation.max_gap_s(window, now),
            "jitter_s": observation.jitter_s(),
            "latency_mean_ms": observation.latency_mean_ms(),
            "latency_max_ms": observation.latency_max_ms(),
        }

        if not expectation.required(self.profile):
            # Not required in this profile. Observed or not, it is not evidence either way.
            return Verdict.NOT_APPLICABLE, [], statistics

        if observation.count == 0:
            return (
                Verdict.NOT_OBSERVED,
                [
                    f"no message received in {window:.1f} s "
                    f"({observation.publisher_count} publisher(s) on the graph)"
                ],
                statistics,
            )

        violations: list[str] = []

        if statistics["rate_hz"] < expectation.rate.min_hz:
            violations.append(
                f"rate {statistics['rate_hz']:.2f} Hz < required {expectation.rate.min_hz:.2f} Hz"
            )
        if statistics["rate_hz"] > expectation.rate.max_hz:
            violations.append(
                f"rate {statistics['rate_hz']:.2f} Hz > allowed {expectation.rate.max_hz:.2f} Hz"
            )
        if statistics["max_gap_s"] > expectation.max_gap_s:
            violations.append(
                f"worst gap {statistics['max_gap_s']:.3f} s > allowed {expectation.max_gap_s:.3f} s"
            )
        if math.isfinite(statistics["jitter_s"]) and statistics["jitter_s"] > expectation.max_jitter_s:
            violations.append(
                f"jitter {statistics['jitter_s']:.4f} s > allowed {expectation.max_jitter_s:.4f} s"
            )
        if (
            math.isfinite(statistics["latency_max_ms"])
            and statistics["latency_max_ms"] > expectation.latency_max_ms
        ):
            violations.append(
                f"worst latency {statistics['latency_max_ms']:.1f} ms > allowed "
                f"{expectation.latency_max_ms:.1f} ms"
            )

        for check, failures in sorted(observation.check_failures.items()):
            violations.append(
                f"check {check} failed on {failures}/{observation.checks_run} message(s): "
                f"{observation.check_first_detail[check]}"
            )

        if observation.observed_types and expectation.type not in observation.observed_types:
            violations.append(
                f"type on the wire is {sorted(observation.observed_types)}, "
                f"contract declares {expectation.type}"
            )

        return (Verdict.FAIL if violations else Verdict.PASS), violations, statistics

    def _metric_values(self) -> dict[str, float]:
        now = self._now()
        window = max(now - self._window_start_s - self.warmup_s, 1.0e-6)
        derived = self.derived.values()

        values: dict[str, float] = {}
        for metric in self.contract.metrics():
            if metric.derived is not None:
                values[metric.name] = derived.get(metric.derived, math.nan)
            else:
                observation = self.observations.get(metric.interface)
                values[metric.name] = (
                    observation.statistic(metric.statistic, window, now)
                    if observation is not None
                    else math.nan
                )
        return values

    # ------------------------------------------------------------------ output ----
    def _publish(self) -> None:
        report, values = self.build_report()
        self.report_publisher.publish(report)

        metrics = MetricArray()
        metrics.stamp = report.stamp
        for name, value in sorted(values.items()):
            metrics.metric_array.append(self._metric_message(name, value))
        self.metric_publisher.publish(metrics)

        for name, value in values.items():
            self._publish_user_defined_value(name, value)

    def _metric_message(self, name: str, value: float) -> Metric:
        metric = Metric()
        metric.name = name
        metric.unit = next(
            (m.unit for m in self.contract.metrics() if m.name == name and m.unit), ""
        )
        metric.value = repr(value)
        return metric

    def _publish_user_defined_value(self, name: str, value: float) -> None:
        expectation = next((m for m in self.contract.metrics() if m.name == name), None)
        if expectation is not None and not expectation.publish_as_user_defined_value:
            return

        publisher = self._user_defined_publishers.get(name)
        if publisher is None:
            # The topic name must satisfy openscenario_interpreter's
            # `^(?:/[\w-]+)*/([\w]+)$` for UserDefinedValueCondition to bind it.
            publisher = self.create_publisher(UserDefinedValue, f"/metrics/{name}", 1)
            self._user_defined_publishers[name] = publisher

        message = UserDefinedValue()
        message.type.data = UserDefinedValueType.DOUBLE
        message.value = repr(float(value))
        publisher.publish(message)

    def build_report(self) -> tuple[EvaluationReport, dict[str, float]]:
        values = self._metric_values()

        report = EvaluationReport()
        report.stamp = self.get_clock().now().to_msg()
        report.scenario = self.scenario
        report.profile = self.profile
        report.duration_s = self._now() - self._window_start_s
        report.verdict.value = Verdict.PASS

        for domain, component in sorted(self.contract.components.items()):
            component_report = ComponentReport()
            component_report.domain = domain
            component_report.verdict.value = Verdict.PASS

            for expectation in component.interfaces:
                observation = self.observations.get(expectation.name)
                if observation is None:
                    # Services are contracted but not subscribed; report them as such.
                    observation = Observation(
                        name=expectation.name,
                        domain=domain,
                        type=expectation.type,
                        kind=expectation.kind,
                    )

                verdict, violations, statistics = self._judge(expectation, observation)

                item = InterfaceObservation()
                item.domain = domain
                item.interface = expectation.name
                item.type = expectation.type
                item.kind = expectation.kind
                item.observed = observation.count > 0
                item.publisher_count = observation.publisher_count
                item.subscriber_count = observation.subscriber_count
                item.type_matches = (
                    not observation.observed_types or expectation.type in observation.observed_types
                )
                item.qos_matches = bool(expectation.qos)
                item.message_count = observation.count
                item.rate_hz = statistics["rate_hz"]
                item.max_gap_s = statistics["max_gap_s"]
                item.jitter_s = statistics["jitter_s"]
                item.latency_mean_ms = statistics["latency_mean_ms"]
                item.latency_max_ms = statistics["latency_max_ms"]
                item.expected_rate_min_hz = expectation.rate.min_hz
                item.expected_rate_max_hz = (
                    expectation.rate.max_hz if math.isfinite(expectation.rate.max_hz) else 0.0
                )
                item.expected_max_gap_s = (
                    expectation.max_gap_s if math.isfinite(expectation.max_gap_s) else 0.0
                )
                item.expected_latency_max_ms = (
                    expectation.latency_max_ms if math.isfinite(expectation.latency_max_ms) else 0.0
                )
                item.verdict.value = verdict
                item.violations = violations
                component_report.interfaces.append(item)

                if verdict in (Verdict.FAIL, Verdict.NOT_OBSERVED):
                    component_report.verdict.value = Verdict.FAIL
                    component_report.violations.extend(
                        f"{expectation.name}: {violation}" for violation in violations
                    )

            for metric in component.metrics:
                value = values.get(metric.name, math.nan)
                component_report.metrics.append(self._metric_message(metric.name, value))
                if metric.gate is not None and metric.required(self.profile):
                    if violation := metric.gate.violation(value):
                        component_report.verdict.value = Verdict.FAIL
                        component_report.violations.append(f"{metric.name}: {violation}")

            if component_report.verdict.value == Verdict.FAIL:
                report.verdict.value = Verdict.FAIL
                report.violations.extend(
                    f"{domain}: {violation}" for violation in component_report.violations
                )
            report.components.append(component_report)

        return report, values

    def write_report(self) -> None:
        report, values = self.build_report()
        verdict_name = {
            Verdict.UNKNOWN: "UNKNOWN",
            Verdict.PASS: "PASS",
            Verdict.FAIL: "FAIL",
            Verdict.NOT_OBSERVED: "NOT_OBSERVED",
            Verdict.NOT_APPLICABLE: "NOT_APPLICABLE",
        }

        document = {
            "scenario": report.scenario,
            "profile": report.profile,
            "duration_s": report.duration_s,
            "verdict": verdict_name[report.verdict.value],
            "written_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "interface_manifest": str(self.contract.manifest_path or ""),
            "undeclared_interfaces": [i.name for i in self.contract.undeclared()],
            "violations": list(report.violations),
            "components": [
                {
                    "domain": component.domain,
                    "verdict": verdict_name[component.verdict.value],
                    "violations": list(component.violations),
                    "interfaces": [
                        {
                            "interface": item.interface,
                            "type": item.type,
                            "kind": item.kind,
                            "verdict": verdict_name[item.verdict.value],
                            "observed": item.observed,
                            "publisher_count": item.publisher_count,
                            "message_count": item.message_count,
                            "rate_hz": item.rate_hz,
                            "max_gap_s": item.max_gap_s,
                            "jitter_s": item.jitter_s,
                            "latency_mean_ms": item.latency_mean_ms,
                            "latency_max_ms": item.latency_max_ms,
                            "expected_rate_min_hz": item.expected_rate_min_hz,
                            "expected_max_gap_s": item.expected_max_gap_s,
                            "violations": list(item.violations),
                        }
                        for item in component.interfaces
                    ],
                    "metrics": {m.name: values.get(m.name, math.nan) for m in component.metrics},
                }
                for component in report.components
            ],
            "metrics": values,
        }

        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        self.report_path.write_text(json.dumps(document, indent=2, default=str))
        self.get_logger().info(
            f"wrote {report.verdict.value == Verdict.PASS and 'PASS' or 'FAIL'} report to "
            f"{self.report_path}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ComponentEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.write_report()
        except Exception as error:  # noqa: BLE001 - never lose the shutdown path
            node.get_logger().error(f"failed to write report: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
