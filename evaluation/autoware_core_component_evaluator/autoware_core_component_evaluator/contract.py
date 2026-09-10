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
The executable contract: what each `autoware_core` component's interfaces are, and what they are
expected to do.

TWO SOURCES, DELIBERATELY
-------------------------
The *structural* half -- interface name, message type, topic-or-service, QoS -- is not ours to
invent. `autoware_core` already declares it, in
`autoware_component_interface_specs/interface_manifest.json`, generated from the C++ specs and
version-controlled alongside the code it describes. Re-typing those 34 entries here would create a
second source of truth that silently rots the first time core renames a topic.

The *performance* half -- rates, gaps, jitter, latency, validity -- is not in the manifest and has
nowhere else to live. That is what `expectation/*.yaml` in this package declares, keyed by
interface name.

So a contract entry is a join:

    manifest entry (type, kind, qos)  x  expectation entry (rates, checks, profiles)

An expectation naming an interface the manifest does not declare is allowed -- core publishes
plenty of topics it has not yet promoted to a declared interface,
`/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` among them -- but
it must then carry its own `type`, and it is reported as `declared: false` so the gap is visible
rather than invisible.

PROFILES
--------
Not every component runs in every configuration. P0 has the simulator fake localization and
perception; P2 runs core's own localization; P3 feeds raw LiDAR into core's perception. An
interface required in P2 and absent in P0 is NOT_APPLICABLE in P0, not a failure -- keeping those
distinct is the difference between a report a reviewer can trust and a wall of green.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any

import yaml

DEFAULT_PROFILE = "P0"

# Interfaces are matched to the manifest by exact name. `kind` follows the manifest vocabulary.
TOPIC = "topic"
SERVICE = "service"


@dataclass
class Rate:
    min_hz: float = 0.0
    max_hz: float = math.inf

    @staticmethod
    def parse(value: Any) -> "Rate":
        if value is None:
            return Rate()
        return Rate(
            min_hz=float(value.get("min_hz", 0.0)),
            max_hz=float(value.get("max_hz", math.inf)),
        )


@dataclass
class Gate:
    """A pass/fail bound on a metric. Absent bounds are not checked."""

    minimum: float | None = None
    maximum: float | None = None

    @staticmethod
    def parse(value: Any) -> "Gate | None":
        if value is None:
            return None
        return Gate(
            minimum=None if value.get("min") is None else float(value["min"]),
            maximum=None if value.get("max") is None else float(value["max"]),
        )

    def violation(self, value: float) -> str | None:
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return "not measured"
        if self.minimum is not None and value < self.minimum:
            return f"{value:.4g} < min {self.minimum:.4g}"
        if self.maximum is not None and value > self.maximum:
            return f"{value:.4g} > max {self.maximum:.4g}"
        return None


@dataclass
class InterfaceExpectation:
    name: str
    domain: str
    type: str = ""
    kind: str = TOPIC
    qos: dict = field(default_factory=dict)
    declared: bool = False  # present in autoware_core's own interface manifest
    direction: str = "output"
    required_in: list[str] = field(default_factory=list)
    rate: Rate = field(default_factory=Rate)
    max_gap_s: float = math.inf
    max_jitter_s: float = math.inf
    latency_max_ms: float = math.inf
    checks: list[str] = field(default_factory=list)
    note: str = ""

    def required(self, profile: str) -> bool:
        return profile in self.required_in


@dataclass
class MetricExpectation:
    name: str
    domain: str
    unit: str = ""
    # Exactly one of these is set.
    interface: str | None = None
    statistic: str | None = None
    derived: str | None = None
    gate: Gate | None = None
    required_in: list[str] = field(default_factory=list)
    publish_as_user_defined_value: bool = True
    note: str = ""

    def required(self, profile: str) -> bool:
        return profile in self.required_in


@dataclass
class ComponentExpectation:
    domain: str
    title: str = ""
    description: str = ""
    interfaces: list[InterfaceExpectation] = field(default_factory=list)
    metrics: list[MetricExpectation] = field(default_factory=list)


class Contract:
    """Every component's expectations, joined against autoware_core's interface manifest."""

    def __init__(self, components: dict[str, ComponentExpectation], manifest_path: Path | None):
        self.components = components
        self.manifest_path = manifest_path

    # ------------------------------------------------------------------ loading ----
    @staticmethod
    def load(expectation_directory: Path, manifest_path: Path | None) -> "Contract":
        manifest = _load_manifest(manifest_path)

        components: dict[str, ComponentExpectation] = {}
        for path in sorted(Path(expectation_directory).glob("*.yaml")):
            document = yaml.safe_load(path.read_text()) or {}
            domain = document.get("component") or path.stem
            component = ComponentExpectation(
                domain=domain,
                title=document.get("title", domain),
                description=(document.get("description") or "").strip(),
            )

            for entry in document.get("interfaces") or []:
                component.interfaces.append(_interface(entry, domain, manifest, path))

            for entry in document.get("metrics") or []:
                component.metrics.append(_metric(entry, domain, path))

            components[domain] = component

        return Contract(components, manifest_path)

    # ------------------------------------------------------------------ queries ----
    def interfaces(self) -> list[InterfaceExpectation]:
        return [i for c in self.components.values() for i in c.interfaces]

    def metrics(self) -> list[MetricExpectation]:
        return [m for c in self.components.values() for m in c.metrics]

    def topics(self) -> list[InterfaceExpectation]:
        return [i for i in self.interfaces() if i.kind == TOPIC]

    def undeclared(self) -> list[InterfaceExpectation]:
        """Interfaces we rely on that core has not promoted to a declared interface.

        Worth reporting on its own: each one is a candidate for an upstream
        autoware_component_interface_specs entry, and until it has one, nothing stops core
        renaming it in a patch release.
        """
        return [i for i in self.interfaces() if not i.declared]


def _load_manifest(manifest_path: Path | None) -> dict[str, dict]:
    if manifest_path is None or not Path(manifest_path).is_file():
        return {}
    document = json.loads(Path(manifest_path).read_text())
    return {entry["interface"]: entry for entry in document.get("interfaces", [])}


def _interface(
    entry: dict, domain: str, manifest: dict[str, dict], source: Path
) -> InterfaceExpectation:
    name = entry["name"]
    declared = manifest.get(name)

    if entry.get("from_manifest", True) and declared is None and not entry.get("type"):
        raise ValueError(
            f"{source.name}: interface {name} is not declared in autoware_core's interface "
            f"manifest and no explicit `type:` was given. Add the type, or set "
            f"`from_manifest: false` if it is deliberately undeclared."
        )

    return InterfaceExpectation(
        name=name,
        domain=domain,
        type=entry.get("type") or (declared or {}).get("type", ""),
        kind=entry.get("kind") or (declared or {}).get("kind", TOPIC),
        qos=(declared or {}).get("qos", {}),
        declared=declared is not None,
        direction=entry.get("direction", "output"),
        required_in=list(entry.get("required_in", [DEFAULT_PROFILE])),
        rate=Rate.parse(entry.get("rate")),
        max_gap_s=float(entry.get("max_gap_s", math.inf)),
        max_jitter_s=float(entry.get("max_jitter_s", math.inf)),
        latency_max_ms=float(entry.get("latency_max_ms", math.inf)),
        checks=list(entry.get("checks", [])),
        note=(entry.get("note") or "").strip(),
    )


def _metric(entry: dict, domain: str, source: Path) -> MetricExpectation:
    interface = entry.get("interface")
    statistic = entry.get("statistic")
    derived = entry.get("derived")

    if bool(derived) == bool(interface):
        raise ValueError(
            f"{source.name}: metric {entry.get('name')} must set either `derived:` or both "
            f"`interface:` and `statistic:`, not both and not neither."
        )
    if interface and not statistic:
        raise ValueError(f"{source.name}: metric {entry.get('name')} needs a `statistic:`.")

    return MetricExpectation(
        name=entry["name"],
        domain=domain,
        unit=entry.get("unit", ""),
        interface=interface,
        statistic=statistic,
        derived=derived,
        gate=Gate.parse(entry.get("gate")),
        required_in=list(entry.get("required_in", [DEFAULT_PROFILE])),
        publish_as_user_defined_value=bool(entry.get("publish", True)),
        note=(entry.get("note") or "").strip(),
    )
