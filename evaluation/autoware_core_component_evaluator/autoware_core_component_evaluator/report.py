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
Turn the evaluator's JSON report into something a human reads and something CI reads.

The JUnit output puts one <testcase> per interface and one per gated metric, grouped into a
<testsuite> per component. That grouping is the point: when CI goes red, the failing test is
named `planning./planning/trajectory` or `control.control_max_lateral_deviation_m`, so the
component that regressed is in the failure name and nobody has to open a bag to find out.

    ros2 run autoware_core_component_evaluator evaluation_report
    ros2 run autoware_core_component_evaluator evaluation_report --junit /tmp/evaluation.xml
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree

GREEN, RED, YELLOW, GREY, RESET = (
    "\033[32m",
    "\033[31m",
    "\033[33m",
    "\033[90m",
    "\033[0m",
)

COLOR = {
    "PASS": GREEN,
    "FAIL": RED,
    "NOT_OBSERVED": RED,
    "NOT_APPLICABLE": GREY,
    "UNKNOWN": YELLOW,
}


def _number(value) -> str:
    if value is None:
        return "-"
    try:
        value = float(value)
    except (TypeError, ValueError):
        return str(value)
    return "n/a" if math.isnan(value) else f"{value:.4g}"


def print_table(report: dict) -> None:
    verdict = report.get("verdict", "UNKNOWN")
    print()
    print(f"scenario  : {report.get('scenario') or '(unnamed)'}")
    print(f"profile   : {report.get('profile')}")
    print(f"duration  : {_number(report.get('duration_s'))} s")
    print(f"manifest  : {report.get('interface_manifest') or '(not found)'}")
    print(f"verdict   : {COLOR.get(verdict, '')}{verdict}{RESET}")
    print()

    for component in report.get("components", []):
        component_verdict = component.get("verdict", "UNKNOWN")
        print(
            f"{COLOR.get(component_verdict, '')}[{component_verdict:>14}]{RESET} "
            f"{component['domain']}"
        )

        for item in component.get("interfaces", []):
            item_verdict = item.get("verdict", "UNKNOWN")
            if item_verdict == "NOT_APPLICABLE":
                continue
            print(
                f"    {COLOR.get(item_verdict, '')}{item_verdict:>14}{RESET}  "
                f"{item['interface']:<66} "
                f"rate={_number(item.get('rate_hz')):>7} Hz  "
                f"gap={_number(item.get('max_gap_s')):>7} s  "
                f"n={item.get('message_count', 0)}"
            )
            for violation in item.get("violations", []):
                print(f"                     {RED}!{RESET} {violation}")

        for name, value in sorted(component.get("metrics", {}).items()):
            print(f"    {'metric':>14}  {name:<66} {_number(value)}")
        print()

    undeclared = report.get("undeclared_interfaces", [])
    if undeclared:
        print(f"{YELLOW}interfaces not declared in autoware_core's manifest{RESET} "
              f"({len(undeclared)}) -- each is a candidate for an upstream interface-spec entry:")
        for name in undeclared:
            print(f"    {name}")
        print()

    if report.get("violations"):
        print(f"{RED}violations{RESET}")
        for violation in report["violations"]:
            print(f"    {violation}")
        print()


def to_junit(report: dict) -> ElementTree.ElementTree:
    suites = ElementTree.Element("testsuites", name="autoware_core_component_evaluation")

    for component in report.get("components", []):
        cases = []

        for item in component.get("interfaces", []):
            verdict = item.get("verdict")
            if verdict == "NOT_APPLICABLE":
                continue
            case = ElementTree.Element(
                "testcase",
                classname=f"{component['domain']}.interface",
                name=item["interface"],
            )
            if verdict not in ("PASS",):
                failure = ElementTree.SubElement(
                    case, "failure", message=f"{verdict}: {item['interface']}"
                )
                failure.text = "\n".join(item.get("violations", [])) or verdict
            cases.append(case)

        for name, value in sorted(component.get("metrics", {}).items()):
            case = ElementTree.Element(
                "testcase", classname=f"{component['domain']}.metric", name=name
            )
            violations = [v for v in component.get("violations", []) if v.startswith(f"{name}:")]
            if violations:
                failure = ElementTree.SubElement(case, "failure", message=violations[0])
                failure.text = "\n".join(violations)
            case.set("time", "0")
            ElementTree.SubElement(case, "system-out").text = f"{name} = {_number(value)}"
            cases.append(case)

        suite = ElementTree.SubElement(
            suites,
            "testsuite",
            name=component["domain"],
            tests=str(len(cases)),
            failures=str(sum(1 for c in cases if c.find("failure") is not None)),
        )
        suite.extend(cases)

    return ElementTree.ElementTree(suites)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report", default="/tmp/core_component_report.json", type=Path,
        help="evaluator JSON output",
    )
    parser.add_argument("--junit", type=Path, help="also write JUnit XML here")
    parser.add_argument("--quiet", action="store_true", help="suppress the table")
    arguments = parser.parse_args(argv)

    if not arguments.report.is_file():
        print(f"{RED}no report at {arguments.report}{RESET}", file=sys.stderr)
        print("Run a scenario first, or pass --report.", file=sys.stderr)
        return 2

    report = json.loads(arguments.report.read_text())

    if not arguments.quiet:
        print_table(report)

    if arguments.junit:
        arguments.junit.parent.mkdir(parents=True, exist_ok=True)
        to_junit(report).write(arguments.junit, encoding="utf-8", xml_declaration=True)
        print(f"wrote JUnit XML to {arguments.junit}")

    return 0 if report.get("verdict") == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
