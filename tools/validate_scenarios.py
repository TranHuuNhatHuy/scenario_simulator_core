#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors. Apache-2.0
"""
Validate scenarios/*.yaml against the real OpenSCENARIO XSD using SSV2's own converter.

Worth running before every scenario change: the OpenSCENARIO schema is unforgiving, and a
structurally invalid scenario fails deep inside openscenario_interpreter with a message that
does not obviously point at the YAML.

    python3 tools/validate_scenarios.py          # exits non-zero on any failure
"""

import sys
import tempfile
from pathlib import Path

try:
    from openscenario_utility.conversion import convert
except ImportError:
    sys.exit("openscenario_utility not importable -- source setup_env.sh first")

GREEN, RED, RESET = "\033[32m", "\033[31m", "\033[0m"


def main():
    root = Path(__file__).resolve().parent.parent
    files = sorted((root / "scenarios").glob("*.yaml"))
    if not files:
        sys.exit("no scenarios/*.yaml found")

    out = Path(tempfile.mkdtemp())
    failures = 0
    print(f"validating {len(files)} scenario(s) against the OpenSCENARIO XSD\n")
    for f in files:
        try:
            produced = convert(f, out, False)
            print(f"  [{GREEN}OK{RESET}]   {f.name} -> {', '.join(p.name for p in produced)}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"  [{RED}FAIL{RESET}] {f.name}: {type(exc).__name__}: {str(exc)[:300]}")

    print()
    if failures:
        print(f"{RED}{failures} scenario(s) invalid{RESET}")
    else:
        print(f"{GREEN}all scenarios valid{RESET}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
