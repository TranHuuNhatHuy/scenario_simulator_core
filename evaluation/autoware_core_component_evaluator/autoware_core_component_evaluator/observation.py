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
Per-interface measurement.

WHAT IS MEASURED AND WHY EACH ONE EARNS ITS PLACE
-------------------------------------------------

rate_hz        Throughput. The obvious one, and on its own the least informative.

max_gap_s      The worst inter-arrival gap. This is the measurement that matters most: a node
               that stalls for 1.2 s in the middle of a 60 s run still reports a healthy mean
               rate. Every stall-shaped regression hides from a rate check and is caught here.

jitter_s       Standard deviation of inter-arrival time. Separates "slow" from "unstable"; a
               planner at 9.8 Hz with 2 ms jitter is fine, one at 9.8 Hz with 40 ms jitter is
               scheduling-bound and will behave differently under load.

latency_ms     header.stamp -> local receipt. Only meaningful for stamped types, and only
               within one machine's clock, which is the case here. NaN when the type carries no
               header, and reported as NaN rather than 0 so the absence is visible.

count          Sanity, and the denominator for everything above.

Timing uses the ROS clock, so `use_sim_time` runs and real-time runs are measured the same way
and a rate expectation means the same thing in both.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def _stddev(values: list[float]) -> float:
    if len(values) < 2:
        return math.nan
    mean = _mean(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


@dataclass
class Observation:
    """Everything measured about one interface over one run."""

    name: str
    domain: str
    type: str
    kind: str

    count: int = 0
    first_seen_s: float | None = None
    last_seen_s: float | None = None

    gaps: list[float] = field(default_factory=list)
    latencies_ms: list[float] = field(default_factory=list)

    # Graph facts, refreshed periodically rather than per message.
    publisher_count: int = 0
    subscriber_count: int = 0
    observed_types: set[str] = field(default_factory=set)
    observed_qos: list[dict] = field(default_factory=list)

    # Validity, accumulated: a single bad message must not be averaged away.
    check_failures: dict[str, int] = field(default_factory=dict)
    check_first_detail: dict[str, str] = field(default_factory=dict)
    checks_run: int = 0

    # ------------------------------------------------------------------ recording ----
    def record(self, receipt_s: float, stamp_s: float | None) -> None:
        if self.last_seen_s is not None:
            self.gaps.append(receipt_s - self.last_seen_s)
        else:
            self.first_seen_s = receipt_s
        self.last_seen_s = receipt_s
        self.count += 1

        if stamp_s is not None and stamp_s > 0.0:
            self.latencies_ms.append((receipt_s - stamp_s) * 1e3)

    def record_check_failure(self, check: str, detail: str) -> None:
        self.check_failures[check] = self.check_failures.get(check, 0) + 1
        self.check_first_detail.setdefault(check, detail)

    # ------------------------------------------------------------------ statistics ----
    def rate_hz(self, window_s: float) -> float:
        """Messages per second over the *evaluation window*, not over the observed span.

        Dividing by the observed span would flatter a topic that started late or died early --
        a node that published for the first 5 s of a 60 s run and then stopped would report its
        healthy startup rate. Dividing by the window makes that read as the near-zero rate it is,
        and `max_gap_s` names the stall.
        """
        return self.count / window_s if window_s > 0.0 else math.nan

    def max_gap_s(self, window_s: float, now_s: float) -> float:
        """Worst silence, including the silence before the first message and after the last.

        A topic that never publishes has no gaps to measure; treating that as 0.0 would let a
        dead component pass a max-gap check. It is reported as the whole window instead.
        """
        candidates = list(self.gaps)
        if self.first_seen_s is None:
            return window_s
        candidates.append(self.first_seen_s - (now_s - window_s))
        candidates.append(now_s - self.last_seen_s)
        return max(candidates) if candidates else window_s

    def jitter_s(self) -> float:
        return _stddev(self.gaps)

    def latency_mean_ms(self) -> float:
        return _mean(self.latencies_ms)

    def latency_max_ms(self) -> float:
        return max(self.latencies_ms) if self.latencies_ms else math.nan

    def statistic(self, name: str, window_s: float, now_s: float) -> float:
        return {
            "rate_hz": lambda: self.rate_hz(window_s),
            "max_gap_s": lambda: self.max_gap_s(window_s, now_s),
            "jitter_s": self.jitter_s,
            "latency_mean_ms": self.latency_mean_ms,
            "latency_max_ms": self.latency_max_ms,
            "count": lambda: float(self.count),
            "observed": lambda: 1.0 if self.count > 0 else 0.0,
            "publisher_count": lambda: float(self.publisher_count),
            "check_failures": lambda: float(sum(self.check_failures.values())),
        }.get(name, lambda: math.nan)()


def stamp_seconds(message: Any) -> float | None:
    """Seconds from whichever stamp the message carries, or None when it carries none.

    Autoware messages put the stamp in one of three places depending on vintage: a std_msgs
    Header, a bare builtin_interfaces/Time field named `stamp`, or nothing at all.
    """
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else getattr(message, "stamp", None)
    if stamp is None:
        return None
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except AttributeError:
        return None
