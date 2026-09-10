# Per-component evaluation of `autoware_core`

**Goal: verify the performance of each component inside `autoware_core` when it runs against
various scenarios - and say precisely which component regressed when something does.**

This document is the specification. The implementation is
`evaluation/autoware_core_component_evaluator`; the declarations are
`evaluation/autoware_core_component_evaluator/expectation/*.yaml`.

---

## 1. The idea

A scenario result is a single bit: the ego reached the goal, or it did not. That bit is nearly
useless for component quality. It does not distinguish a planner that got 20% slower from one that
crashed, and it says nothing at all about a component whose output was wrong but survivable.

So every component is given an **executable contract**, and the contract is evaluated on every
scenario run:

```
    what the component must publish        →  interface conformance
    how fast, how steadily, how fresh      →  timing
    what a valid message looks like        →  per-message validity
    how well it did its actual job         →  derived metrics, against ground truth
```

Each of the four is separately attributable. `/planning/trajectory` below 8 Hz is a *planning*
failure; a NaN inside it is a *different* planning failure; the ego drifting 2 m off it is a
*control* failure. All three are invisible to "did it reach the goal".

## 2. Two sources of truth, joined

The structural half of every contract entry - interface name, message type, topic-or-service,
QoS - is **not written here**. `autoware_core` already declares it, in
`autoware_component_interface_specs/interface_manifest.json`, generated from its own C++ specs and
version-controlled beside the code. Re-typing those 34 entries into this repo would create a second
source of truth that rots the first time core renames a topic.

The performance half - rates, gaps, jitter, latency, validity, gates - is not in the manifest and
has nowhere else to live. That is what this repo declares.

```
    contract entry  =  manifest entry (type, kind, qos)  ×  expectation entry (rates, checks, gates)
```

An expectation may name an interface the manifest does not declare - core publishes plenty of
load-bearing topics it has not promoted - but it must then carry its own `type:`, and it is
reported as **undeclared**. Seven currently are:

| Undeclared interface | Why it matters |
|---|---|
| `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` | a component boundary, and it changed message package once already |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | the boundary that isolates `behavior_velocity_planner` |
| `/perception/object_recognition/detection/objects` | the boundary between detection and the converter |
| `/perception/obstacle_segmentation/pointcloud` | three planning modules subscribe to it |
| `/api/operation_mode/state` | the interface `autoware_core` does not implement at all |
| `/api/localization/initialization_state`, `/api/routing/state` | the rest of the lifecycle triple |

Each is a candidate for an upstream `autoware_component_interface_specs` entry. The test suite
pins the set, so it fails - usefully - when core declares one.

## 3. What is measured, per interface

| Measurement | Why it earns its place |
|---|---|
| `rate_hz` | Throughput. On its own, the least informative of the four. |
| **`max_gap_s`** | The worst inter-arrival gap, *including the silence before the first message and after the last*. This is the one that matters: a node that stalls for 1.2 s inside a 60 s run still reports a healthy mean rate. Every stall-shaped regression hides from a rate check and is caught here. |
| `jitter_s` | Separates "slow" from "unstable". A planner at 9.8 Hz with 2 ms jitter is fine; at 9.8 Hz with 40 ms jitter it is scheduling-bound and will behave differently under load. |
| `latency_ms` | `header.stamp` → receipt. Reported as `NaN` - never `0` - when the type carries no stamp, so the absence stays visible. |

`rate_hz` divides by the **evaluation window**, not by the observed span. Dividing by the span
would flatter a topic that published healthily for five seconds and then died. A topic that never
publishes reports its `max_gap_s` as the whole window, not `0`.

## 4. Per-message validity

The checks that a rate cannot see. Declared per interface, so each runs only where it means
something.

| Check | Catches |
|---|---|
| `finite` | **NaN/Inf anywhere in the message.** The highest-value check in the whole contract: a NaN in a trajectory propagates through the controller without raising anything and presents as a vehicle that quietly stops or veers. |
| `nonempty` | An empty trajectory or path published at a flawless 10 Hz - the exact signature of an ego spawned off-lanelet. |
| `arc_length_monotonic` | Trajectory points that jump or reverse. |
| `velocity_bounds` | Negative longitudinal velocity on a forward plan. Not a slow plan - a sign error the controller will act on. |
| `control_bounds` | Steering or acceleration outside physical plausibility. |
| `pose_plausible` | Non-finite or out-of-range pose, and an unnormalised quaternion - the classic cause of a heading error that looks like a control bug. |
| `covariance_valid` | Negative covariance diagonals, which make every consumer's filter diverge. |
| `objects_plausible` | Zero-dimension objects and classification probabilities outside `[0, 1]`. A zero-dimension object passes every finiteness check and makes every downstream distance meaningless. |
| `map_nonempty` | A `LaneletMapBin` with an empty payload, i.e. the map failed to load and everything after that point is a consequence rather than a cause. |

Failures are counted, not averaged: one bad message in a thousand is still a finding, and the
first offending detail is kept for the report.

## 5. Derived metrics - the ones only simulation can produce

`/simulation/entity/status` carries the exact pose of every entity, ego included. That makes four
classes of measurement possible that no unit test and no vehicle log can produce:

| Class | Metrics |
|---|---|
| **Localization accuracy vs ground truth** | `localization_ate_m` (RMS position error), `localization_max_position_error_m`, `localization_heading_error_rad`, `localization_max_pose_jump_m` |
| **Perception recall vs known objects** | `perception_object_recall`, `perception_mean_object_count` |
| **Tracking error** | `control_max_lateral_deviation_m` (segment-wise distance to the trajectory polyline), `control_mean_speed_error_mps` |
| **Lifecycle timing** | `localization_time_to_initialized_s`, `api_time_to_route_set_s`, `planning_time_from_route_to_trajectory_s`, `system_time_to_autonomous_s`, `system_time_from_autonomous_to_motion_s`, `planning_time_to_arrival_s` |
| **Comfort** | `control_max_accel_mps2`, `control_max_jerk_mps3` - taken from successive *commands*, so they measure what control asked for rather than what the vehicle model produced |

`planning_time_from_route_to_trajectory_s` is the tightest of the timing metrics and the most
useful: it isolates planning's own latency from core's whole bring-up, and it is what moves when a
planning module gets slower.

`system_time_from_autonomous_to_motion_s` exists because "engaged but not moving" is the most
common integration failure and the least self-explanatory one. Bounding it turns a 180 s scenario
timeout into a named failure.

Every derived metric degrades to `NaN` when its inputs were absent, and `NaN` is reported as
*not measured* rather than as a pass.

## 6. Profiles

Not every component runs in every configuration.

| Profile | Localization | Perception | What it proves |
|---|---|---|---|
| `P0` | faked by the simulator | detected objects injected | planning + control + AD API close the loop |
| `P2` | core's `ekf_localizer` / `stop_filter` / `twist2accel` | injected | localization accuracy against ground truth |
| `P3` | core's own | core's `ground_filter` + clustering from raw LiDAR | full stack |

`required_in:` lists the profiles in which an interface or metric is judged. Outside those it is
`NOT_APPLICABLE` - **not** a pass and **not** a failure. Keeping those distinct is the difference
between a report a reviewer trusts and a wall of green.

The test suite enforces that P0 is the baseline: anything required in P0 must remain required in
P2 and P3, so a metric cannot silently stop being judged as the profile gets more demanding.

## 7. Output - three consumers, three forms

| Topic / file | Type | For |
|---|---|---|
| `/simulation/evaluation/report` | `autoware_component_evaluation_msgs/EvaluationReport` | the full structured result, live |
| `/simulation/evaluation/metrics` | `autoware_internal_metric_msgs/MetricArray` | AWF-native, so anything in the ecosystem that already consumes Autoware metrics gets these for free |
| `/metrics/<name>` | `autoware_scenario_simulation_msgs/UserDefinedValue` | **scenario gates** |
| `/tmp/core_component_report.json` | JSON | `evaluation_report` → table + JUnit XML |

The third is what turns a report into a gate. `openscenario_interpreter`'s
`UserDefinedValueCondition` binds any topic matching `^(?:/[\w-]+)*/([\w]+)$` carrying that type,
so **every one of the 52 metrics can be written straight into a scenario's stop trigger with no
interpreter change** - and a component regression then fails the scenario, and lands in the same
JUnit XML, exactly like a functional one.

`scenarios/core_all_components.yaml` promotes ten of them to gates, at least one per component.

The JUnit output puts one `<testcase>` per interface and per metric inside a `<testsuite>` per
component, so a red CI run names `planning./planning/trajectory` or
`control.control_max_lateral_deviation_m`. **The component that regressed is in the failure name**,
and nobody has to open a bag to find out.

## 8. Coverage - 9 components, 29 interfaces, 52 metrics

| Component | Interfaces | Metrics | Headline |
|---|---|---|---|
| `map` | 2 | 3 | map loaded and non-empty; projector present |
| `sensing` | 2 | 3 | velocity converter rate and stalls |
| `localization` | 4 | 9 | rate, jitter, **ATE / heading error vs ground truth**, pose jumps |
| `perception` | 2 | 6 | rate, latency, object validity, **recall vs known objects** |
| `planning` | 7 | 10 | per-stage rates across the whole chain, **route→trajectory latency**, malformed-trajectory count |
| `control` | 3 | 10 | rate, jitter, **lateral deviation**, speed error, accel/jerk |
| `vehicle` | 4 | 4 | status rates; control mode reached AUTONOMOUS |
| `system` | 2 | 3 | operation mode published; engage→motion latency |
| `api` | 3 | 4 | the three lifecycle topics exist |

## 9. Deliberately out of scope

Stating this is what makes the coverage claims credible.

| Not covered | Why |
|---|---|
| **NDT closed-loop localization** | Structurally impossible with this simulator: its LiDAR model raycasts spawned entities only, never the map, so a scan matcher has no environment geometry to align against. A property of the simulator, not a gap in core. Needs AWSIM or a map-aware raycaster. |
| `ground_filter` functional accuracy | Same root cause - no ground plane exists in the raycast. |
| Traffic light behaviour | Core has no traffic light arbiter; `behavior_velocity_planner` launches only `StopLineModulePlugin`. |
| RTC / cooperation | Core launches no module that requests cooperation. Permanently N/A. |
| Lane change / intersection / crosswalk | Not in core's module set. |
| Multi-run repeatability | Not yet implemented. The obvious next metric: fixed seeds, N runs, max Fréchet distance between ego paths - a number quantifying core's concurrency sensitivity that no unit test can produce. |
