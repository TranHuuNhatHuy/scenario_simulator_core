# Component expectations

One file per `autoware_core` component. Each declares, for that component:

* **which interfaces it must serve**, in which profile, and how well — rate, worst tolerable
  gap, jitter, latency, and which per-message validity checks apply;
* **which metrics it is judged on**, and the bounds that make them pass or fail.

The *structural* half of each interface — message type, topic-or-service, QoS — is not repeated
here. It is read at runtime from `autoware_component_interface_specs/interface_manifest.json`,
which `autoware_core` generates from its own C++ interface specs. An interface that core has not
declared there must carry an explicit `type:`, and is reported as `declared: false` so the gap
stays visible: each one is a candidate for an upstream interface-spec entry, and until it has
one, nothing stops core renaming it in a patch release.

## Profiles

| Profile | Localization | Perception | What it proves |
|---|---|---|---|
| `P0` | faked by the simulator | detected objects injected by the simulator | planning + control + API close the loop |
| `P2` | core's own `ekf_localizer` / `stop_filter` / `twist2accel` | injected | localization accuracy against ground truth |
| `P3` | core's own | core's own `ground_filter` + clustering from raw LiDAR | full stack |

`required_in:` lists the profiles in which an interface or metric is judged. Outside those, it is
reported `NOT_APPLICABLE` rather than passed or failed — an interface that is absent because the
profile does not launch its component is not evidence of anything.

## Schema

```yaml
component: planning              # must match the AWF interface manifest's `domain`
title: Planning
description: >
  Prose, shown in the generated report.
interfaces:
  - name: /planning/trajectory   # topic or service name
    type: ...                    # only when core does not declare it
    from_manifest: true          # default; false asserts "deliberately undeclared"
    direction: output            # output | input
    required_in: [P0, P2, P3]
    rate: {min_hz: 8.0, max_hz: 60.0}
    max_gap_s: 0.5
    max_jitter_s: 0.05
    latency_max_ms: 300
    checks: [finite, nonempty]   # see ../autoware_core_component_evaluator/checks.py
    note: why this bound
metrics:
  - name: planning_trajectory_rate_hz
    unit: Hz
    interface: /planning/trajectory
    statistic: rate_hz           # rate_hz | max_gap_s | jitter_s | latency_*_ms | count | observed | check_failures
    gate: {min: 8.0}
    required_in: [P0, P2, P3]
  - name: planning_time_to_first_trajectory_s
    unit: s
    derived: time_to_first_trajectory_s   # see ../autoware_core_component_evaluator/derived.py
    gate: {max: 30.0}
```

Every metric is published on `/metrics/<name>` as `tier4_simulation_msgs/UserDefinedValue`, so
it can be used directly in a scenario's `UserDefinedValueCondition` with no interpreter change.
Metric names must therefore match `[A-Za-z0-9_]+`.
