# scenario_simulator_core

**Scenario-based testing and per-component evaluation for `autoware_core`, running on pure Autoware Foundation interfaces.**

This repository is an **overlay**. It contains this project's custom tools, component evaluators, and launch configurations—without maintaining any vendored forks of upstream repositories. `scenario_simulator_v2` and `autoware_core` are fetched at build time by `dependency.repos`.

```
66 files · 3 packages · ~6600 lines · 0 files copied from scenario_simulator_v2
```

---

## 1. How this composes

Three pieces, and the separation between them is the point:

```
  ┌─ scenario_simulator_core ───────────────────────────────────────────┐
  │  THIS repository. 100% this project's work.                         │
  │                                                                     │
  │  evaluation/   the per-component contract + KPI engine              │
  │  msgs/         the evaluation data model                            │
  │  core_adapter/ the autoware_core profile launcher                   │
  │  scenarios/ tools/ docs/                                            │
  └────────────────────────────┬────────────────────────────────────────┘
                               │ dependency.repos
       ┌───────────────────────┴───────────────────────┐
       ▼                                               ▼
  scenario_simulator_v2                          autoware_core
  tier4/master                                   autowarefoundation/main
  unmodified, out-of-the-box                     with native AD API updates
```

**Zero Forks.** Earlier versions of this project maintained an external fork of `scenario_simulator_v2` to bridge AWF/TIER4 message types and inject an adapter node. However, this has been entirely replaced by a native approach:
1. `autoware_core` itself has been natively updated to support the AD API Operation Modes.
2. `scenario_simulator_v2` out-of-the-box (`awf/universe/20250130` architecture) natively supports `autoware_internal_planning_msgs`. 

As a result, no forks, adapter nodes, or message shims are required.

---

## 2. How to Build and Test

### Prerequisites
Make sure you have standard ROS 2 (this project has been tested on 22.04 Humble) installed, and `vcstool` available:
```bash
pip install --user vcstool
```

### Step 1. Clone
```bash
git clone git@github.com:TranHuuNhatHuy/scenario_simulator_core.git
cd scenario_simulator_core
```

### Step 2. Fetch Codebase
Run the bootstrap script to create the workspace (`ws/`) and clone `autoware_core`, `scenario_simulator_v2`, and other dependencies:
```bash
./bootstrap.sh
```

### Step 3. Build
From the project root, source the environment and build the required packages:
```bash
source setup_env.sh
cd ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to \
    autoware_core \
    scenario_simulator_v2 \
    autoware_core_component_evaluator \
    autoware_core_scenario_launch \
    openscenario_experimental_catalog \
    autoware_sample_vehicle_description \
    autoware_sample_sensor_kit_description
cd ..
```

### Step 4. Run Scenarios
You can easily test integration by running `run_scenario.sh`. This script handles the complex `scenario_test_runner` launch commands internally.

```bash
# Run the complete scenario testing loop
./run_scenario.sh

# Run minimal smoke test (no gates)
SCENARIO=core_smoke ./run_scenario.sh

# Run with core's native localization instead of faked simulation
PROFILE=P2 ./run_scenario.sh

# Run with Rviz visualization enabled
LAUNCH_RVIZ=true ./run_scenario.sh
```

Once completed, the component evaluator will parse the metrics and output a JUnit evaluation report in `/tmp/core_component_report.json` and print a summary to the console.

---

## 3. What this repository contains

### 3.1 Per-component evaluation

Every `autoware_core` component gets an **executable contract**, evaluated on every scenario run:
**9 components, 29 interfaces, 52 metrics**.

```
what it must publish        →  interface conformance  (against core's own interface manifest)
how fast, steadily, fresh   →  rate, worst gap, jitter, latency
what a valid message is     →  9 per-message invariants
how well it did its job     →  derived metrics, against simulator ground truth
```

**Two sources of truth, joined.** The structural half of each contract entry - name, message type,
topic-or-service, QoS - is not written here. It is read at runtime from `autoware_core`'s own
`autoware_component_interface_specs/interface_manifest.json`. This repository declares only the
performance half, which the manifest does not carry.

*Why.* Re-typing core's 34 declared interfaces would create a second source of truth that rots
silently the first time core renames a topic. The join turns that rename into a loud failure
instead of a contract that checks nothing.

*What falls out of it.* An expectation may name an interface core has **not** declared, but it
must then carry its own `type:` and is reported as `declared: false`. **Seven currently are** - and
a test pins that exact set, so it fails, usefully, when core declares one:

```
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
/planning/scenario_planning/lane_driving/behavior_planning/path
/perception/object_recognition/detection/objects
/perception/obstacle_segmentation/pointcloud
/api/operation_mode/state
/api/localization/initialization_state
/api/routing/state
```

The evaluator doubles as a gap report on `autoware_core`'s own interface specification.

**The measurement that earns its place.** `max_gap_s`, not `rate_hz`: a node that stalls for 1.2 s
inside a 60 s run still reports a healthy mean rate, so every stall-shaped regression hides from a
rate check. Two deliberate consequences - rate divides by the *evaluation window* rather than the
observed span, so a topic that published for five seconds and died reads as the near-zero rate it
is; and a topic that never publishes reports its worst gap as the **whole window**, not `0`, so a
dead component cannot pass a gap check.

**What only simulation can measure.** The simulator publishes exact ground truth on
`/simulation/entity/status`:

| Metric | Why it is impossible elsewhere |
|---|---|
| `localization_ate_m`, `localization_heading_error_rad` | a **true** absolute trajectory error, not the self-consistency a covariance reports |
| `perception_object_recall` | scored against the objects that actually exist |
| `control_max_lateral_deviation_m` | segment-wise distance to the trajectory polyline - nearest-vertex would overestimate systematically on core's ~0.5 m spacing |
| `planning_time_from_route_to_trajectory_s` | planning's own latency, isolated from bring-up; the number that moves when a planning module slows down |
| `system_time_from_autonomous_to_motion_s` | *engaged but not moving* is the most common integration failure and the least self-explanatory; bounding it turns a 180 s timeout into a named failure |

Every derived metric degrades to `NaN` when its inputs were absent, and `NaN` reports as **not
measured** - never as a pass.

**Every metric is a scenario gate, for free.** The evaluator publishes each one on
`/metrics/<name>` as `autoware_scenario_simulation_msgs/UserDefinedValue`.
`openscenario_interpreter`'s `UserDefinedValueCondition` binds any topic matching
`^(?:/[\w-]+)*/([\w]+)$` carrying that type, so all 52 go straight into a scenario's stop trigger
with no interpreter change:

```yaml
UserDefinedValueCondition:
  name: /metrics/control_max_lateral_deviation_m
  rule: lessThan
  value: '1.0'
```

A component regression then fails the scenario, and lands in the same JUnit XML, exactly like a
functional one. The JUnit names `planning./planning/trajectory` and
`control.control_max_lateral_deviation_m` - **the component that regressed is in the failure
name.** `scenarios/core_all_components.yaml` gates on ten, at least one per component.

**Profiles.** `P0` (simulator fakes localization and detection) · `P2` (core's own localization) ·
`P3` (core's own perception from raw LiDAR). An interface required in P2 and absent in P0 is
`NOT_APPLICABLE` - **not a pass, not a failure**. Keeping those distinct is the difference between
a report a reviewer trusts and a wall of green.

Full specification: [docs/COMPONENT_EVALUATION.md](docs/COMPONENT_EVALUATION.md).

### 3.2 Native Architecture Integration
Instead of building "shims" on the outside, this project proves that `autoware_core` can interact with `scenario_simulator_v2` out of the box using pure AWF interfaces:
- **Planning Messages:** `scenario_simulator_v2` natively supports `autoware_internal_planning_msgs/PathWithLaneId` via macro conditional checks.
- **AD API Integration:** `autoware_core/api/autoware_default_adapi` is configured to natively handle modern `change_to_autonomous`, `enable_autoware_control`, and `change_operation_mode` service calls.
- **Two-Axis Command Gate:** `autoware_core` implements a two-axis `autoware_command_gate` that completely separates mode logic from system control logic. 

These native enhancements make simulator interaction completely transparent without any middleware node.

---

## 4. Provenance

### From `scenario_simulator_v2` - **nothing in this repository**
Fetched via `dependency.repos`. This overlay runs strictly against upstream `tier4/scenario_simulator_v2` branch `master`.

### From `autoware_core` - 3 files, each named in [NOTICE](NOTICE)

| File | Relationship |
|---|---|
| `evaluation/.../test/interface_manifest.reference.json` | unmodified copy, **test fixture only** - never read at runtime, because a stale snapshot silently accepted as the contract would defeat the join |
| `core_adapter/.../config/pose_initializer_p0.param.yaml` | derived; covariances and thresholds verbatim, the five estimator flags resolved to `false` |
| `core_adapter/.../launch/pose_initializer_only.launch.xml` | derived; all ten remappings verbatim, so the node binds the names core binds |

Everything else core contributes is **referenced, not copied**: the profile launcher `include`s core's own launch files and reads its configs through `allow_substs`, so core's parameters always match core's binaries. 

### Written for this project - everything else

| Package | Lines | What it is |
|---|---:|---|
| `evaluation/autoware_core_component_evaluator` | 1723 py + 822 yaml | the contract engine, the 9 invariants, the derived metrics, the report tool |
| `msgs/autoware_component_evaluation_msgs` | 77 | the evaluation data model |
| `core_adapter/autoware_core_scenario_launch` | 311 | the profile launcher |
| `scenarios/`, `tools/`, `docs/`, CI, bootstrap | ~1400 | scenarios, tools, workflows |

---

## License

Apache License 2.0. See [LICENSE](LICENSE) for the licence and [NOTICE](NOTICE) for attribution of the derived files.
