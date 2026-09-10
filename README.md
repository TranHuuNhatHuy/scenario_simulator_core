# scenario_simulator_core

**Scenario-based testing and per-component evaluation for `autoware_core`, on Autoware Foundation
interfaces only.**

This repository is an **overlay**. It contains 4 ROS packages, 2 scenarios, 5 tools and its own
documentation - and **no copy of any upstream repository**. `scenario_simulator_v2` and
`autoware_core` are fetched at build time by `dependency.repos`, the same way both of those
projects fetch their own dependencies.

```
66 files · 4 packages · ~6600 lines · 0 files copied from scenario_simulator_v2
                                     · 3 files from autoware_core, each named in NOTICE
```

---

## 1. How this composes

Three pieces, and the separation between them is the point:

```
  ┌─ scenario_simulator_core ───────────────────────────────────────────┐
  │  THIS repository. 100 % this project's work.                        │
  │                                                                     │
  │  evaluation/   the per-component contract + KPI engine              │
  │  msgs/         the evaluation data model                            │
  │  core_adapter/ the autoware_core profile launcher + the AD API gap  │
  │  scenarios/ tools/ docs/                                            │
  └────────────────────────────┬────────────────────────────────────────┘
                               │ dependency.repos
       ┌───────────────────────┴───────────────────────┐
       ▼                                               ▼
  scenario_simulator_v2                          autoware_core
  fork branch `feat/awf-core`                    autowarefoundation/main
  = tier4/master + 6 commits                     unmodified
  (docs/FORK.md - each one a PR)
```

**Why not one repository with everything in it?** 

A vendored copy of `scenario_simulator_v2`
would bury this project's ~6600 lines inside ~215 000 lines of someone else's, make every upstream
release a merge exercise, and give a TIER IV reviewer no way to tell a contribution from a copy.
The six changes this project genuinely needs to the simulator are carried where changes belong -
as commits on a branch, with upstream's history underneath them:

```bash
git log  --oneline upstream/master..HEAD   # 6 commits, all authored here
git diff --stat     upstream/master..HEAD  # >50 files
git rebase          upstream/master        # how a new release is picked up
```

Each of those six is a pull request as it stands. When they land upstream, the branch shrinks;
when the last lands, `dependency.repos` points back at `tier4/scenario_simulator_v2` and the fork
is deleted. [docs/FORK.md](docs/FORK.md) lists them, their independent value to upstream, and the
two questions worth settling before filing.

A nightly CI job rebases that branch onto `tier4/master` and fails on conflict. It pushes nothing -
it exists so drift is found while it still costs ten minutes.

---

## 2. Quick start

```bash
pip install --user vcstool
./bootstrap.sh                 # fetch the simulator, autoware_core and the AWF messages into ./ws
source setup_env.sh
cd ws && colcon build --symlink-install --packages-up-to autoware_core scenario_simulator_v2 \
    autoware_core_component_evaluator autoware_core_adapi_compat autoware_core_scenario_launch \
    openscenario_experimental_catalog \
    autoware_sample_vehicle_description autoware_sample_sensor_kit_description
# The last three are in the fetched sources but nothing DEPENDS on them, so
# --packages-up-to never reaches them. They are resolved at runtime instead:
# the catalog by $(ros2 pkg prefix) inside a scenario, the two descriptions by
# autoware_global_parameter_loader from vehicle_model / sensor_model.
cd ..

./run_scenario.sh                       # all-component scenario, per-component gates
SCENARIO=core_smoke ./run_scenario.sh   # minimal closed loop, no gates
PROFILE=P2 ./run_scenario.sh            # core's own localization in the loop

ros2 run autoware_core_component_evaluator evaluation_report
ros2 run autoware_core_component_evaluator evaluation_report --junit /tmp/evaluation.xml
```

Debugging without the simulator at all - `tools/handdrive.py` is the highest-value tool here,
because it separates *"does `autoware_core` work"* from *"does the integration work"*. If
`/planning/trajectory` never appears under handdrive, the problem is core, the map or the spawn
pose, not this repository.

```bash
python3 tools/handdrive.py         # fake vehicle + localization, no simulator
python3 tools/verify_lifecycle.py  # drive the AD API lifecycle by hand
python3 tools/map_coords.py        # lanelet id + s -> map-frame x/y/yaw
python3 tools/validate_scenarios.py
```

> Guessing spawn coordinates is a trap: an off-lane ego makes `path_generator` silently emit
> nothing, which is indistinguishable from a broken integration. Get them from `map_coords.py`.

---

## 3. What this project does

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

### 3.2 Autoware Foundation interfaces only

`scenario_simulator_v2` cannot currently be built outside TIER IV without a TIER IV message
repository. Commits 4–6 on the fork branch remove all six such dependencies:

| Removed | Replaced by |
|---|---|
| `tier4_planning_msgs/PathWithLaneId` | `autoware_internal_planning_msgs/PathWithLaneId` |
| `tier4_simulation_msgs/*` | `autoware_scenario_simulation_msgs/*` (new; no AWF equivalent exists) |
| `tier4_debug_msgs` | `autoware_internal_debug_msgs` |
| `tier4_external_api_msgs/Engage` | AD API `/api/operation_mode/change_to_autonomous` |
| `tier4_external_api_msgs/SetVelocityLimit` | `autoware_internal_planning_msgs/VelocityLimit`, latched |
| `tier4_external_api_msgs/ResponseStatus` | `autoware_common_msgs` / `autoware_adapi_v1_msgs` `ResponseStatus` |
| `tier4_external_api_msgs/Emergency` | `autoware_adapi_v1_msgs/MrmState` |
| `tier4_rtc_msgs/*` | AD API cooperation (planning factors + `SetCooperation{Commands,Policies}`) |

Three are genuine remodellings rather than renames; what changed in each, and what was lost, is in
[docs/AWF_INTERFACE_MIGRATION.md](docs/AWF_INTERFACE_MIGRATION.md). Legacy `tier4_rtc_msgs` module
names still work in scenarios, as aliases.

`awf/core/1.0.0` also becomes a first-class `architecture_type` and the default, replacing eight
scattered `find("awf/universe")` tests with one predicate - which is what removes the need to
*lie* about the architecture in order to run against `autoware_core`.

### 3.3 The one remaining shim, and its retirement plan

`core_adapter/autoware_core_adapi_compat` - the six `/api/operation_mode/*` AD API services
`autoware_core` does not implement, plus the latched `OperationModeState` they publish.

*Why it must exist.* `concealer` derives its whole lifecycle from three AD API topics.
`autoware_default_adapi` implements interface, localization and routing only, so
`/api/operation_mode/state` has **no publisher at all**, `LegacyAutowareState` is pinned at
`INITIALIZING` forever, and every scenario times out with no other symptom. `change_to_stop` is
called in `FieldOperatorApplication`'s constructor **unconditionally**, and `service.hpp` waits
180 s then throws - so nothing works before any scenario logic runs.

*Disposition.* Every endpoint graduates to an upstream `autoware_default_adapi` `OperationModeNode`.
**Two bridges this package used to carry are already gone**, deleted because the interface
migration removed the interfaces they bridged:

| Was | Now |
|---|---|
| `/api/external/set/engage` | concealer engages via AD API `change_to_autonomous`, already served here |
| `/api/autoware/set/velocity_limit` | concealer publishes `VelocityLimit` on the topic `velocity_smoother` already reads |

That is the migration paying for itself. Report this package as a shrinking line count, not a
feature.

---

## 4. Provenance

### From `scenario_simulator_v2` - **nothing in this repository**

Fetched via `dependency.repos`. The six required changes are commits on
`TranHuuNhatHuy/scenario_simulator_v2:feat/awf-core`, each carrying TIER IV's copyright headers
unmodified and stating what changed and why. See [docs/FORK.md](docs/FORK.md) and [NOTICE](NOTICE).

Two scenarios are *derived* from upstream's `sample.yaml` and say so in their own headers.
`scenarios/core_smoke.yaml` removes upstream's
`currentMinimumRiskManeuverState == NORMAL` assertion: it needs `/api/fail_safe/mrm_state`, which
core has no MRM to publish, so it could never be satisfied.

### From `autoware_core` - 3 files, each named in [NOTICE](NOTICE)

| File | Relationship |
|---|---|
| `evaluation/.../test/interface_manifest.reference.json` | unmodified copy, **test fixture only** - never read at runtime, because a stale snapshot silently accepted as the contract would defeat the join |
| `core_adapter/.../config/pose_initializer_p0.param.yaml` | derived; covariances and thresholds verbatim, the five estimator flags resolved to `false` |
| `core_adapter/.../launch/pose_initializer_only.launch.xml` | derived; all ten remappings verbatim, so the node binds the names core binds |

Everything else core contributes is **referenced, not copied**: the profile launcher `include`s
core's own launch files and reads its configs through `allow_substs`, so core's parameters always
match core's binaries. Hand-writing them breaks the next time core adds a parameter.

The contract thresholds were derived by **reading core's source**, and each is traceable:
`path_generator`'s `planning_hz: 10.0` → the 8 Hz planning floors; `simple_pure_pursuit`'s 30 ms
timer → the 20 Hz control floor; `behavior_velocity_planner` publishing
`autoware_planning_msgs/Path` → the type on that contract entry.

### Written for this project - everything else

| Package | Lines | What it is |
|---|---:|---|
| `evaluation/autoware_core_component_evaluator` | 1723 py + 822 yaml | the contract engine, the 9 invariants, the derived metrics, the report tool |
| `msgs/autoware_component_evaluation_msgs` | 77 | the evaluation data model - `Verdict` distinguishes `NOT_OBSERVED` from `NOT_APPLICABLE` from `FAIL`, and reuses `autoware_internal_metric_msgs/Metric` rather than defining a parallel type |
| `core_adapter/autoware_core_adapi_compat` | 306 | the AD API operation-mode gap |
| `core_adapter/autoware_core_scenario_launch` | 311 | the profile launcher |
| `scenarios/`, `tools/`, `docs/`, CI, bootstrap | ~1400 | 2 scenarios, 5 tools, 3 design documents, 2 workflows |

---

## License

Apache License 2.0. See [LICENSE](LICENSE) for the licence and [NOTICE](NOTICE) for attribution of the derived
files and the fork branch.
