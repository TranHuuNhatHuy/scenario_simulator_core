# TIER IV interfaces → Autoware Foundation interfaces

**Goal: `scenario_simulator_core` must be usable by anyone in the Autoware Foundation community,
without a TIER IV message repository.**

Upstream `scenario_simulator_v2` reaches Autoware's control plane through six TIER IV message
packages. All six are removed by commits 4–6 on the `feat/awf-core` branch described in
[FORK.md](FORK.md); after them, `grep -rE '<[a-z_]*depend[^>]*>\s*tier4_' --include=package.xml` returns nothing across the
whole workspace, and this repository's `dependency.repos` has no `tier4_autoware_msgs` entry.

Nothing here is vendored. The changes below are commits on a fork of `scenario_simulator_v2`,
each one a pull request as it stands.

This document records what each removed interface was replaced by, and — more importantly — where
the replacement is not a rename but a change in behaviour.

---

## 1. Summary

| TIER IV interface | Used for | Replaced by | Kind of change |
|---|---|---|---|
| `tier4_planning_msgs/PathWithLaneId` | concealer subscribes to the behaviour path | `autoware_internal_planning_msgs/PathWithLaneId` | **rename** — identical fields |
| `tier4_simulation_msgs/UserDefinedValue`, `UserDefinedValueType` | scenario-visible metric values | `autoware_scenario_simulation_msgs/…` (defined on the fork branch) | **rename** — identical fields |
| `tier4_simulation_msgs/SimulationEvents`, `FaultInjectionEvent` | `FaultInjectionAction` | `autoware_scenario_simulation_msgs/…` (defined on the fork branch) | **rename** — identical fields |
| `tier4_debug_msgs` | declared dependency of `traffic_simulator` | `autoware_internal_debug_msgs` | **rename** |
| `tier4_external_api_msgs/Engage` (srv) | engage the vehicle | AD API `ChangeOperationMode` on `/api/operation_mode/change_to_autonomous` | **remodelled** |
| `tier4_external_api_msgs/SetVelocityLimit` (srv) | scenario `maxSpeed` | `autoware_internal_planning_msgs/VelocityLimit` published on `/planning/scenario_planning/max_velocity` | **remodelled** — service → topic |
| `tier4_external_api_msgs/ResponseStatus` | service success test | `autoware_common_msgs/ResponseStatus` / `autoware_adapi_v1_msgs/ResponseStatus` | **remodelled** — `code == SUCCESS` → `success` |
| `tier4_external_api_msgs/Emergency` | abort on emergency | `autoware_adapi_v1_msgs/MrmState` on `/api/fail_safe/mrm_state` | **remodelled** — folded into an existing subscriber |
| `tier4_rtc_msgs/*` (RTC) | cooperation with planning modules | AD API cooperation: factors + `SetCooperationCommands` / `SetCooperationPolicies` | **remodelled** — enum → behavior string, boolean → policy |
| `tier4_adapi_rviz_plugins::RouteTool` | an RViz tool entry | removed from the shipped RViz config | **removed** |

Three of these carry real semantic weight. They are the rest of this document.

---

## 2. Engage: a service call becomes an operation mode change

Upstream calls `tier4_external_api_msgs/Engage` on `/api/external/set/engage`, immediately after
`/api/operation_mode/enable_autoware_control`.

The AD API has no engage endpoint, because engaging *is* an operation mode change. `autoware_core`
implements no external API at all, so this is not merely tidier — it is the only form that works
on both stacks:

```
enable_autoware_control   →  vehicle is under Autoware control
change_to_autonomous      →  Autoware drives
```

`concealer::FieldOperatorApplication::engage()` now calls `requestChangeToAutonomous` where it
called `requestEngage`. `EgoEntity` already called `enableAutowareControl()` first, so the
sequence is unchanged; only the second call's identity changed.

**What is lost:** nothing observable. The `ResponseStatus` semantics differ (see below), and
concealer never used the response for state — it polls the AD API triple through
`waitForAutowareStateToBe()`.

## 3. Velocity limit: a service becomes a latched topic

`tier4_external_api_msgs/SetVelocityLimit` is a service. The AD API has **no velocity-limit
endpoint of any kind**, so there is nothing to map it onto. The AWF-native equivalent is the topic
`autoware_velocity_smoother` already subscribes to:

```
/planning/scenario_planning/max_velocity   autoware_internal_planning_msgs/VelocityLimit
```

published **transient-local**, because the limit is state rather than an event and a scenario
often sets `maxSpeed` before `velocity_smoother` is up.

**What is lost:** the acknowledgement. A scenario can no longer learn that the limit was rejected.
In practice OpenSCENARIO's `maxSpeed` is a constraint rather than a request, and the previous
acknowledgement only confirmed that the API node received the call — never that the planner
honoured it. `VelocityLimit::sender` is set to `SENDER_API` so a stack with several limit sources
can still attribute this one.

> This is the strongest argument in the whole migration for an **upstream AD API gap**: there is
> no way for any AD API client — simulator, bench rig, or remote operator — to set a velocity
> limit. Worth raising with the Architecture WG independently of this repo.

## 4. Response status: `code == SUCCESS` becomes `success`

`tier4_external_api_msgs/ResponseStatus` reports `uint32 code` with `SUCCESS = 1`.
Both AWF status types — `autoware_common_msgs/ResponseStatus` and
`autoware_adapi_v1_msgs/ResponseStatus` — carry a plain `bool success` alongside a `code` that is
an *error* code.

`concealer/include/concealer/service.hpp` previously branched on the status type and compared
against the right package's constant. It now has one branch for both AWF types. This removes a
real trap: `code == 1` means *success* in the TIER IV type and is a meaningless value in the AWF
types, so a mis-typed comparison would have been silently wrong rather than a compile error.

## 5. RTC → AD API cooperation: the largest remodelling

`tier4_rtc_msgs` models cooperation as `(Module enum, uuid) → Command{ACTIVATE, DEACTIVATE}`, with
status on `/api/external/get/rtc_status` and commands on `/api/external/set/rtc_commands`.

The AD API models it without a TIER IV message, but differently:

| | tier4_rtc_msgs | AD API |
|---|---|---|
| status | `CooperateStatusArray` on its own topic | `CooperationStatus` embedded in each `VelocityFactor` / `SteeringFactor` on `/api/planning/{velocity,steering}_factors` |
| identify a manoeuvre | `Module` enum (20 values) | `behavior` string (`PlanningBehavior`) + optional `sequence` |
| command | `CooperateCommands` srv | `SetCooperationCommands` srv on `/api/planning/cooperation/set_commands` |
| auto mode | `AutoModeWithModule` srv, boolean per module | `SetCooperationPolicies` srv, `OPTIONAL` / `REQUIRED` per behavior |

Consequences, all handled in `external/concealer/include/concealer/cooperation.hpp`:

* **Module names.** Every legacy `tier4_rtc_msgs::Module` spelling is kept as an alias for the AD
  API behavior it corresponds to, so scenarios written against upstream run unchanged.
  `LANE_CHANGE_LEFT` and `LANE_CHANGE_RIGHT` both map to `"lane-change"` — the AD API carries
  direction in `SteeringFactor::direction`, not in the behavior name.
* **Four names have no AD API equivalent** and are rejected with an explanation rather than a
  lookup failure: `BLIND_SPOT`, `INTERSECTION_OCCLUSION` (folded into `"intersection"`),
  `OCCLUSION_SPOT`, `NONE`.
* **Auto mode becomes a policy.** `requestAutoModeForCooperation(module, true)` sets
  `CooperationPolicy::OPTIONAL`; `false` sets `REQUIRED`.
* **Duplicate suppression keys on the uuid alone.** Upstream keyed on
  `(module, uuid, command_status)`; the AD API has no `command_status` field. The uuid is what
  actually identifies a request, so nothing is lost.

**None of this is exercised by `autoware_core`**, which launches no module that requests
cooperation: its `behavior_velocity_planner` carries only `StopLineModulePlugin` and its
`motion_velocity_planner` only `ObstacleStopModule`. It is kept — rather than deleted along with
the messages — because deleting it would make this fork Universe-incompatible, which is the
opposite of the goal.

## 6. Emergency → MRM state

`tier4_external_api_msgs/Emergency` on `/api/external/get/emergency` had a single `bool emergency`
field, and concealer aborted the scenario when it went true. It was set exactly when a
minimum-risk manoeuvre was running — information concealer *already* subscribes to, on
`/api/fail_safe/mrm_state`.

The separate subscriber is gone and the abort now fires from the `MrmState` callback when the
state is `MRM_OPERATING`, `MRM_SUCCEEDED` or `MRM_FAILED`. `MRM_SUCCEEDED` is deliberately
included: a completed emergency stop still means the scenario's premise no longer holds.

## 7. `autoware_scenario_simulation_msgs`: the four types with no AWF home

`UserDefinedValue`, `UserDefinedValueType`, `SimulationEvents` and `FaultInjectionEvent` have no
equivalent anywhere in `autoware_msgs`, `autoware_adapi_msgs` or `autoware_internal_msgs`. They
are simulation-harness concepts, and the harness is the right place to own them, so they are
defined in `common/autoware_scenario_simulation_msgs` on the fork branch, with **byte-identical
field layouts** to the messages they replace.

**Not `autoware_simulation_msgs`.** That package name is already taken: `autoware_msgs` has
carried an `autoware_simulation_msgs` since April 2026, when `SimulatedObject` and
`SimulatedObjectInitialState` were ported there out of `tier4_autoware_msgs`. Since
`dependency.repos` fetches `autoware_msgs`, a second package under that name puts two packages
with one name in the same workspace, and `colcon build` refuses to start. Worse quietly:
`user_defined_value_condition.cpp` binds the metric topics behind
`__has_include(<…/msg/user_defined_value.hpp>)`, so against upstream's package that guard is
simply false and **every `UserDefinedValueCondition` gate compiles out** — the scenario still
runs, and gates nothing. The four types keep their field layouts; only the package name moved.

That identity is deliberate: it makes the change a rename at the type level and nothing else, so
no scenario, and no consumer of `/metrics/*`, needed editing.

`msgs/autoware_component_evaluation_msgs`, in this repository, is new rather than a replacement — see
[COMPONENT_EVALUATION.md](COMPONENT_EVALUATION.md).

---

## 8. `architecture_type`: `awf/core` as a first-class value

Removing the messages is necessary but not sufficient for AWF usability. Upstream recognises only
`awf/universe*`, and spells that test as a bare `find("awf/universe") != npos` at eight separate
sites. Running against `autoware_core` therefore required *lying* about the architecture —
passing `awf/universe/20240605` and overriding `autoware_launch_package` underneath, which every
gate accepted because each one is a substring or lexicographic test that string satisfies.

This fork adds `common/architecture_type`, one header that is the single definition of what each
value means, and makes every gate ask it:

| Value | Meaning |
|---|---|
| `awf/core/1.0.0` | `autoware_core` — **the default** |
| `awf/universe/20250130` | Autoware Universe, the oldest generation this fork still speaks |

Universe releases before `20250130` are rejected rather than silently mishandled: they published
`PathWithLaneId` and the engage/RTC control plane through the TIER IV messages this fork no longer
builds against.

Sites converted:

| Site | Was | Now |
|---|---|---|
| `traffic_simulator/src/entity/ego_entity.cpp` | one branch, Universe's argument set | two branches, per-architecture argument sets |
| `simple_sensor_simulator/…/sensor_simulation.hpp` (×3) | `find("awf/universe")` | `architecture_type::isSupported` |
| `simple_sensor_simulator/…/traffic_lights_detector.hpp` | Universe topics only | core publishes to `/perception/traffic_light_recognition/traffic_signals`, since core has no arbiter to merge an "internal" stream into |
| `traffic_simulator/…/traffic_lights.hpp` | Universe types only | `TrafficLightGroupArray` for core |
| `scenario_test_runner.launch.py`, `mock_test.launch.py` | three hard-coded Universe entries | one `ARCHITECTURES` table |
| `random_test_runner/data_types.{hpp,cpp}` | `AWF_AUTO`, `AWF_UNIVERSE`, `TIER4_PROPOSAL` | `AWF_CORE`, `AWF_UNIVERSE` |
| `traffic_simulator/api.{hpp,cpp}`, `simulator_core.hpp`, `cpp_scenario_node.cpp` | `"awf/universe/20240605"` literals (×8) | `architecture_type::default_architecture_type` |

One behaviour change is worth calling out: `architectureTypeFromString` now matches on a **prefix**
rather than with `find`, so `" awf/universe"` is rejected instead of silently classified.
`test_runner/random_test_runner/test/test_data_types.cpp` pins this.

`concealer::AutowareUniverse` is renamed to `concealer::AutowareVehicleInterface`, because every
type it touches (`autoware_vehicle_msgs`, `autoware_control_msgs`) is identical on both stacks —
the name claimed a Universe coupling that was never there.

---

## 9. What was **not** migrated, and why

| Kept as-is | Reason |
|---|---|
| `traffic_simulator_msgs` | The simulator's own entity model. Not an Autoware interface, and nothing in Autoware consumes it. |
| `openscenario_interpreter_msgs`, `openscenario_preprocessor_msgs` | Interpreter-internal. Same reason. |
| `simulation_api_schema` (protobuf) | The simulator↔sensor-simulator ABI, not a ROS interface. |
| `__tier4_modifier_*` parameter names in two sample scenarios | Renamed to `__modifier_*`. Cosmetic; no code reads the prefix. |
