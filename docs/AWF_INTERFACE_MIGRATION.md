# TIER IV interfaces ↔ Autoware Foundation interfaces

**Goal: `scenario_simulator_core` must be usable by anyone in the Autoware Foundation community, while minimizing disruption to the upstream `scenario_simulator_v2` repository.**

Instead of stripping `tier4_*` messages out of `scenario_simulator_v2` (which would require massive upstream PRs), we treat the `tier4_*` messages as the simulator's public API. We provide an **Adapter Node** inside `scenario_simulator_core` that seamlessly translates between AWF interfaces (used by `autoware_core`) and TIER IV interfaces (expected by the simulator).

This document records how each interface is bridged by the adapter node, and — more importantly — where the replacement is not a simple rename but a change in behaviour.

---

## 1. Summary of Adapter Bridging

| TIER IV interface (Simulator Side) | Replaced by / Bridged to (AWF Side) | Kind of mapping |
|---|---|---|
| `tier4_planning_msgs/PathWithLaneId` | `autoware_internal_planning_msgs/PathWithLaneId` | **rename** — identical fields, translated on the fly |
| `tier4_simulation_msgs/UserDefinedValue` | Native (kept as-is for scenario gating) | **no change** — avoids governance debate over where metric types live |
| `tier4_external_api_msgs/Engage` (srv) | AD API `ChangeOperationMode` on `/api/operation_mode/change_to_autonomous` | **remodelled** — service intercepted and converted to AD API call |
| `tier4_external_api_msgs/SetVelocityLimit` (srv) | `autoware_internal_planning_msgs/VelocityLimit` published on `/planning/scenario_planning/max_velocity` | **remodelled** — service intercepted and published as topic |
| `tier4_external_api_msgs/ResponseStatus` | `autoware_common_msgs/ResponseStatus` | **remodelled** — translated between boolean `success` and integer `code` |
| `tier4_external_api_msgs/Emergency` | `autoware_adapi_v1_msgs/MrmState` on `/api/fail_safe/mrm_state` | **remodelled** — AD API state triggers the simulator emergency |
| `tier4_rtc_msgs/*` (RTC) | AD API cooperation: factors + `SetCooperationCommands` / `SetCooperationPolicies` | **remodelled** — AD API behavior strings translated to `tier4` enums |

Three of these carry real semantic weight. They are the rest of this document.

---

## 2. Engage: a service call becomes an operation mode change

The simulator calls `tier4_external_api_msgs/Engage` on `/api/external/set/engage`.
The AD API has no engage endpoint, because engaging *is* an operation mode change. 
The Adapter Node provides the `/api/external/set/engage` service. When the simulator calls it, the adapter forwards the request as an AD API `ChangeOperationMode` call on `/api/operation_mode/change_to_autonomous`.

## 3. Velocity limit: a service becomes a latched topic

The simulator calls `tier4_external_api_msgs/SetVelocityLimit` as a service. The AD API has **no velocity-limit endpoint of any kind**. 
The Adapter Node provides the service, and upon receiving a limit, translates and publishes it as a latched `autoware_internal_planning_msgs/VelocityLimit` topic on `/planning/scenario_planning/max_velocity`.

## 4. Response status: `code == SUCCESS` becomes `success`

`tier4_external_api_msgs/ResponseStatus` reports `uint32 code` with `SUCCESS = 1`.
AWF status types carry a plain `bool success`. The Adapter Node explicitly maps `success == true` to `code = 1` and vice versa when returning responses to the simulator.

## 5. RTC → AD API cooperation: the largest remodelling

The simulator expects `tier4_rtc_msgs` (modeling cooperation as `(Module enum, uuid) → Command{ACTIVATE, DEACTIVATE}`).
The Adapter Node translates this to AD API semantics:
* **Module names**: Maps `tier4_rtc_msgs::Module` enumerations to their corresponding AD API behavior strings (e.g., `LANE_CHANGE_LEFT` to `"lane-change"`).
* **Command Translation**: Intercepts `tier4_rtc_msgs` service calls and maps them to `SetCooperationCommands` and `SetCooperationPolicies`.

**None of this is currently exercised by `autoware_core`**, which launches no module that requests cooperation, but the translation is kept to ensure full architectural completeness.

---

## 6. `architecture_type`: `awf/core` as a first-class value

While we handle messages via the adapter, we still push 3 commits to `scenario_simulator_v2` to make it officially recognize `autoware_core`.
This introduces `common/architecture_type`, creating a single definition of what each value means, and making every gate ask it:

| Value | Meaning |
|---|---|
| `awf/core/1.0.0` | `autoware_core` — **the default** |
| `awf/universe/20250130` | Autoware Universe, the oldest generation this fork still speaks |

This replaces scattered substring tests (e.g., `find("awf/universe") != npos`) with a clean, type-safe architecture predicate.
