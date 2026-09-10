# The `scenario_simulator_v2` fork branch

This project needs six changes to `scenario_simulator_v2` that have not landed upstream. They are
**not copied into this repository**. They live as six commits on a branch of a fork:

```
https://github.com/TranHuuNhatHuy/scenario_simulator_v2   branch: feat/awf-core
  = tier4/scenario_simulator_v2 master + 6 commits, and nothing else
```

`dependency.repos` pins that branch. `vcs import` fetches it like any other dependency.

## Why a branch and not a copy

A vendored copy of upstream answers none of the questions a reviewer will ask. A branch answers
all of them with `git`:

| Question | How the branch answers it |
|---|---|
| Did you copy TIER IV's code? | `git log` shows their history, then six commits authored here. |
| What exactly did you change? | `git diff upstream/master` — 53 files, +1060 / −544. |
| Is that reviewable? | Each commit is one topic with a message explaining the reasoning, so each is a pull request as it stands. |
| How do you stay current? | `git rebase upstream/master`. Six commits, not 1279 files. |
| When does the fork end? | When the last commit lands upstream, `dependency.repos` points back at `tier4/scenario_simulator_v2` and the branch is deleted. |

The nightly `upstream-drift` workflow rebases the branch onto `tier4/master` and fails on
conflict. It pushes nothing; it exists so that a conflict is found on the day it appears, when it
costs ten minutes, rather than after ninety days of drift, when it costs a week.

## The six commits

Ordered so the least contentious land first. Commits 1–2 are defensible with no reference to
`autoware_core` at all — worth filing first, to establish the series before asking for the ones
that need discussion.

| # | Commit | Files | Independent value to upstream | Needs agreement on |
|---|---|---:|---|---|
| 1 | `refactor(concealer): rename AutowareUniverse to AutowareVehicleInterface` | 6 | Every type the class touches is identical on both stacks; the name claims a coupling the code does not have. | nothing |
| 2 | `feat(common): add architecture_type, one definition of the architecture gate` | 14 | Eight scattered `find("awf/universe")` tests and eight repeated default literals become one predicate with one error message that lists what is accepted. Behaviour-preserving. | the prefix-vs-substring tightening |
| 3 | `feat: support awf/core as an architecture_type` | 12 | Makes `autoware_core` a target instead of something reached by passing a Universe architecture and overriding the launch package underneath. | the name `autoware_core_scenario_launch`; dropping Universe < 20250130 |
| 4 | `feat: replace TIER IV message packages with AWF equivalents` | 19 | Removes `tier4_simulation_msgs` and `tier4_debug_msgs`. | **where `autoware_scenario_simulation_msgs` should live** — here, or in `autoware_msgs` |
| 5 | `feat(concealer): use autoware_internal_planning_msgs/PathWithLaneId only` | 3 | Deletes dual-type `__has_include` dispatch that no longer has a reachable caller. | nothing |
| 6 | `feat(concealer): drive the control plane through the AD API` | 5 | Removes `tier4_external_api_msgs` and `tier4_rtc_msgs`. After it, `grep -rE '<[a-z_]*depend[^>]*>\s*tier4_' --include=package.xml` returns nothing. | the RTC → cooperation remodelling; the lost velocity-limit acknowledgement |

Full reasoning for 4–6 is in [AWF_INTERFACE_MIGRATION.md](AWF_INTERFACE_MIGRATION.md).

### The two open questions worth raising before filing

#### 1. Where does `autoware_scenario_simulation_msgs` belong? 
`UserDefinedValue`, `UserDefinedValueType`, `SimulationEvents` and `FaultInjectionEvent` have no equivalent in
`autoware_msgs`, `autoware_adapi_msgs` or `autoware_internal_msgs`. Commit 4 defines them inside
`scenario_simulator_v2/common/`, which makes the pull request self-contained. If the Autoware
Foundation would rather host them, moving the package is a one-line change to every include —
better decided in review than after.

The name is not free, and this is the argument for upstream hosting rather than against it:
`autoware_msgs` has shipped its own `autoware_simulation_msgs` since April 2026
(`SimulatedObject`, ported from `tier4_autoware_msgs`). These four types are therefore
`autoware_scenario_simulation_msgs` here. If AWF hosts them, they most naturally become four more
messages *in* `autoware_simulation_msgs` and this package disappears rather than moves.

#### 2. The AD API cannot express a velocity limit. 
Commit 6 replaces `tier4_external_api_msgs/SetVelocityLimit` with a topic publication, because the AD API has no
endpoint for it. That is a gap affecting every AD API client — simulators, bench rigs, remote
operation — not just this project, and belongs in front of the Architecture WG independently of
these commits.

## Working on the branch

```bash
git clone https://github.com/TranHuuNhatHuy/scenario_simulator_v2.git
cd scenario_simulator_v2
git remote add upstream https://github.com/tier4/scenario_simulator_v2.git
git checkout feat/awf-core

git log --oneline upstream/master..HEAD    # the six commits
git diff --stat upstream/master..HEAD      # the whole delta

git fetch upstream && git rebase upstream/master   # pick up a new release
```

**Amend in place; do not stack fixups.** The branch is a proposal, and its value is that it reads
as six clean changes. A fix to commit 3 belongs in commit 3
(`git rebase -i upstream/master`), not in a seventh commit called "address review".

**When a commit lands upstream**, drop it from the branch on the next rebase — git usually does
this by itself — and update the table above. The number of rows is the metric: it should only
ever go down.

## Burn-down

The same burn-down applies to `core_adapter/autoware_core_adapi_compat` in this repository, which
retires when an `OperationModeNode` lands in `autoware_core`'s `default_adapi`.
