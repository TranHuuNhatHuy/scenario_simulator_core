# The `scenario_simulator_v2` fork branch

This project needs three changes to `scenario_simulator_v2` that have not landed upstream. They are
**not copied into this repository**. They live as three commits on a branch of a fork:

```
https://github.com/TranHuuNhatHuy/scenario_simulator_v2   branch: feat/awf-core
  = tier4/scenario_simulator_v2 master + 3 commits, and nothing else
```

`dependency.repos` pins that branch. `vcs import` fetches it like any other dependency.

## Why a branch and not a copy

A vendored copy of upstream answers none of the questions a reviewer will ask. A branch answers
all of them with `git`:

| Question | How the branch answers it |
|---|---|
| Did you copy TIER IV's code? | `git log` shows their history, then three commits authored here. |
| What exactly did you change? | `git diff upstream/master` — ~15 files. |
| Is that reviewable? | Each commit is one topic with a message explaining the reasoning, so each is a pull request as it stands. |
| How do you stay current? | `git rebase upstream/master`. Three commits, not 1279 files. |
| When does the fork end? | When the last commit lands upstream, `dependency.repos` points back at `tier4/scenario_simulator_v2` and the branch is deleted. |

## The three commits

These three commits are entirely behavior-preserving and introduce no AWF messaging coupling, making them highly acceptable to upstream.

| # | Commit | Files | Independent value to upstream | Needs agreement on |
|---|---|---:|---|---|
| 1 | `refactor(concealer): rename AutowareUniverse to AutowareVehicleInterface` | 6 | Every type the class touches is identical on both stacks; the name claims a coupling the code does not have. | nothing |
| 2 | `feat(common): add architecture_type, one definition of the architecture gate` | 14 | Eight scattered `find("awf/universe")` tests and eight repeated default literals become one predicate with one error message that lists what is accepted. Behaviour-preserving. | the prefix-vs-substring tightening |
| 3 | `feat: support awf/core as an architecture_type` | 12 | Makes `autoware_core` a target instead of something reached by passing a Universe architecture and overriding the launch package underneath. | the name `autoware_core_scenario_launch`; dropping Universe < 20250130 |

## Message Bridging (The Adapter Node Paradigm)

Unlike earlier plans, **we do not replace `tier4_*` messages inside the simulator.** `scenario_simulator_v2` continues to use its native TIER IV interfaces.
Instead, we provide an **Adapter Node** inside `scenario_simulator_core` that translates AWF messages to/from TIER IV messages at runtime. 
This limits the upstream PR footprint drastically and cleanly decouples the simulator's internal architecture from AWF's message definitions.

## Working on the branch

```bash
git clone https://github.com/TranHuuNhatHuy/scenario_simulator_v2.git
cd scenario_simulator_v2
git remote add upstream https://github.com/tier4/scenario_simulator_v2.git
git checkout feat/awf-core

git log --oneline upstream/master..HEAD    # the three commits
git diff --stat upstream/master..HEAD      # the whole delta

git fetch upstream && git rebase upstream/master   # pick up a new release
```

**Amend in place; do not stack fixups.** The branch is a proposal, and its value is that it reads
as three clean changes.
