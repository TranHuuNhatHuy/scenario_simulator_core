#!/usr/bin/env bash
# Source before building or running:  source setup_env.sh
#
# NOTE: never `set -u` here. ROS's own setup.bash reads AMENT_TRACE_SETUP_FILES unguarded and
# dies with "unbound variable".
#
# NOTE: fail-fast ONLY when this file is executed, never when it is sourced -- which is how it is
# meant to be used. `set -e` and `set -o pipefail` persist into the calling shell, so afterwards
# the first command that returns non-zero closes the terminal: a failing `colcon build`,
# `ros2 pkg prefix` on a package that is not installed, even a `grep` that matches nothing.
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
  set -eo pipefail
fi

export SSC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SSC_WS="${SSC_WS:-$SSC_ROOT/ws}"

# This repository is an overlay: the simulator and Autoware are fetched into $SSC_WS by
# ./bootstrap.sh, not vendored here. Run that first if $SSC_WS/src is empty.
#
# Underlay: any workspace already supplying the Autoware Foundation message packages
# (autoware_adapi_v1_msgs, autoware_internal_planning_msgs, autoware_common_msgs, ...).
# There is deliberately no tier4_* dependency; see docs/AWF_INTERFACE_MIGRATION.md.
: "${AUTOWARE_UNDERLAY:=$HOME/Documents/Autoware/autoware/install}"
if [ -f "$AUTOWARE_UNDERLAY/setup.bash" ]; then
  source "$AUTOWARE_UNDERLAY/setup.bash"
fi

# Overlay: this repo. Skip on the very first build.
if [ -f "$SSC_WS/install/setup.bash" ]; then
  source "$SSC_WS/install/setup.bash"
fi

# rmw_cyclonedds_cpp fails with "rmw_create_node: failed to create domain" at roughly 20
# participants, and a full core + simulator run is well past that.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Do NOT set ROS_LOCALHOST_ONLY=1: it interacts badly with the forked Autoware launch process.

# RAM, not CPU, is the build constraint here.
export MAKEFLAGS="${MAKEFLAGS:--j2}"

echo "scenario_simulator_core"
echo "  repo     : $SSC_ROOT"
echo "  workspace: $SSC_WS"
echo "  underlay : $AUTOWARE_UNDERLAY"
echo "  rmw      : $RMW_IMPLEMENTATION"
