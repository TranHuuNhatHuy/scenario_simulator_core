#!/usr/bin/env bash
# Create the workspace this overlay builds in.
#
#   ./bootstrap.sh [workspace-dir]        default: ./ws
#
# Fetches scenario_simulator_v2 (the feat/awf-core branch, see docs/FORK.md), autoware_core and
# the Autoware Foundation message repositories, then symlinks this repository in beside them.
# Nothing upstream is vendored here, so this step is what turns a 57-file repo into a workspace.
set -eo pipefail

SSC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${1:-$SSC_ROOT/ws}"

command -v vcs >/dev/null || { echo "vcstool not installed: pip install --user vcstool"; exit 1; }

mkdir -p "$WS/src"
vcs import "$WS/src" < "$SSC_ROOT/dependency.repos"

# Link this repository's package trees in -- the three directories that hold packages, not the
# repository root. The default workspace lives inside the repository, so a root symlink would make
# $WS/src/scenario_simulator_core/ws/src/scenario_simulator_core/... an infinitely deep path.
# colcon stops itself on that; rosdep, ament and every other tree walker do not.
mkdir -p "$WS/src/scenario_simulator_core"
#
# -r makes them RELATIVE. An absolute symlink dangles the moment the tree is moved, copied to
# another machine or unpacked under a different user -- and a dangling src/ link presents as
# "Package not found" for everything, with nothing pointing at the cause.
for tree in evaluation msgs core_adapter; do
  ln -sfnr "$SSC_ROOT/$tree" "$WS/src/scenario_simulator_core/$tree"
done

echo
echo "workspace ready: $WS"
vcs status "$WS/src" --nested 2>/dev/null | grep -E "^===|^[A-Z]" | head -30
echo
echo "next:"
echo "  source setup_env.sh"
echo "  cd $WS && colcon build --symlink-install --packages-up-to autoware_core scenario_simulator_v2 autoware_core_component_evaluator autoware_core_scenario_launch openscenario_experimental_catalog autoware_sample_vehicle_description autoware_sample_sensor_kit_description"
