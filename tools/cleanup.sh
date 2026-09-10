#!/usr/bin/env bash
# Kill anything left over from a previous run.
#
# Deliberately matches on the launch-file name and our own node names only. A broad
# `pkill -f autoware_` is dangerous: compiler command lines contain include paths with
# "autoware_" in them, so it will happily kill a running colcon build.
set -eo pipefail

PATTERNS=(
  'core_scenario_simulator.launch.xml'
  'scenario_test_runner.launch.py'
  'adapi_compat_node'
  'evaluator_node'
  'contract_checker_node'
  'handdrive.py'
  'openscenario_interpreter_node'
  'simple_sensor_simulator_node'
  'scenario_test_runner.py'
  'openscenario_preprocessor_node'
  'visualization_node'
)

for p in "${PATTERNS[@]}"; do
  pkill -f -- "$p" 2>/dev/null || true
done

sleep 2

# Second pass for autoware nodes started by our launch file: match the exact executable
# basenames, never a bare substring.
for exe in autoware_lanelet2_map_loader autoware_map_projection_loader_node \
           autoware_lanelet2_map_visualizer autoware_pointcloud_map_loader \
           autoware_pose_initializer_node autoware_behavior_velocity_planner_node \
           autoware_motion_velocity_planner_node velocity_smoother_node \
           path_generator_node mission_planner autoware_simple_pure_pursuit_exe \
           autoware_command_gate_exe path_to_trajectory_converter \
           vehicle_velocity_converter robot_state_publisher \
           component_container component_container_mt \
           routing_adaptor_node initial_pose_adaptor_node ground_filter_node; do
  pkill -f -- "/lib/[^ ]*/${exe}" 2>/dev/null || true
done

sleep 1
# exclude colcon: its command line contains "--packages-up-to scenario_test_runner"
left=$(pgrep -af 'core_scenario_simulator|scenario_test_runner' 2>/dev/null | grep -vc colcon || true)
echo "cleanup done (${left:-0} stray processes remain)"
