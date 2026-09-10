#!/usr/bin/env bash
# Run one scenario against autoware_core and print the per-component evaluation.
#
#   ./run_scenario.sh                              # core_all_components.yaml
#   SCENARIO=core_smoke ./run_scenario.sh          # the minimal loop, no gates
#   PROFILE=P2 ./run_scenario.sh                   # core's own localization in the loop
#   LAUNCH_RVIZ=true ./run_scenario.sh
#
# Any further arguments are passed straight through to `ros2 launch`, so a one-off override
# needs no edit here:
#
#   ./run_scenario.sh autoware.shim_standalone_mode:=false
set -eo pipefail

SSC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SSC_ROOT/setup_env.sh"

SCENARIO="${SCENARIO:-core_all_components}"
PROFILE="${PROFILE:-P0}"
# These must be non-empty. traffic_simulator's EgoEntity forwards both to `ros2 launch`
# unconditionally, and `ros2 launch` rejects an empty argument value outright --
#   malformed launch argument 'sensor_model:=', expected format '<name>:=<value>'
# -- so Autoware exits 1 during bring-up and the scenario fails 180 s later with the
# unrelated-looking "Autoware process is unintentionally exited". Leaving these unset does not
# fall back to core_scenario_simulator.launch.xml's defaults; it never gets that far.
VEHICLE_MODEL="${VEHICLE_MODEL:-autoware_sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-autoware_sample_sensor_kit}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"
REPORT="${REPORT:-/tmp/core_component_report.json}"
OUTPUT_DIRECTORY="${OUTPUT_DIRECTORY:-/tmp/ssc_out}"

# initialize_duration is raised well above concealer's 30 s default: autoware_core starts more
# slowly than Universe's planning simulator, and the default budget is tight enough that a cold
# machine fails here for no reason other than bring-up time.
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/core/1.0.0 \
  scenario:="$SSC_ROOT/scenarios/${SCENARIO}.yaml" \
  output_directory:="$OUTPUT_DIRECTORY" \
  initialize_duration:=120 \
  vehicle_model:="$VEHICLE_MODEL" \
  sensor_model:="$SENSOR_MODEL" \
  global_timeout:=300 \
  launch_rviz:="$LAUNCH_RVIZ" \
  "autoware.profile:=$PROFILE" \
  "autoware.scenario_name:=$SCENARIO" \
  "autoware.evaluation_report_path:=$REPORT" \
  "$@"

echo
ros2 run autoware_core_component_evaluator evaluation_report --report "$REPORT" || true
