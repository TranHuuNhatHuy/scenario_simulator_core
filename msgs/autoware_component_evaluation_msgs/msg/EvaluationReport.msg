# Whole-run evaluation result: every component, every interface, every metric.
#
# Published periodically on /simulation/evaluation/report and written to JSON at shutdown, from
# which tools/report.py produces the JUnit XML that CI consumes.

builtin_interfaces/Time stamp

string scenario
string profile                   # evaluation profile in force, e.g. "P0", "P2"
float64 duration_s

autoware_component_evaluation_msgs/Verdict verdict
autoware_component_evaluation_msgs/ComponentReport[] components
string[] violations
