# Conformance and performance of one Autoware component over one scenario run.
#
# `metrics` carries the component's derived quantities (accuracy, comfort, behaviour) as AWF-native
# metrics; `interfaces` carries the per-interface I/O conformance. A component fails if any of its
# required interfaces fails or any of its gated metrics is out of bounds.

string domain
autoware_component_evaluation_msgs/Verdict verdict

autoware_component_evaluation_msgs/InterfaceObservation[] interfaces
autoware_internal_metric_msgs/Metric[] metrics

string[] violations
