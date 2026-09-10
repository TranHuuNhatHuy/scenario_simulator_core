# What was actually observed on one declared component interface, and how that compares with the
# expectation declared for it.
#
# The structural half of the expectation (name, type, kind, QoS) comes from
# `autoware_component_interface_specs/interface_manifest.json`, which autoware_core generates and
# version-controls. The performance half (rates, gaps, jitter, latency) is declared by this repo,
# because the manifest does not carry it.

# Identity, as declared.
string domain                    # map | sensing | localization | perception | planning | control | vehicle | system | api
string interface                 # topic or service name
string type                      # e.g. "autoware_planning_msgs/msg/Trajectory"
string kind                      # "topic" | "service"

# Graph facts.
bool observed
uint32 publisher_count
uint32 subscriber_count
bool type_matches
bool qos_matches

# Timing, measured over the evaluation window.
uint64 message_count
float64 rate_hz                  # message_count / observation window
float64 max_gap_s                # worst inter-arrival gap; catches stalls a mean rate hides
float64 jitter_s                 # standard deviation of inter-arrival time
float64 latency_mean_ms          # header.stamp -> receipt; NaN when the type carries no stamp
float64 latency_max_ms

# The expectation this was judged against, echoed so a report is self-contained.
float64 expected_rate_min_hz
float64 expected_rate_max_hz
float64 expected_max_gap_s
float64 expected_latency_max_ms

autoware_component_evaluation_msgs/Verdict verdict
string[] violations              # one line per failed check, empty when verdict is PASS
