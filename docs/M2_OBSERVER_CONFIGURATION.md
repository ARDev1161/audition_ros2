# M2z — YAMNet observer configuration

The independent workload timer and source-state age thresholds are configurable
using startup-only ROS parameters:

| Parameter | Default | Allowed range (milliseconds) |
| --- | ---: | ---: |
| `classifier_workload_sample_period_ms` | 1000 | 200–10000 |
| `classifier_queued_threshold_ms` | 10000 | 1000–600000 |
| `classifier_inference_threshold_ms` | 30000 | 1000–600000 |

Both thresholds must be greater than or equal to the sample period. Invalid
values reject node construction; they cannot be changed through ROS parameter
updates while running. A validated profile restart is required.

`GetSemanticStatus` returns all three effective settings, alongside
`classifier_workload_sample_valid`, age, counters and overdue source counts.
This makes the threshold interpretation reproducible for monitoring clients.
Defaults preserve the prior M2x/M2y behavior. This is a diagnostic
state-residency threshold, not a job deadline or automatic restart policy.

The five unit tests cover defaults, valid bounds, invalid sample periods,
invalid thresholds and threshold/sample-period consistency. CI exercises both
default-disabled ROS 2 builds and native YAMNet integration.
