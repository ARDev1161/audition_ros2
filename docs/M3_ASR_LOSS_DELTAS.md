# M3e — native ASR loss-counter delta diagnostics

Native libaudition ASR reports cumulative dropped-segment and evicted-result
counters. These may be nonzero before a ROS status consumer starts observing.

The independent workload sampler now computes unsigned positive differences
between consecutive successful native snapshots. A first sample is a baseline.
After a polling failure, the next success is a new baseline, avoiding a false
loss spike from an unknown interval. A native counter decrease likewise
produces zero increment rather than an unsigned wrap. Observed totals use
saturating addition and reset on stream generation changes or worker reload.

GetSemanticStatus fields:
- `asr_last_dropped_delta` and `asr_last_evicted_delta`
- `asr_observed_dropped_delta_total` and `asr_observed_evicted_delta_total`
- `asr_loss_on_last_sample`

The last-sample flag describes observed changes in the most recent successful
poll, not an enduring failure state. Raw native cumulative counters remain
available separately. Neither loss counters nor backlog occupancy establish
ASR accuracy or a per-job deadline. No automatic restart is performed.

Four model-independent tests cover initial baselines, positive deltas,
counter regression, polling failures and worker reset.
