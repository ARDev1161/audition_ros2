# M3c — ASR pressure freshness fencing

M3b's queue-residency clock was measured from the first nonempty observation.
Without a fresh-sample guard, a stalled sampling timer could produce a pressure
flag from one old observation after ten seconds. M3c requires **both**:
- At least two consecutive valid nonempty native ASR queue samples.
- A last successful sample no older than 3000 ms.

The existing thresholds (>=4 pending segments, >=10000 ms observed queue
residency) still apply. The status service exposes
`asr_workload_sample_fresh` and `asr_contiguous_queue_samples` so operators
can distinguish absent telemetry from sustained observed backlog.

Queue drain, sample exception, stream generation reset and worker reload
clear consecutive evidence. Freshness is monotonic and rejects backwards
time. Tests cover stopped timers, fresh repeated observations, drain,
failure, reset and clock regression.

This remains diagnostic-only. A pending segment is not a running inference
job and no restart or hard deadline is inferred.
