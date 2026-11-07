# M2y — native source incarnation and workload sample fencing

The YAMNet status observer now uses the native per-worker source ID as the
state-residency key. Reusing the same external source name with a new native
source ID starts a fresh queue/inference age.

A status polling exception invalidates the cached workload and clears the
residency tracker. Retirement and stream reset also invalidate the cache.
The existing historical sample counter is preserved; a new
`classifier_workload_sample_valid` boolean identifies whether cached workload
is current. An invalid sample reports no overdue states until the next
successful observation. This avoids interpreting old telemetry as a live stall.

The regression tests exercise reused external names (distinct native IDs) and
resets following failed samples. No automatic worker restart is enabled.
These are observed *source-state* durations, not native per-job deadlines.
