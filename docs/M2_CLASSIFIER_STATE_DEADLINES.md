# M2w — YAMNet sampled state-residency diagnostics

The semantic status service reports four additional fields:

- `classifier_overdue_queued_sources`: current sources continuously observed in Queued for more than 10 seconds.
- `classifier_overdue_inferencing_sources`: current sources continuously observed in Inferencing for more than 30 seconds.
- `classifier_max_queued_state_age_ms` and `classifier_max_inferencing_state_age_ms`: longest sampled state residency.

The observation clock is monotonic and begins at the first service poll seeing
a state, not the native job enqueue time. Call the status service periodically
to observe transitions. Repeated reads of the same state do not reset its age.
When a source retires or a stream resets, the next workload sample prunes it.
Reloading workers creates a fresh tracker. Waiting, collecting and classified
states do not produce deadline alerts. Disabled YAMNet returns zero.

These fields are **diagnostics, not proof of a hung job**. One source can
process multiple jobs while remaining in the same state, and polling can miss
fast transitions. No automatic supervisor restart is enabled. The native ASR
API currently exposes pending queue size but not the start time of its running
job; claiming an ASR deadline would be misleading. Future work should add
per-job IDs and start/end timestamps in libaudition before enforcement.

Tests cover state transitions, stale repeated observations, silence, retirement,
reset and monotonic-clock behavior. Humble and native workflows validate
compilation and model-independent behavior; real-model timing needs HIL tests.
