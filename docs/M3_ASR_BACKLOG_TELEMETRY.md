# M3a — ASR workload sampling and backlog diagnostics

The native speech runtime exposes pending ASR segments, buffered result
mailbox occupancy, retired generations, completed/dropped/evicted counters
and the last worker error flag. These are now read by the semantic node's
periodic workload timer rather than on every `GetSemanticStatus` call.

Seven new fields report sample presence, validity, successes, failures,
last-success age, maximum observed pending segments and maximum observed
buffered results. Current native statistics are zeroed on a sampling
exception; peaks and historical counters remain available for diagnosis.
Worker replacement and stream generation changes reset the observer to
avoid attributing an old backlog to the new worker incarnation.

YAMNet and ASR polling use separate exception handlers: a native status
failure in one adapter cannot prevent polling the other adapter. The timer
interval is the existing startup-only
`classifier_workload_sample_period_ms` (200–10000 ms).

No automatic restart or ASR deadline is implemented. Pending segments
exclude the running inference job; silent audio may legitimately generate
no ASR jobs. Peaks are sampled observations, not guaranteed exact maxima.
Unit tests cover initialization, independent sampling, peak retention,
failure invalidation, reset fencing and monotonic age semantics.
