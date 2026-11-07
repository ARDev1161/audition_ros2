# M2v — native classifier and ASR workload snapshots

The namespaced `/audition/semantic_perception/get_semantic_status` service
now exposes actual libaudition worker status instead of estimating jobs
from admitted PCM. Disabled backends report zero/false.

## YAMNet

`classifier_active_sources` counts live ROS YAMNet source workers.
`waiting`, `collecting`, `queued`, `inferencing`, `classified`, and
`error` are mutually exclusive **source states** at sampling time, not
the number of individual jobs or windows in a queue.
`classifier_dropped_windows_active` sums native dropped/replaced windows
for currently active sources only. It may decrease on retirement/reload.

`classifier_max_last_inference_ms` and
`classifier_max_last_queue_wait_ms` are maxima of each live source's
**last successful** native measurements. Their `classifier_has_*_timing`
flags distinguish missing data from a measured zero. They are not the
elapsed time of an in-flight job. Native per-source reads are sequential,
so this is a sampled, non-atomic aggregate.

## Sherpa/Whisper

`asr_queued_segments` is the native pending queue length, **excluding
the currently executing ASR job**. `asr_buffered_results` counts
completed results awaiting ROS drain, and `asr_retired_generations`
counts retired generations awaiting release.
`asr_completed_segments_native`, `asr_dropped_segments_native` and
`asr_evicted_results_native` are native **worker-lifetime** counters,
reset when ReloadSemanticWorkers replaces the worker. M2u ROS-side
completion counters instead persist across reloads.
`asr_worker_error` indicates a native error without exposing its text.

These are read-only bounded snapshots, not a watchdog or proof of model
accuracy. A single hung ASR job can coexist with an empty pending queue.
No result is required during silence. Per-job deadline enforcement
requires native enqueue/start timestamps and explicit VAD semantics.

```bash
ros2 service call /audition/semantic_perception/get_semantic_status \\
  audition_msgs/srv/GetSemanticStatus "{}"
```

Tests cover deterministic C++ aggregation, ROS serialization, disabled
backend graph smoke, and compilation against pinned native YAMNet and
Sherpa/Whisper integrations. Real model latency, fault-injection HIL,
ReSpeaker calibration and two-phase ReloadPipeline remain unverified.
