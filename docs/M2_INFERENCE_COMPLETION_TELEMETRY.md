# M2u — semantic inference completion diagnostics

Use the namespaced read-only service:

```bash
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
```

`classifier_active` and `speech_active` indicate configured native workers.
`classifier_completed_windows` counts valid, current-generation YAMNet
windows processed and published. `classifier_discarded_results` counts stale
or duplicate completed windows ignored. `asr_completed_segments` counts
native ASR results drained for current metadata **including empty text**;
`asr_empty_segments` is the empty subset, and `asr_discarded_results`
counts results whose source metadata/generation was retired.
`inference_poll_failures` counts caught polling exceptions.

`classifier_last_completion_age_ms` and `asr_last_completion_age_ms`
are monotonic wall-clock ages since the last accepted completed result.
**Zero means no completion has yet been seen, not zero inference latency.**
Counters are process-lifetime, including same-process worker reload.
A stale result cannot refresh the completion age.

The absence of a completion is not a fault on its own: YAMNet needs enough
PCM for a window; VAD/ASR requires speech and may return empty text. This
does not measure queue depth, per-job inference duration, quality or accuracy.
The M2t ingress watchdog remains independent; it cannot be used to infer
native model throughput. Automatic inference watchdog containment requires
a future explicit per-job scheduled/deadline contract to avoid silence
false positives.

Tests: C++ deterministic completion/age tests, ROS message serialization,
disabled-worker Humble graph smoke, native YAMNet and Sherpa/Whisper CI builds.
Physical ReSpeaker calibration and real model latency remain unverified.
