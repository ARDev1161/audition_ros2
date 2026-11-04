# M3l — opt-in source observation freshness gate

The opt-in attention manager can reject a source whose `last_seen` is too
old relative to the canonical `AcousticScene.header.stamp`.

Startup-only parameter: `maximum_source_age_ms` (default 0, disabled).
When enabled, eligible sources must have a valid nonnegative timestamp
`source.last_seen`, with `0 <= scene_stamp - last_seen <= threshold`.
Future source timestamps, malformed nanoseconds, negative seconds and missing
last_seen are rejected. Both stamps must originate from the same scene
producer/time domain. **Do not compare source stamps to local wall time**;
the independent steady-clock `scene_timeout_ms` remains responsible for
missing scene messages.

Example:

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -p maximum_source_age_ms:=250 \
  -p switch_confirmations:=2
```

The gate is disabled by default for producers that do not populate last_seen.
A LOST/ENDED source remains ineligible regardless of timestamp. This prevents
old per-source observations from being treated as current merely because a
scene is freshly published. It does not estimate range or authorize motion.

Four policy tests cover stale, future, malformed, missing and valid timestamps,
default compatibility, and invalid configuration.
