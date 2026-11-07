# M3h — advisory sound-attention selection

The new `audition_behaviors` package provides `attention_manager` (a standalone
ROS 2 executable, **opt-in**). It consumes the canonical relative `scene`
topic and publishes `attention/target` (`audition_msgs/AttentionTarget`).

This is **not** `LookAtSound`, `InvestigateSound`, a Nav2 goal, a TF
transform, a metric position, or a motion command. The output is only a
bearing in the source microphone frame and must not directly actuate hardware.
A future authorized robot-specific controller must validate TF, kinematic
limits, collision safety, user consent, cancellation and stale observations.

## Selection contract

- Only ACTIVE sources with matching scene/source generation, matching frame,
  finite [0,1] activity score, and a nearly unit-length finite direction
  are eligible. No fabricated direction from mono audio.
- Highest activity wins; retain an existing candidate until another exceeds
  it by `switch_margin`. Lexicographic source ID breaks exact ties.
- Missing candidates immediately publish `valid=false`. A stream generation
  or frame change invalidates the previous selection.
- Without a new scene, `scene_timeout_ms` expires the candidate and publishes
  a single invalidation. Steady clock is used for local freshness; ROS header
  timestamps are copied from the observed scene and are not a remote clock
  synchronization guarantee.
- The score is relative model-native activity, **not** a probability of a
  human, a hazard, a physical object, or a calibrated confidence.

## Example

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -r __ns:=/robot_a/audition \
  -p minimum_activity:=0.3 -p switch_margin:=0.15 \
  -p scene_timeout_ms:=1500
ros2 topic echo /robot_a/audition/attention/target
```

Startup-only parameters are validated; no dynamic parameter update callback.
This standalone executable is intentionally not enabled by default in existing
WAV/ALSA launch presets, preserving M0–M2 behavior and native CI coverage.

## Tests and limitations

Pure-policy GTests cover selection, hysteresis, source-generation/frame
fencing, stale expiry, invalid directions, ended sources and invalid config.
The generated ROS message has a serialization round-trip test. Hardware
orientation, TF2 and Nav2 integration are deliberately deferred until
motion safety contracts and a robot-specific actuator are available.
