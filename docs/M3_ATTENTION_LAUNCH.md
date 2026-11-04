# M3n — opt-in attention launch integration

The M2 profile launch now supports an **explicitly enabled** advisory attention
consumer. The existing default remains off. It subscribes to the canonical
relative `scene` topic in the selected namespace and publishes
`attention/target` in the same namespace.

```bash
ros2 launch audition_bringup m2_profile.launch.py \
  profile:=replay_ci wav_path:=/absolute/path/mono.wav \
  enable_attention:=true
```

For a calibrated spatial source, pass the corresponding ODAS profile and
unique microphone frame. Mono profiles generally have no usable bearing;
an invalid advisory target is expected, not an invented direction.

For advanced policy configuration, supply an absolute YAML path:

```yaml
/**/attention_manager:
  ros__parameters:
    minimum_activity: 0.3
    switch_margin: 0.15
    switch_confirmations: 3
    switch_confirmation_window_ms: 750
    scene_timeout_ms: 1500
```

Then add `attention_parameters:=/absolute/path/attention.yaml`. The
fail-closed preflight rejects unknown parameters, malformed values, ambiguous
node scopes, invalid files, and parameters supplied when attention is disabled.
The standalone `m3_attention.launch.py` can also be launched in the same
namespace as an existing pipeline. Namespace validation prevents accidental
global subscription. The consumer remains bearing-only: **no TF-based
position, Nav2 goal, motion command, or safety authorization**.

Five Python preflight regression tests run under ament/ROS2 CI. Physical
direction accuracy and actual robot behavior remain unvalidated.
