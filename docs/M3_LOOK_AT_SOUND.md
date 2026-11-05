# M3 LookAtSound advisory acquisition action

The opt-in `look_at_sound` action server listens to
`attention/target_base` (the TF2 bridge output) and returns the first
**fresh, valid, unit-length bearing** in the configured target frame.
This is **not a robot motion action**: it never publishes velocity,
navigation, or joint commands, and does not claim that a robot turned.

Launch `m3_attention_tf.launch.py` and
`m3_look_at_sound.launch.py` with the same namespace and target frame.
Call `/audition/look_at_sound` with an optional `source_id` filter
and `timeout_ms` (1–10000 by default). Invalid timeouts and concurrent
goals are rejected. Accepted goals support cancellation, feedback while
waiting, timeout abortion and successful acquisition. A target must have
arrived within `maximum_target_age_ms` (default 500), have `valid=true`,
the correct frame and a finite unit vector. Only one active goal is allowed.
The returned `AttentionTarget` retains source and scene provenance.

**Limitations:** the freshness gate is based on arrival monotonic time
and relies on the upstream TF2 bridge to validate observation stamps and
replay. It does not verify actual actuator rotation or robot orientation.
Downstream motion controllers must independently validate freshness,
frames, safety limits and collision clearance; they must not interpret
this action's success as physical completion.

## ROS graph regression

The Humble CI workflow launches the actual Python action server in a
separate process and DDS domain (115), then runs
`audition_tests/smoke_look_at_sound_graph.py`. It checks source-filtered
acquisition, rejection of concurrent and invalid goals, timeout abortion,
cancellation and preservation of source/scene provenance. The probe injects
synthetic transformed targets; it does not measure acoustic localization or
exercise robot actuators.
