# M3q — opt-in strict scene timestamp replay fencing

`attention_manager` now accepts startup-only
`require_advancing_scene_stamp` (default `false`). When enabled, every
accepted scene in the same generation and microphone frame must have a strictly
greater ROS header stamp. Equal timestamps (including exact replay), older
timestamps, zero timestamps and invalid nanosecond fields are rejected.
Rejected snapshots neither advance challenger confirmation nor refresh the
steady-clock scene timeout. The next strictly newer scene can recover; a
new generation or frame begins a separate ordering epoch.

Use this only if the scene producer guarantees nonzero, monotonically
advancing timestamps. Some replay and simulated publishers intentionally
reuse timestamps; default behavior preserves their compatibility.

Example:

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -p require_advancing_scene_stamp:=true \
  -p switch_confirmations:=3
```

Four pure-policy regression tests cover duplicate replay, timeout fencing,
challenger confirmation, malformed stamps, epoch changes and default behavior.
This is a bearing-only advisory; no robot motion is authorized.
