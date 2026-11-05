# M3 timestamped TF2 attention bridge

The opt-in `attention_tf_bridge` subscribes to `attention/target` and
publishes `attention/target_base` (both `audition_msgs/AttentionTarget`).
It rotates the **unit direction only** from the sensor frame to
`target_frame` (default `base_link`) at the observation timestamp.

- Rejects invalid, stale, future-dated or unstamped observations.
- Rejects non-unit or non-finite direction vectors and malformed TF rotations.
- Missing timestamped TF publishes `valid=false` with `reason=tf_unavailable`.
- Invalid upstream attention also publishes an invalid target; it is never
  silently interpreted as a motion request.
- No translation is applied, no distance is estimated, and **no robot motion
  commands are issued**.

Run the existing M3 attention manager separately and launch
`m3_attention_tf.launch.py` only when the robot has a valid TF tree.
`maximum_age_ms` defaults to 500 and must be positive. This is a
transport/coordinate adapter, not an acoustic localization algorithm.
