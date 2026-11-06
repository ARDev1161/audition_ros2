# M3 TrackSound — bounded advisory tracking

The optional `m3_track_sound.launch.py` starts a standalone `TrackSound` ROS 2
action server. It subscribes to namespaced `attention/target_base` from the
existing timestamped TF2 bridge and publishes **feedback only** for each
new valid observation during a bounded goal window. It never issues robot
motion, camera commands, Nav2 goals, or costmap changes.

```bash
ros2 launch audition_bringup m3_track_sound.launch.py audition_namespace:=robot_a/audition
ros2 action send_goal /robot_a/audition/track_sound audition_msgs/action/TrackSound \
  '{source_id: "", timeout_ms: 5000}' --feedback
```

A goal starts at the current ingress sequence; cached observations cannot
produce feedback. The server admits only unit-length, finite directions in
its configured frame with advancing scene-generation/stamp and non-regressing
source generation. Explicit valid=false advisories revoke the cache and
fence older replay. A source filter selects only that source; a blank filter
accepts any source. One goal is active per server. Cancellation stops feedback
and returns `canceled`; completion reports the number of emitted updates.
A window with no fresh observations aborts rather than fabricating tracking.

This is a **bounded observation stream**, not an autonomous tracking/motion
controller or a guarantee of persistent source identity. There is no inferred
range or 3-D position. The source must be transformed to `target_frame` by
the existing TF2 bridge. Tests cover stamp replay, generation transitions,
revocation and invalid geometry; real-array tracking accuracy and physical
motion admission remain future M3 hardware gates.
