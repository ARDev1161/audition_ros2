# M4-C: opt-in radar observation adapter (ROS2 only)

The M4-C adapter does **not** implement radar DSP or infer distance from an
acoustic bearing. A radar tracker outside audition_ros2 supplies measured
positions and finite sensor-coordinate covariance through
`/<ns>/radar/tracks` (`RadarTrackArray`). Each input carries the real radar
`header.frame_id`, an increasing per-session sequence, a lowercase 32-hex
`radar_session_id`, and a complete bounded active-track snapshot.

Launch:

```bash
ros2 launch audition_bringup m4_radar_adapter.launch.py \
  audition_namespace:=audition world_frame:=odom
```

The `radar_adapter_node` uses TF2 at each radar observation timestamp,
validates coordinates and 3x3 symmetric sensor covariance, and emits
`/<ns>/fusion/radar_observations` (`RadarObservationArray`). These are
**radar-origin hypotheses**, not acoustic emitters, identities or Nav2
obstacles. No acoustic distance is inferred, and no automatic radar/audio
cross-association is performed in this first adapter increment.

Important limitations: rotated output covariance includes the radar's
reported measurement noise **only**. Uncertainty in radar extrinsics,
odometry, TF2, timestamps and any correlated tracking must be accounted
for before using it as an absolute world-frame localization covariance.
Unknown/zero axis variance, malformed transforms, stale observations,
missing TF, duplicate track IDs, replayed snapshots and invalid sessions
are rejected. Snapshot arrays are bounded to 16 tracks by default.
Snapshots are unlatched: an empty output withdraws the previous tracks;
silence triggers a bounded TTL clear.

The ROS 2 Humble CI runs a pure-Python covariance/rotation regression,
a message serialization contract, and an opt-in real DDS/TF2 graph smoke
with a synthetic **radar measurement fixture**, not a sound simulator.
No radar hardware, Gazebo, Isaac Sim or robot HIL acceptance is claimed.

Later M4 work may match these independently measured radar coordinates
to acoustic bearings with explicit association uncertainty and ambiguity
preservation. That must remain a diagnostic hypothesis until independent
ground-truth evaluation establishes precision, recall and false matches.
