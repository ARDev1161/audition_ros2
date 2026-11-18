# M4 pose evidence node (diagnostic only)

An opt-in ROS 2 executable `audition_fusion/pose_evidence_node` receives
`fusion/input_pose` (`nav_msgs/Odometry` by default, or
`geometry_msgs/PoseWithCovarianceStamped` with `pose_type:=pose_stamped`)
and canonical `sources` (`audition_msgs/AcousticSourceArray`).
It publishes `fusion/pose_evidence_diagnostics`
(`diagnostic_msgs/DiagnosticArray`) with admitted/rejected pose counters,
time-matched/unmatched active bearings and bounded pose cache size.

Launch:

```bash
ros2 launch audition_bringup m4_pose_evidence.launch.py \
  audition_namespace:=audition world_frame:=odom pose_type:=odometry
```

Relative topics are scoped beneath the namespace. No source position,
`LocalizedSource`, costmap or robot command is published. This node is
**not** yet connected to `diagnostic_pipeline.py`: it establishes a measured
pose message ingestion and timestamp-matching contract first.

Pose messages must carry finite, nonzero joint covariance for (x,y,yaw), a
small roll/pitch, compatible ROS clock and the exact declared world frame.
Odometry `child_frame_id` is not substituted for `header.frame_id`.
The microphone lever arm is a configurable static XY offset; the launch
default [0,0] is a *diagnostic placeholder*, not physical calibration.
Default buffer capacity is 32, maximum age 500ms and maximum matching
skew 50ms. A clock rollback clears the cache. Reordered, duplicate,
future and stale samples are rejected, and expiry is automatic.

A timestamp match means neither TF2 quality nor causal association:
pose-covariance estimation may contain common-mode biases and must be
validated before it can be used as independent source-position evidence.
No interpolation, audio identity inference, 3D triangulation or metric
localization is claimed.

Tests:
`python3 -m unittest discover -s audition_fusion/test -p 'test_pose_sample_cache.py' -v`.

Follow-up acceptance: ROS-graph smoke for pose message flow, TF2/extrinsics
validation, coordinated cache/session resets, explicit error counters and
calibrated covariance provenance.
