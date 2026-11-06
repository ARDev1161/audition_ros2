# M3 camera aim proposals (opt-in, advisory only)

`m3_attention_camera_aim.launch.py` converts validated, timestamped
`attention/target_base` unit bearings in `base_link` into
`attention/camera_aim_proposal` (`audition_msgs/CameraAimProposal`).
Pan is `atan2(y,x)`, tilt is `atan2(z,hypot(x,y))` under REP-103 axes
(+X forward, +Y left, +Z up). Both angles are in radians and relative to
the configured target frame, **not** camera joint setpoints. The caller
must separately account for camera mount orientation and calibration.

The adapter is disabled by default. It does not publish `cmd_vel`,
`JointTrajectory`, camera driver topics, Nav2 goals or actuator commands.
Any future executor must explicitly authorize motion, validate joint limits,
collision safety and timestamps, and enforce independent emergency stop.

Admission requires a finite normalized forward-facing bearing, configured
pan/tilt bounds, valid timestamp, advancing scene/stamp and non-regressing
source generation. Invalid geometry revokes a previously active proposal
without advancing the watermark. Explicit retirement and monotonic-clock
expiry publish `valid=false` once; replay cannot reactivate stale aim.
These are observations only: there is no source distance estimate.

```bash
ros2 launch audition_bringup m3_attention_camera_aim.launch.py \
  audition_namespace:=robot_a/audition maximum_pan_rad:=1.2 maximum_tilt_rad:=0.7
ros2 topic echo /robot_a/audition/attention/camera_aim_proposal
```

Deterministic geometry/policy tests and a live DDS graph smoke test cover
axis signs, angular limits, replay, retirement, recovery and expiry. Actual
camera kinematics, motor safety and hardware pointing accuracy remain
**untested** and are not part of this advisory milestone.
