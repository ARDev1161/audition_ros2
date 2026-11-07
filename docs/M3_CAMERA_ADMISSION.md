# M3 opt-in camera aim admission (advisory only)

This is a **second policy boundary**, not a camera controller. Start
`m3_attention_camera_admission.launch.py` explicitly. It subscribes to
`attention/camera_aim_proposal` and publishes
`attention/camera_aim_admitted` (same `CameraAimProposal` type).
No motor, joint, camera, Nav2 or `cmd_vel` command is emitted.

Default is **disabled**. An independent supervisory component must repeatedly
publish `std_msgs/Bool(data=true)` on `attention/camera_aim_enable`.
Each heartbeat authorizes at most `authorization_timeout_ms` (default 300 ms).
A false message immediately revokes the current admission. A heartbeat timeout
or expired proposal also publishes a single `valid=false` advisory.
There is no retained/latching enable state across restarts.

Proposals must be stamped, in the configured frame, monotonic in
(scene generation, stamp), have source IDs at most 256 UTF-8 bytes, finite
angles, and lie within configured pan/tilt bounds. Invalid geometry does not
advance the replay watermark. Retirement invalidates the current proposal.
This is *not* a hardware safety controller: any downstream actuator driver
still needs independent physical interlocks, calibration, joint feedback,
collision checks, emergency stop and bounded command execution. Never wire
this advisory directly to a motor controller.

Example:

```bash
ros2 launch audition_bringup m3_attention_camera_admission.launch.py
# A separate trusted supervisor must issue short-lived enable heartbeats.
ros2 topic echo /audition/attention/camera_aim_admitted
```

Policy unit tests cover default denial, timeout, revocation, replay,
retirement, malformed identity, frame and angle bounds. Hardware validation
remains outside this milestone.

An explicit retirement from a foreign source or older tracker generation is ignored while a different active aim exists; it cannot poison the replay watermark. This is a ROS policy gate, not a cryptographic authentication mechanism.
