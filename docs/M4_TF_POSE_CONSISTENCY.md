# M4: opt-in TF2 / measured pose cross-check

`pose_evidence_node` can now require `require_tf_consistency:=true` when
launched with `m4_pose_evidence.launch.py`. Set `microphone_frame` to the
calibrated microphone frame (default `microphone_array`) and supply
consistent `Odometry` or `PoseWithCovarianceStamped` and TF2 transforms.

After timestamp matching, the node looks up exact-time TF2
`world_frame -> microphone_frame` for the acoustic observation stamp,
rejecting missing TF, static zero-stamp TF for a moving microphone, unexpected
parent/child frames, excessive TF/pose time skew, nonfinite coordinates or
XY disagreement exceeding `maximum_tf_pose_disagreement_m` (default 0.25 m).
A rejected observation increments `tf_rejected_bearings` and never
increments `matching_bearings`.

**This is not independent localization**: TF2 and pose covariance can be
derived from the same estimator, and agreement cannot prove correct
extrinsics, independence, direct acoustic propagation or physical emitter
position. The helper only compares microphone origins, not full 3D
orientation/covariance; no source position, robot action or Nav2 occupancy
is published. Turn on this cross-check only with a known synchronized
TF tree and measured microphone extrinsics.

Existing timestamp-only behavior remains available with
`require_tf_consistency:=false` (default). This is intentionally *not*
the confidence gate for publishing metric positions.

CI exercises pure geometry checks in
`test/test_tf_pose_consistency.py` and an opt-in ROS graph smoke
`audition_tests/smoke_m4_tf_pose_evidence.py`: missing TF, matching TF,
and contradictory TF with valid odometry.
