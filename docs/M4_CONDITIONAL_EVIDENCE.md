# M4 conditional two-window evidence (diagnostic only)

The opt-in `pose_evidence_node` can now pass **measured** microphone-origin
covariance and **measured** audio angular variance through the existing
`diagnostic_pipeline.evaluate_disjoint_windows` gate. This integrates:
canonical scene/session replay fencing, bounded fresh pose samples,
exact-time TF2 pose-origin agreement, planar bearing rotation, a bounded
6-ray per-source history and two nonoverlapping 3-ray windows.

This feature is disabled by default. To enable, supply a calibrated
base-to-microphone translation and consistent world-frame pose covariance,
then set these node/launch parameters:

```text
require_tf_consistency:=true
enable_conditional_evidence:=true
microphone_extrinsics_calibrated:=true
```

The calibration Boolean is an **operator declaration**, not proof of
calibration. Provide `microphone_lever_arm_xy_m` and ensure a real
`world_frame -> microphone_frame` TF2 chain for the acoustic sample
timestamp. AcousticSource must carry `has_angular_variance=true`, a finite
positive `angular_variance_rad2`, calibrated DOA, a plausible planar
bearing, and a valid stable source ID + generation.

Each source/session/generation buffers a maximum of six admitted rays.
Six rays are evaluated once as two disjoint windows. Missing, stale,
duplicate, uncalibrated or inconsistent evidence fails closed, and
session/clock resets discard the buffer. Per-source buffers are bounded,
and absence in the next accepted snapshot revokes a source's history.

The published `DiagnosticArray` contains **only counts**:
`conditional_evaluated`, `conditional_admitted`,
`conditional_rejected`. It deliberately publishes NO XY, confidence
ellipse, `LocalizedSource`, occupancy layer or navigation command.

**Critical limitation:** the conditional covariance model assumes
independent ray errors. Shared SLAM drift, correlated pose estimation,
microphone extrinsic uncertainty, moving sources, source-ID swaps,
reverberation/multipath and DOA variance calibration errors are NOT
established by this implementation. Consequently, a successful diagnostic
window is **not** localization ground truth and must NOT feed Nav2.
Future acceptance requires synchronized recorded/simulated multichannel
acoustic scenes, correlated error treatment and independent ground truth.

Unit tests: `test_ray_evidence.py` and `test_evidence_buffer.py`
are included in `audition_fusion`/ROS2 Humble CI.
