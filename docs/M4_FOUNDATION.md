# M4: multimodal fusion foundation

Branch: `feat/m4-multimodal-fusion`, based on merged M0–M3 main.

## Diagnostic bearing-candidate pipeline

Opt-in `m4_bearing_candidates.launch.py` starts the `audition_fusion`
`bearing_candidate_node`. It consumes **canonical**
`audition_msgs/AcousticSourceArray` from `sources` (not attentional
single-target `AttentionTarget`) and produces an unlatched
`fusion/bearing_candidates` (`BearingIntersectionCandidateArray`).

Canonical scene incarnation `scene_session_id` (32 lowercase hex), strictly
increasing scene sequence, the source ID/generation, and true `last_seen`
observation time are required. Every input must have a fresh ROS timestamp.
TF2 resolves the microphone-origin position and rotated bearing into
`odom` (or configured `world_frame`) **at the observation timestamp**,
never at the latest transform. Unknown TF, rejected/retired sources, duplicate
observations, high elevation or invalid directions produce no candidate.

Three independently timestamped and spatially separated observations of
the *same hypothesis* must agree on an intersection. Geometry rejects short
baselines, nearly parallel rays, backward rays, inconsistent third bearings,
nonfinite and non-unit input. Bounded histories (8 observations per 16
tracks) are cleared on incarnation change, source retirement or no fresh
data. Empty snapshots clear prior candidates.

```bash
ros2 launch audition_bringup m4_bearing_candidates.launch.py \
  audition_namespace:=robot_a/audition world_frame:=odom
ros2 topic echo /robot_a/audition/fusion/bearing_candidates
```

**Critical limitations:** The output is an *unvalidated diagnostic XY
candidate*, with no metric Z, calibrated covariance, pose uncertainty, proof
that the stationary physical emitter was correctly associated or certainty
against reflection. It is deliberately NOT `LocalizedSource`, an obstacle,
or a Nav2 input. The 2-D intersection code is ROS-side cross-observation
geometry; audio DSP, ODAS, ASR and classification remain in libaudition.

## Still required for M4 acceptance

- Explicit AV evidence and observation-time synchronization (camera/radar).
- Extrinsic/pose uncertainty, propagation to 3-D covariance and moving
  emitter rejection; ambiguity preservation and reflection safeguards.
- Benchmarked source association precision/recall and range accuracy on
  labeled moving and reverberant environments.
- Demonstrated gain over independent sensing before enabling metric
  localization or any navigation consumer.
