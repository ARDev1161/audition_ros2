# M4: multimodal fusion foundation

Branch: `feat/m4-multimodal-fusion`, draft PR #2, based on merged M0–M3 main.

## Diagnostic bearing-candidate pipeline

Opt-in `m4_bearing_candidates.launch.py` starts the `audition_fusion`
`bearing_candidate_node`. It consumes canonical `AcousticSourceArray`
from `sources` and produces unlatched `fusion/bearing_candidates`
(`BearingIntersectionCandidateArray`).

The scene incarnation is 32 lowercase hex, scene sequences and timestamps
strictly increase, and the source ID/generation and true `last_seen`
observation time are mandatory. TF2 resolves microphone origin and bearing
into `odom` (or configured world frame) **at the observation timestamp**.
Unknown TF, stale/retired sources, duplicate observations, high elevation or
invalid directions produce no candidate.

Three independently timestamped, spatially separated observations of the
same hypothesis must agree on an intersection. Short baselines, nearly
parallel/backward rays, inconsistent third bearings and invalid input are
rejected. Histories are bounded and cleared on incarnation change, source
retirement or no fresh data. Empty snapshots clear prior candidates.

```bash
ros2 launch audition_bringup m4_bearing_candidates.launch.py \
  audition_namespace:=robot_a/audition world_frame:=odom
ros2 topic echo /robot_a/audition/fusion/bearing_candidates
```

**Critical limitation:** Diagnostic XY intersections are not metric 3-D
localization. They have no metric Z, calibrated covariance, pose uncertainty,
stationary-source identity proof or reflection immunity. They must never
be published as `LocalizedSource`, occupancy or Nav2 cost.

## AV association: running opt-in diagnostic ROS graph

`av_pair_node` is wired to canonical `AcousticSourceArray` and independent
camera-origin `VisualBearingObservationArray` with exact-time TF2, session
fences, bounded ambiguity and temporal confirmations. It only publishes
`AudioVisualPairHypothesisArray`, never source identity or metric positions.
See [M4 AV pairing](M4_AV_ASSOCIATION.md).

## Conditional planar covariance (not a ROS position output)

[Uncertainty gate](M4_UNCERTAINTY.md) requires explicit independent
angular and sensor-origin pose covariances for every ray. Unknown uncertainty
or poor conditioning makes the gate abstain. The existing ROS sources and
TF2-only transforms lack the needed calibrated pose covariance, so this
module does not publish `LocalizedSource` or inject confidence into Nav2.

## Still required for M4 acceptance

- Live camera/radar observation adapter with independently sourced activity
  evidence, observation-time TF2, timestamp-domain and epoch fences,
  explicit unknown/invalid calibration handling, and ROS graph smoke tests.
- Extrinsic and pose uncertainty, 3-D covariance propagation, moving-emitter
  rejection, multi-hypothesis/reflection safeguards and validated identity.
- Benchmarked association precision/recall and range error on labeled moving,
  overlapping and reverberant environments; measured advantage over independent
  sensing before metric localization or navigation use.
- Hardware acceptance remains unverified; M4 is not ready to merge.


## AV evaluation tools

- [Adversarial synthetic bearing benchmark](M4_SYNTHETIC_BENCHMARK.md): deterministic regression dataset, including known unavoidable false associations
- [Strict ground-truth AV scorer](M4_GROUND_TRUTH_EVAL.md): externally annotated JSONL frames and confusion matrices, with full-session provenance
- [Planar range and position scorer](M4_BEARING_EVAL.md): surveyed source/robot XY truth, false triangulation and position/range errors
