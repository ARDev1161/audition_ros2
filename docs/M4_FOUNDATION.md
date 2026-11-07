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

## AV association: bounded, conservative pure logic (not yet wired to ROS)

`audition_fusion/av_matching.py` introduces a ROS-independent `PairMatcher`
and `StreamFence` with unit tests. This is a preparatory logic module,
**not a live AV fusion node** and not proof of speaker identity.

The caller must provide independent timestamped world-frame audio and
visual 3-D *unit rays* with measured angular variances, stable track
identifiers/generations, and independently observed activity evidence.
The matcher rejects nonfinite or invalid rays, unknown/zero variance,
inactive tracks, excess temporal skew, excessive sensor separation,
angular disagreement, oversized batches and duplicate track identities.
It returns candidate pair suggestions and a `unique_match` flag only
when a pair wins by a configured margin in *both* audio and visual groups.
Ambiguous alternatives are preserved as nonunique suggestions, not silently
assigned. The normalized angular residual is a heuristic gate; it is **not**
a calibrated probability or a 3-D covariance. The baseline term uses a
configured assumed minimum range, never a measured distance.

`StreamFence` rejects repeated/out-of-order snapshots and older epochs.
The producer must explicitly reset it after a verified ROS clock reset.
Before a ROS graph consumer is enabled, it must additionally require
observation-time TF2, independently measured camera intrinsics/extrinsics,
bounded time synchronization and freshness/retirement clearing.
No `/fusion/av_pairs` topic is currently claimed to exist.

Tests:
```bash
python3 -m unittest discover -s audition_fusion/test -p 'test_*.py' -v
```

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
