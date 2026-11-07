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

## Optional diagnostic radar/acoustic association primitive

`radar_bearing_matching.py` is a **pure-Python, non-publishing** candidate
matcher that can be fed independently observed world-frame `Ray` bearings
and radar points after TF2 conversion. It conservatively gates finite
timestamps (default maximum 150 ms skew), valid 3-D geometry, forward
radar-point distance along an acoustic ray (0.5–30 m), ray/point angular
difference, measurement variance, and per-frame track capacity.

For each eligible edge, it reports the radar point's **distance along the
acoustic ray**, geometric lateral residual, normalized *diagnostic* residual
and whether the edge is isolated among other candidates on both sides.
This distance is **not** measured acoustic source range; the radar object
may be an unrelated person, wall or reflector aligned with a sound.
Ambiguous many-to-one and one-to-many alternatives remain explicit and
cannot be promoted to unique matches.

The uncertainty denominator is a conservative approximation combining
angular variance at range with the trace of radar measurement covariance.
It is **not** calibrated probability, a cross-sensor covariance fusion
algorithm or evidence of sensor-error independence. Accordingly this
increment intentionally has **no ROS publisher, no `LocalizedSource`,
no automatic audio/radar identity claim, and no Nav2 consumer**.

CI unit tests cover correct geometry, impossible angles, behind-sensor
and excessive distance, time skew, invalid covariance, duplicate identity
and reciprocal ambiguity. Physical-sensor precision/recall and the M4
ground-truth gate remain deferred.

## M4-C deterministic synthetic association benchmark

Run `python3 audition_fusion/benchmarks/radar_association_benchmark.py`
for 160 labeled synthetic fixtures (8 classes × 20 seeds). The explicit
ground-truth construction separates true direct associations from off-axis,
rear-facing, time-skewed and one-to-many ambiguous alternatives, plus two
adversarial negative classes whose measured inputs exactly match a true
association: an aligned radar impostor and an aligned reflected sound.

The fixed **expected** synthetic confusion matrix (not yet CI-verified for
this commit) is TP=40, FP=40, TN=80, FN=0: precision 50%, recall 100%,
false-positive rate 33.3%. The 40 unavoidable false positives arise from
the aligned impostor and reflection classes. These metrics are scoped
**only to the constructed examples**, not real people, acoustic recordings,
radar hardware or a measured cross-sensor improvement. They demonstrate
that geometric compatibility alone cannot prove radar/audio identity,
justify `LocalizedSource`, or satisfy M4's physical acceptance gate.

## M4-D temporal candidate corroboration (software-only)

`radar_temporal.py` provides an optional bounded `RadarPairTemporalGate`
over the diagnostic edges of `RadarBearingMatcher`. A pairing needs at
least three independently **advanced audio and radar observations**
(default minimum spacing 100 ms, maximum gap 600 ms). The gate carries
evidence only for a stable pair of source ID+generation and radar track
ID+generation within unchanged audio and radar session identifiers.

It immediately invalidates history on any ambiguity, explicit empty
snapshot, session change, source generation change or clock rollback.
Repeated observations from either modality cannot increase evidence.
Its `temporally_correlated` flag is deliberately **not** an assertion of
speaker identity, accurate range, sensor independence, or physical
localization: an aligned impostor or reflection may match for many
consecutive frames. This module is pure Python and has no ROS publisher,
Nav2 consumer, robot control, or acoustic simulator dependency.

CI runs `test_radar_temporal.py` as part of the existing radar test
pattern. Promotion to a ROS-level consumer is deferred until a
ground-truth comparison documents false matches and independent-sensing
baseline performance.

## M4-D opt-in integrated ROS2 diagnostic graph

Launch the dedicated radar + acoustic diagnostic chain with:

```bash
ros2 launch audition_bringup m4_radar_diagnostics.launch.py \
  audition_namespace:=audition world_frame:=odom
```

`radar_adapter_node` consumes independent `radar/tracks` and emits
`fusion/radar_observations`; `radar_audio_association_node` additionally
consumes canonical acoustic `sources`, transforms each accepted acoustic
bearing into the common frame at the observation timestamp, and publishes
`fusion/radar_audio_candidates` as `RadarAudioCandidateArray`.
The output gives the *radar target* distance projected along the *acoustic
ray*, perpendicular geometric mismatch, pair ambiguity and up to three or
more separately refreshed cross-sensor observations. It never reports a
measured acoustic distance, verified identity or position covariance of a
sound emitter.

Accepted input frames are bounded by observation TTL, monotonically
increasing per-session sequence and capture time, valid 32-hex stream
incarnations, actual stamped TF2, measured sensor covariance, and reciprocal
ambiguity. Old stream incarnations cannot be reopened by newer timestamps.
When either authoritative snapshot is empty, competing matches are
published but cannot be confirmed; when the input producer stops, output
candidates expire. The graph publishes no `LocalizedSource`, Nav2
costmap entries or motion commands.

The ROS2 Humble CI smoke
`audition_tests/smoke_m4_radar_audio_graph.py` exercises the **actual
combined DDS/TF2 graph**, verifies three independent observations, two
radar candidates competing for one acoustic bearing, two acoustic
sources competing for one radar object, explicit retirement, session
restart and old-epoch rejection. CI acceptance is still software-only;
synthetic fixture correctness does not demonstrate physical precision.

## M4-D paired benchmark: single-frame versus three refreshed observations

Run both deterministic, **synthetic-only** radar evaluations:

```bash
python3 audition_fusion/benchmarks/radar_association_benchmark.py
python3 audition_fusion/benchmarks/radar_temporal_benchmark.py
```

The latter compares two strategies on the **same 180 labeled scenarios**
(9 classes × 20 seeds; 3 synchronized frames/scenario). Each scenario
has one truth label, scored **once per scenario**, never once per frame:
`single_frame_any` accepts any isolated geometrically compatible frame;
`three_independent_updates` accepts only after the radar and audio timestamps
have each advanced in the same stable ID/generation/session. The ninth
class is a one-frame coincidence that becomes clearly off-ray thereafter.
The original eight classes include plausible static impostors and
reflections that stay geometrically indistinguishable from the true source.

Expected fixed fixture counts:

| Synthetic strategy | TP | FP | TN | FN | Precision | Recall | FPR |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Any one compatible frame | 40 | 60 | 80 | 0 | 40% | 100% | 42.9% |
| Three independently refreshed frames | 40 | 40 | 100 | 0 | 50% | 100% | 28.6% |

These are unit-test regression expectations, **not measurements from real
radar, acoustic recordings or people**. Both methods still falsely
associate 20 aligned impostors and 20 reflected signals. The temporal
method's apparent synthetic improvement comes entirely from the
deliberately constructed transient category. It establishes neither
a positive real-world fusion gain over standalone sensors nor completion
of the Roadmap's physical M4 acceptance gate. In particular, the matcher's
`radar_range_along_audio_ray_m` is a distance to a radar return, not a
measured acoustic source range. Do not activate `LocalizedSource` or Nav2
from these fixtures.
