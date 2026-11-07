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
