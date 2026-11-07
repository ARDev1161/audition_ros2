# M4: diagnostic audio/visual pair hypotheses

The independent ROS 2 node `av_pair_node` is **opt-in** and has no actuator,
navigation, speaker-identification or object-position output. Launch with:

```bash
ros2 launch audition_bringup m4_av_pairs.launch.py \
  audition_namespace:=robot_a/audition world_frame:=odom
ros2 topic echo /robot_a/audition/fusion/av_pair_hypotheses
```

It consumes `sources` (`AcousticSourceArray`, produced by the existing
libaudition ROS adapter) and `vision/bearings`
(`VisualBearingObservationArray`, supplied by an **independent** camera
tracker). All arrays must have a canonical 32-hex session incarnation,
monotonically increasing snapshot sequence and timestamp, and correct
sensor `header.frame_id`. Each track must carry an actual sample time,
generation and a unit sightline. **No camera or mouth-motion detector is
implemented here**: the visual producer owns calibration and the truthfulness
of `activity_evidence`, such as observed lip/mouth motion.

Each ray and its sensor origin are transformed using TF2 **at the actual
observation stamp** into the configured world frame. Input is refused for
missing TF, old/invalid timestamps, inactive tracks, unknown angular variance
or invalid geometry. The pair gate requires maximum 150 ms observation-time
difference, maximum 20 cm sensor-origin baseline, maximum 0.30 rad angle
difference, and bounded normalized residual (defaults, configurable).

The baseline requirement is intentional: without measured source depth,
translating camera and microphone origins introduces parallax. The matcher
conservatively models an *assumed* minimum range in the normalized residual;
this does NOT estimate distance or calibrated covariance. It cannot claim
the observed face is the acoustic emitter.

All plausible pairs are published as `AudioVisualPairHypothesisArray`.
`unique_match` means a mutual best match with a score margin over currently
available alternatives, not verified identity or a calibrated probability.
Competing pairs remain in the same snapshot with `unique_match=false`.
Input histories and track counts are bounded (max 16 each). Missing fresh
input, explicit empty snapshots, epoch changes and expiry withdraw matches
via an empty array. The output must **never** be forwarded to Nav2 costmaps,
motion controllers or `LocalizedSource` without independent metric
localization, uncertainty accounting and separate safety review.

Unit testing covers skew, activity, variances, duplicate IDs, ambiguity and
stream replay. Dedicated live ROS graph testing validates actual TF2
calibration, pairing, ambiguity and removal against synthetic observations.
Real-camera evaluation, AV precision/recall, reflections and moving-source
validation remain outstanding.
