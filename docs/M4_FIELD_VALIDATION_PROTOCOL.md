# M4 field-ground-truth evaluation protocol (not yet executed)

**Status: collection/acceptance protocol only.** The existing CI fixtures and
audio/radar TF2 tests validate software contracts, not localization accuracy,
sensor calibration, person identity or real-world association performance.
Use this protocol before declaring the M4 Roadmap gate passed.

## Required measurement provenance

For each synchronized trial record a human-readable trial ID, immutable
recording artifact hash, sensor serials/types, mounting positions, frame IDs,
calibration and software revisions. Retain the raw multi-channel capture
with its sample clock, radar native measurements, available camera detections
(if used), ROS2 clock/TF/odometry and any loss/restart events. Include measured
clock offsets and drift with uncertainty; DO NOT silently align streams from
their detection timestamps after evaluating results.

Keep ground truth **independent** of model/track predictions: surveyed
emitter position over time, activity/speech onset and end, true physical
emitter identifier, radar object association or an explicit `unobservable`
label, plus confidence or annotation limitations. A radar reflection or
silent bystander can have a legitimate track without being a sound source.
Never label any alignment as true merely because its ray residual is small.

## Trial matrix and adversarial strata

- One stationary verified emitter; a moving microphone/robot with
  measured extrinsics, translated baseline and pose covariance.
- Several simultaneous emitters and silent radar/visual bystanders,
  with collisions and exchanges in bearing.
- Moving speakers, moving non-speakers, and intermittent occlusion or loss
  of microphone, radar and camera observations.
- A reflective environment: strong direct path versus plausible
  loudspeaker reflections, including geometry where a reflection and
  real track share a nearly identical bearing.
- Clock offset, skew, dropped/reordered packets, session restarts and
  expired source generations; demonstrate fail-closed behavior.
- Intentionally poor-baseline and poorly conditioned geometry plus
  independently measured extrinsic calibration perturbations.

Each declared stratum must include positive and negative ground truth
opportunities and report its own counts; do not pool away high-risk failure
cases or quietly exclude abstentions.

## Predeclared scoring

Before looking at results, freeze matching time tolerance, truth association
rules, candidate confidence thresholds, temporal settings, and treatment
of identity handoffs. Evaluate at the **physical event/trial level** and
report frame-level metrics separately; multiple publication frames from a
single event are not independent trials.

Report TP, FP, TN, FN and abstentions with a documented denominator,
association precision/recall/F1, false positive rate, identity swaps,
track handoff errors, time-to-confirm, session-restart failures, dropped
snapshots, and estimated clock skew. An `unobservable` physical identity
must **not** be forced into a positive label. Report false associations
separately for reflections, co-linear impostors and moving entities.

For localization-capable configurations, independently report metric
position/range error distribution, covariance coverage/calibration,
conditioning, frame and timestamp errors; a radar-object range is not an
acoustic-emitter range. Compare identical held-out trials using:
(1) acoustic-only bearings/source labels,
(2) independent radar-only object tracking where applicable,
(3) instantaneous cross-modal association,
(4) temporally corroborated cross-modal association.
Where a baseline cannot estimate an outcome, report it as **not applicable**
instead of zero error or a fabricated localization measurement.

Fix training/tuning trials before evaluation; split evaluation by scene,
physical subjects and acquisition session, and publish per-stratum results
with bootstrap intervals clustered by trial. Require measured improvement
on the declared target outcome, with no unacceptable increase in unsafe
false associations, before any downstream `LocalizedSource` or Nav2
consumer is considered. Synthetic regression results alone cannot meet
this criterion.

## Reproducible release evidence

An M4 acceptance report must link artifact hashes, split manifests, fixture
schema, ground-truth annotation protocol, complete evaluation command,
versioned parameters and a machine-readable confusion/error report.
Document missing hardware coverage explicitly. As of this protocol update,
**no such field acceptance report has been completed**.
