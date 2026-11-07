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

## Machine-readable independent trial manifest v1

A strict trial-level evaluator is available as:

```bash
python3 audition_fusion/benchmarks/evaluate_field_manifest.py \
    /path/to/independently_annotated_trial_manifest.json > report.json
```

The CLI defaults to **held-out trials only**. `--all-splits` can be used
for internal debugging, but is not an acceptance run. The input file must be
valid UTF-8 JSON, at most 16 MiB, with no repeated keys, NaN or Infinity.
`manifest_sha256` is included in the report, so the exact evaluated input
can be audited. The evaluator does **not** check that the referenced
recording bytes match the declared recording hash: this must be done by
the recording ingestion/verification procedure before calling the scorer.

The root object must have exactly `schema_version: 1`, `dataset_id`
and a nonempty `trials` array. Each trial has:

- `trial_id`: unique immutable event/trial ID, not an observation frame
- `stratum`: adversarial class, reported separately
- `split`: exactly `development` or `held_out`
- `recording_sha256`: lowercase 64-hex SHA-256 of recording artifact
- `truth`: `{"observable":true,"pair":{"audio_id":"...","radar_id":"..."}}`
  for an independently verified emitter/object pairing;
  `{"observable":true,"pair":null}` for a verified negative opportunity;
  `{"observable":false,"pair":null}` for identity that cannot be
  independently established
- `predictions`: exactly four predeclared strategies
  (`audio_only`, `radar_only`, `instant_fusion`,
  `temporal_fusion`), each with `{"decision":"pair","pair":{...}}`,
  `{"decision":"none","pair":null}`, or
  `{"decision":"abstain","pair":null}`

Both `none` and `abstain` must be used consistently.
`none` asserts that no acceptable correspondence was found; it
counts as TN **only if** the independently established ground truth says
there was no pair. `abstain` denotes insufficient evidence and
never becomes a true negative. Positive unrecognized associations
contribute FN; a wrong emitted pair on a positive trial contributes
both FP (incorrect candidate) and FN (missed true candidate), while
the trial itself is counted once. Unknown/unobservable truth is reported
as a separate count, not silently forced to a negative label.

The v1 validator also rejects any recording SHA-256 that appears in both
`development` and `held_out` trials: separate events from the same
recording are not independent test recordings and must not cross splits.
Multiple trials from the same recording **within** a split remain valid,
but their uncertainty estimates should be clustered by recording/trial.

The report includes TP, FP, TN, FN, abstention and unobservable counts,
precision, recall, F1, false-positive rate, and breakdown by stratum.
A rate with a zero denominator is `null`, not a fictitious 0% or 100%.
These are **association** metrics only; the contract does not infer
acoustic source range or validate a localization covariance.

`audition_fusion/test/fixtures/m4_field_contract_example.json` exists
solely to test parsing, report generation and the metric definitions;
its recording hash is a placeholder and it is **not field evidence**.
The scorer consumes externally supplied predictions and ground truth;
it does not fabricate annotations, run radar/audio inference, or prove
positive gain. That still requires frozen detector outputs for each
strategy on the exact same held-out recordings, independently checked
reference annotations, sensor calibration and trial-clustered uncertainty
analysis.

## Paired baseline comparison report (software diagnostic)

The trial-level evaluator also emits `paired_comparison`: for each of
`audio_only`, `radar_only` and `instant_fusion`, it compares the
`temporal_fusion` decision on the **same independently labeled trial**.
It reports both-correct, candidate-only-correct, baseline-only-correct,
both-incorrect and unobservable-excluded counts, per stratum and overall.
On verified negatives only the explicit `none` decision is correct;
an abstention is never counted as correct. An emitted incorrect pair is
a false association even when the true pairing was positive.

`paired_accuracy_delta = (candidate_only_correct -
baseline_only_correct) / observable_trials`, and
`paired_false_pair_delta_per_trial = (candidate_false_pairs -
baseline_false_pairs) / observable_trials`. Zero denominators produce
`null` rather than invented rates. `descriptive_improvement` only
indicates a positive observed paired accuracy difference; it is **not**
statistical significance, an independent-cluster confidence bound or
proof of safe physical performance. The report always explicitly marks
`positive_gain_validated: false`. Repeated event trials in a recording
are correlated; independent held-out scene/recording evaluation with
clustered uncertainty and a predeclared safety margin is still required
before granting the M4 release gate. Simulated and field trial reports
must never be pooled into one claim.

## Recording-cluster sensitivity (software evaluation increment)

The evaluator additionally emits `clustered_comparison` using the **verified
recording SHA-256** as the resampling unit. Multiple trial events in the same
recording remain one cluster. The diagnostic reports observed-trial counts per
recording, the pooled paired gain and false-pair delta, leave-one-recording-out
ranges and a reproducible 512-draw percentile cluster bootstrap with fixed
seed. It resamples entire recordings (not individual event rows). Fewer than
five recording clusters yield `null` bootstrap bounds; fewer than two yield
`null` leave-one-recording-out bounds. Unknown-truth opportunities are
excluded rather than scored as successful.

**These are exploratory intervals, not M4 acceptance thresholds.** Even
different recording hashes may contain the same room, speaker, robot session,
sensor calibration error or correlated synthetic scenario. The SHA-256 groups
do not establish physically independent subjects or scenes; no guarantee of
95% real-world interval coverage is made, and `physical_validation_claim`
remains `false`. The release gate still requires externally documented
scene/subject/session partitioning, predeclared safety limits, calibrated
sensors and independent field truth. A confidence interval above zero on
synthetic data is not proof that fusion safely improves physical localization.
