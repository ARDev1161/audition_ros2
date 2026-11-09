# M4 benchmark audit — 2026-11-09

Status: **software evaluation tooling exists; independent physical acceptance NOT VERIFIED**. This audit reviews repository code and documentation, not new measurements. Do not use this document to authorize metric 3D localization, Nav2, or costmap updates.

## Inventory and exact limitations

| Tool | Evaluation unit | What it measures | Cannot establish |
| --- | --- | --- | --- |
| `benchmarks/av_association_benchmark.py` | 12 procedural classes × 20 seeds × 3 frames = 240 trials | Instantaneous vs temporal AV confusion matrix, per-class failure | Real sensor performance, calibrated clocks/extrinsics, physical identity |
| `benchmarks/radar_association_benchmark.py` | 8 classes × 20 seeds = 160 trials | Unique radar/audio candidate confusion and aligned-impostor failures | Independent radar acquisition, true acoustic range |
| `benchmarks/score_av_groundtruth.py` | Fully annotated timestamped frames | Micro TP/FP/TN/FN, instantaneous vs temporal precision/recall/FPR | Independent event-level confidence intervals, gain vs single-modality baselines |
| `benchmarks/score_bearing_groundtruth.py` | Annotated 2D source frames | Candidate precision/recall, p50/p95 2D position/range errors | 3D metric source localization, covariance calibration, safety admission |
| `benchmarks/evaluate_field_manifest.py` | Independently labeled trials, held-out by default | Paired audio-only/radar-only/instant/temporal comparisons, adverse strata, provenance blockers | Physical validity without actual recordings, trustworthy independent labels and measured calibration |
| `benchmarks/common_mode_geometry_benchmark.py` | Constructed correlated-error cases | Negative-control geometry regression | A physical false-triangulation rate |

## Verified from source (not freshly rerun)

The documented deterministic AV synthetic regression expects instantaneous TP=40, FP=80, TN=120, FN=0; temporal TP=40, FP=40, TN=160, FN=0. Precision rises 1/3 to 1/2 and FPR falls 0.4 to 0.2 *only in this constructed dataset*. Two intrinsically indistinguishable classes, co-linear impostors and reflections, remain false positives.

The radar benchmark asserts TP=40, FP=40, TN=80, FN=0 and explicitly counts aligned impostors/reflections as false positives. These are source-level expected values, not field measurements.

The AV frame scorer counts each audio × visual candidate pair as a possible negative. Large cross-products can inflate TN and make FPR look favorable; always report raw FP and precision and cluster event-level estimates by recording/session. It rejects duplicate IDs, stale sessions and incomplete annotations; unannotated frames are skipped, not treated as negatives.

The bearing scorer computes position and absolute range error **conditional on an emitted 2D candidate**. Missing candidates are FN; do not interpret conditional error as all-trial localization accuracy. It does not evaluate calibrated 3D covariance.

The field manifest evaluator can reject missing held-out adverse strata, nonindependent acquisition groups, insufficient recording coverage, false-pair recordings/scenes, missing artifact hashes and missing provenance. A valid JSON fixture is not evidence of physical truth.

## Scope boundary: ROS2 integration, not libaudition algorithm validation

The independent acoustic simulator and ODAS/SST/VAD/ASR/source-separation quality evaluation belong to the separate **libaudition** workstream. Do not block or mislabel audition_ros2 software contract acceptance by attempting to measure libaudition acoustic algorithm accuracy here. M4 AV/radar integration metrics describe behavior of ROS2 adapters, synchronization, TF2, provenance, candidate association and failure handling **given controlled or recorded upstream outputs**. Once the separate simulator is ready, reuse its independently recorded outputs as ROS2 integration inputs, without duplicating acoustic-algorithm tests or claiming upstream quality from downstream scores. Full-stack end-to-end validation is a later joint milestone; real-robot hardware acceptance remains deferred.

## Priority execution plan

1. **Freeze evaluation protocol before data collection.** Record exact git SHA, model versions, matching thresholds, clock synchronization, sensor extrinsics, calibration uncertainties, consent/retention, and acquisition hashes. Freeze positive/negative and unobservable truth rules independently of predictions.
2. **Produce held-out ROS2 integration replay scenarios with controlled upstream observations** now, and consume independently generated simulator/physical acquisitions later; do not fabricate libaudition acoustic performance. At minimum cover static and moving emitters/robot, silent visual/radar distractors, co-linear impostors, reflections, poor translated baseline, calibration error, clock skew, session resets, dropped/reordered data. Separate development and held-out scenes, people and recording sessions.
3. **Run all four policies on identical acquisitions**: audio-only, radar-only (if available), instantaneous cross-modal and temporal cross-modal. Label unavailable outcomes N/A, never invent positions for bearing-only baselines.
4. **Score and audit** with `score_av_groundtruth.py`, `score_bearing_groundtruth.py` where 2D surveyed truth exists, and `evaluate_field_manifest.py` with recording-index, provenance, required-stratum and held-out gain checks. Publish confusion counts, FP by reflection/co-linear stratum, coverage, abstentions, identity switches, time-to-confirm, error p50/p95, recording/scene-clustered uncertainty.
5. **Admission decision:** only declare measured gain if held-out independent acquisitions show improvement without unacceptable false associations. Otherwise keep M4 diagnostic-only and report NO-GO. Hardware and physical accuracy remain UNVERIFIED until measured.

## Acceptance gap

**No independent real annotated AV/radar recording or physical acceptance report is present in the reviewed benchmark/documentation paths.** This is a future end-to-end gate, not a reason to reimplement or retest libaudition's audio algorithms in audition_ros2. The source already includes evaluators and strict manifest contracts, so another synthetic unit-ray benchmark is not the priority. Next engineering deliverable is a reproducible data export + held-out acquisition pipeline and an executed evidence report. Never promote planar bearing intersections to metric 3D localization or Nav2.
