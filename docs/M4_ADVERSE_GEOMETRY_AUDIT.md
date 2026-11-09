# M4 adverse conditional geometry audit

The automated benchmark
`audition_fusion/benchmarks/conditional_geometry_adversarial.py`
runs the **real** `DiagnosticEvidenceBuffer` and its existing conditional
covariance and two-window consensus pipeline on six reproducible,
timestamped planar bearing observations.

It checks four deliberately distinct cases:

| Scenario | Expected result | Interpretation |
| --- | --- | --- |
| Stationary direct-path synthetic source | Admitted | Positive geometry control |
| Source moves 2 m between disjoint windows | Rejected | Temporal stationarity control |
| Coherent virtual image of a reflected path | Admitted | **Known false-positive risk** |
| Robot/SLAM positions share a common +1 m bias | Admitted | **Known unobservable systematic error** |

The two admitted adverse controls are **not successful source localization**.
They demonstrate the exact limitations of a conditional model that assumes
direct-path rays and independent pose/angular errors. Because the source
identifier is held fixed, the benchmark does not test association swaps.
No source position, obstacle or Nav2 cost is published.

This is a geometry-only test, not a sound-wave/room impulse response
simulation; it does **not** claim to quantify the rate or magnitude of
reverberation-induced errors in a room. The eventual validation campaign
must provide an independent ground-truth dataset, room impulse responses,
moving microphone and emitter trajectories, calibrated angle variance
and realistic correlated localization errors.

Run locally (plain Python 3, no ROS required):

```bash
python3 audition_fusion/benchmarks/conditional_geometry_adversarial.py
```

The script prints a machine-readable JSON summary including
`physical_localization_validated: false`, then exits nonzero if any
expected control behavior changes. A green result validates the expected
boundary of the implementation, *not physical accuracy*.

## Correlated-error magnitude audit (additional negative control)

Run:

```bash
python3 audition_fusion/benchmarks/common_mode_geometry_benchmark.py
```

This uses the existing two-window `evaluate_disjoint_windows` implementation
to assess six timestamped planar rays against a separately defined *synthetic*
true emitter at (3 m, 2 m). Cases include an unperturbed source, a shared
0.5 m or 1 m systematic error across all reported microphone positions,
and a coherent reflection-image source. The output contains the conditional
major standard deviation and *actual error relative to synthetic truth*.

In the shared-pose-bias cases the observed bearings still come from the true
emitter: only reported microphone positions shift together. The triangulated
candidate can therefore shift by 0.5 m or 1 m while the internal conditional
covariance remains essentially unchanged. The stable virtual-image control
can produce a 4 m error while still passing the geometry consistency gate.

**Critical interpretation:** These are intentional, passing *negative*
controls that demonstrate false confidence. Neither the geometry test nor
its results justify publishing `LocalizedSource`, generating Nav2 costs,
or asserting any real-world acoustic localization accuracy. Independent
reference pose measurements, calibrated extrinsics, source truth and actual
room-acoustic data are still required. The audit does not model a room
impulse response, distinct physical acoustic paths, or correlated SLAM
covariance across recordings.
