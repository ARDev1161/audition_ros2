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
