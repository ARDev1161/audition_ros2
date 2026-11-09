# M4 temporal stability diagnostic (disjoint windows)

`audition_fusion/stationarity.py` adds `compare_disjoint_fixes` to reject
planar intersections that disagree across two **non-overlapping** time windows.

Required evidence:
- Same scene incarnation, acoustic source ID and source generation.
- Both fixes independently evaluated, with distinct input observations.
- Strictly increasing timestamps and at least 200 ms gap between windows.
- Finite strictly positive conservative major-axis error estimates.
- Spatial disagreement within both a fixed 0.5 m bound and three times the
  combined supplied major-axis standard deviation (tunable limits).

The returned midpoint is **diagnostic only**: it is not a Bayesian fused
estimate and cannot be published as `LocalizedSource`. The gate cannot prove
stationarity: a reflection may remain consistent, the same biased odometry
may affect both windows, and an emitter may move between samples. Do not
assume independent covariances unless the localization producer actually
models shared odometry/extrinsic error.

This is a separately installed pure-Python guard with regression tests.
It is not wired into the existing bearing candidate topic, which remains
explicitly diagnostic. Integration with a future metric publisher requires
pose-covariance inputs, calibration, independently verified data association
and direct-path/motion acceptance first.

Test: `python3 -m unittest discover -s audition_fusion/test -p 'test_stationarity.py' -v`.
