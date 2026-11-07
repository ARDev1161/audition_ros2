# M4: composed diagnostic admission

`audition_fusion/diagnostic_pipeline.py` composes existing pure-Python M4
geometry, conditional covariance and temporal stability gates. It produces
`WindowConsensus` **only for diagnostic evaluation**. It does not publish ROS
`LocalizedSource`, infer depth from a single bearing, or drive Nav2.

## Evidence contract

The caller supplies two chronological, non-overlapping windows (3–8
observations each) for the same canonical acoustic source incarnation and
generation. Each `PlanarRay` must already be transformed into the same world
XY frame, with an independently measured nonzero angular variance and
world-frame 2x2 origin covariance. Validated camera/audio association,
microphone extrinsics and shared pose correlations are *not* derived here.

For **each** window: candidate triangulation requires three translated,
consistent ray observations; the independent `conditional_planar_covariance`
gate requires explicit uncertainty, a sufficiently conditioned information
matrix and acceptable normalized residual. The two admitted windows must
then be disjoint in time by at least 200ms and agree in space within both
the fixed distance threshold and the supplied uncertainty envelope.

## What passing does not prove

A persistent reflection may yield two consistent solutions, and systematic
SLAM/TF bias can affect both windows. Supplying arbitrary positive
covariance values is not sensor calibration. The two windows are not
independent if they reuse samples. No confidence probability or validated
physical emitter position is produced. A separate fully calibrated
measurement/provenance path is required before any `LocalizedSource` ROS
publisher or costmap integration can be enabled.

Run `python3 -m unittest discover -s audition_fusion/test -p 'test_diagnostic_pipeline.py' -v`.
The package CMake integrates this test into the ROS 2 Humble workflow.
