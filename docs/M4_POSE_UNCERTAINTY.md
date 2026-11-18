# M4: microphone-origin pose uncertainty

The pure-Python `audition_fusion/pose_uncertainty.py` adapts a measured planar
robot pose and full joint 3x3 covariance (x, y, yaw) to the XY origin and 2x2
covariance of a microphone with known fixed base-frame lever arm.

For base pose `(x,y,theta)` and calibrated offset `(lx,ly)`:

`p_mic = (x + cos(theta)lx - sin(theta)ly, y + sin(theta)lx + cos(theta)ly)`

The 2x3 Jacobian is

`J = [[1,0,-sin(theta)lx-cos(theta)ly], [0,1,cos(theta)lx-sin(theta)ly]]`

and the propagated covariance is `J C_pose J^T`. This is a *first-order*
conditional uncertainty propagation and requires a trustworthy full, symmetric,
positive-semidefinite pose covariance with consistent world-frame coordinates
and SI units. Unknown covariance or extrinsics fail closed.

**Not provided by ordinary TF2:** TF transforms contain geometry and
timestamps but not the robot's joint pose covariance. An external estimator
(e.g. state estimation with documented frame/time semantics) must supply it,
and an explicit calibration source must supply microphone extrinsics.
Covariance in a moving body frame must be rotated/converted before use.

This module does not model uncertainty in extrinsics, shared SLAM errors,
clock alignment, microphone angular calibration or cross-window pose
correlations. A covariance passed through this adapter must therefore never
be treated as independent evidence without additional validated modeling.
There is no `LocalizedSource` publisher or Nav2 coupling.

Test with:
`python3 -m unittest discover -s audition_fusion/test -p 'test_pose_uncertainty.py' -v`.
