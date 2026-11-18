# M4: conditional planar bearing uncertainty diagnostics

`audition_fusion/uncertainty.py` is a standalone mathematical **admission
gate** for hypothetical XY intersections, not a metric localization
publisher. It requires at least **three independently timed and translated
2D bearing rays**, including measured angular variance and **world-frame
origin pose covariance (2×2)** for *every* observation.

For candidate point `p`, unit ray `d_i`, origin `o_i`, and normal
`n_i = (-d_{iy}, d_{ix})`, the first-order, independent Gaussian model is

`r_i = n_i · (p-o_i)`

`Var(r_i) ≈ ||p-o_i||² Var(theta_i) + n_iᵀ Σ(origin_i) n_i`

`H = Σ_i n_i n_iᵀ / Var(r_i)`, `Σ(p) ≈ H⁻¹`

The implementation uses forward projected distance for the angular term
(only near-consistent rays are accepted), checks finite/PSD pose covariance,
minimum translation baseline and cadence, positive forward range,
normalized ray residual, normal-matrix conditioning and maximum major-axis
uncertainty. Any missing angular/pose variance returns **None** rather than
substituting a fabricated precision.

```bash
python3 -m unittest discover -s audition_fusion/test -p 'test_uncertainty.py' -v
```

**No production ROS publisher consumes this output.** Neither
`AcousticSource` nor the current TF2 transformer carries independently
calibrated world-frame robot pose covariance, correlated extrinsic errors,
source-motion model or verified direct-path confidence. A TF transform alone
is not pose covariance. Filling `LocalizedSource.position_covariance`
from this conditional model would be misleading. Actual sensor calibration,
pose uncertainty, synchronized observations, path/reflection discrimination
and stationary-source validation must precede publication. Cross-time
correlation (e.g. shared SLAM drift) invalidates the independent noise
assumption and may drastically underestimate uncertainty.

Synthetic tests demonstrate well-conditioned, missing calibration, non-PSD,
poor baseline, parallel geometry, temporal replay, contradictory ray and
large-error abstention. Passing these tests is not measured robotic accuracy.
