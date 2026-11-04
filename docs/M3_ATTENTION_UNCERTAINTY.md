# M3k — opt-in DOA angular-uncertainty gate

The advisory `attention_manager` now supports the startup-only ROS parameter
`maximum_angular_variance_rad2` (default `0.0`, **disabled**).

When set to a positive finite threshold, a source must report
`has_angular_variance=true` and a finite nonnegative
`angular_variance_rad2` no greater than the threshold to be eligible for
attention. A missing, NaN, infinite, negative, or excessive variance makes the
source ineligible. When disabled, existing eligibility and selection behavior
are unchanged.

Example:

```bash
ros2 run audition_behaviors attention_manager --ros-args \
  -p maximum_angular_variance_rad2:=0.05 \
  -p switch_confirmations:=3
```

This is an **uncertainty gate**, not a calibrated probability or a guarantee
that a source is a person or object. Only enable it when the spatial backend
actually provides meaningful angular variance. If it does not, all sources
without this field will be filtered. No motion command, range inference or
Nav2 action is introduced.

Four pure-policy tests cover missing/excessive variance, malformed values,
default compatibility and parameter validation.
