# M3ae — challenger bearing continuity during confirmed switching

With `switch_confirmations > 1` and the opt-in
`maximum_direction_jump_rad > 0`, consecutive challenger observations must
agree in bearing as well as in `source_id` and generation. A large jump
restarts the switch confirmation count at one, rather than permitting an
unstable source to win solely by persistent identity and activity.

The reference is the previous accepted challenger bearing, not the incumbent
bearing. Small changes within the configured angular threshold are accepted.
Changing candidate, exceeding the confirmation window or losing challenger
eligibility clears the pending reference. No additional ROS parameters are
required; both existing parameters retain their defaults.

This is a bearing-only advisory selection constraint. It does not estimate
distance, command navigation, or guarantee acoustic localization accuracy.
Four GTests cover large jumps, small changes, disabled gating, and candidate
replacement.
