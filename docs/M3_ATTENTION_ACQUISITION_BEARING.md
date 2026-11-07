# M3ad: bearing continuity during initial acquisition

When both `acquisition_confirmations > 1` and
`maximum_direction_jump_rad > 0`, each accepted observation of the
pending candidate must have a bearing within the configured angular
threshold of its preceding observation. A jump resets the acquisition
confirmation count to one and starts a new window. Small angular changes
remain admissible. This is independent of the existing incumbent quarantine:
a not-yet-selected candidate is not quarantined merely for a jump.

When the direction jump gate is disabled, the legacy confirmation behavior
is unchanged. Generation, frame, timeout, identity changes and invalid
snapshots retain their existing fences. The output remains an advisory
bearing, never a motion command.

Three deterministic C++ regressions cover large jumps, small changes and
opt-out behavior.
