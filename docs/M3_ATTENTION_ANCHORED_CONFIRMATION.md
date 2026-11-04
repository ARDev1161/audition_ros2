# M3af: anchored direction confirmation

Acquisition and challenger switch confirmation now compare every accepted
bearing to the **first** bearing in the current confirmation run, rather than
to the preceding sample. A series of small incremental rotations can otherwise
accumulate into a large overall drift while satisfying every pairwise check.

A candidate change, expired confirmation window, or bearing beyond the configured
`maximum_direction_jump_rad` restarts the run and establishes a new anchor.
Jitter within the limit remains acceptable, including samples on opposite
sides of the initial direction. This applies only when the existing optional
direction-jump gate and multiple confirmations are enabled; defaults do not
change. The retained incumbent's own continuity quarantine is unchanged.

Three deterministic GTests cover acquisition drift, switch drift, and
nonmonotonic small jitter. This remains an advisory DOA policy, not a motion
controller or a guarantee of acoustic source identity.
