# M3w: quarantine discontinuous attention bearings

The optional `maximum_direction_jump_rad` continuity gate checks the
direction of the currently selected source against the last accepted
advisory. A sudden jump can mean a tracker identity switch; it must not
become a valid new target on the next scene just because the previous
advisory was invalidated.

When the gate detects a jump, `AttentionPolicy` records the source ID
and source generation as quarantined. Further snapshots containing the
same identity cannot select it, even after a scene timeout. Independent
eligible sources can still win selection. A subsequent *accepted* scene
without that identity releases the quarantine, as does a new generation,
frame, or explicit policy reset. Regressed snapshots cannot clear it.

This is intentionally conservative: a real fast-moving source with a
stable track ID may be ignored until it disappears/reappears or the stream
generation changes. The feature is opt-in and disabled at the default
`maximum_direction_jump_rad: 0.0`.

Four pure-policy regression tests cover the bypass, alternate candidates,
accepted absence, and timeout versus new-generation behavior. This is an
advisory bearing policy only: no Nav2 commands, metric range, or safety
guarantee are implied. Hardware motion and real ODAS tracking accuracy
remain unverified.
