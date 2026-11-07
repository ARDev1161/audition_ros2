# M3aj — independent confirmation evidence via scene timestamps

Optional ROS startup parameter `require_distinct_confirmation_stamp` (default
`false`) requires a valid, strictly increasing scene header timestamp within
a generation/frame stream. This is a stream-level gate: it applies even when
the currently selected source does not need confirmation. The mode prevents
replayed identical-stamp snapshots from accumulating evidence for
`acquisition_confirmations` or `switch_confirmations`.

The existing `require_advancing_scene_stamp` retains its behavior; either
setting enables the strict timestamp check. Invalid (zero, negative or
out-of-range nanosecond) stamps produce `invalid_scene_stamp`; duplicate or
regressed stamps produce `regressed_scene_stamp`. Rejected snapshots do not
advance the accepted watermark or count as confirmation. A fresh generation
or frame begins a new timestamp sequence.

This is opt-in for producers whose scene header stamps represent distinct
observations. A producer that reuses the same stamp for genuinely new scenes
must leave the mode disabled or fix its stamping. This is not cryptographic
anti-replay protection and does not prove the audio itself is fresh.

Four GTests cover replayed acquisition, replayed switching, malformed stamps
and default compatibility. Advisory output only; no robot movement.
