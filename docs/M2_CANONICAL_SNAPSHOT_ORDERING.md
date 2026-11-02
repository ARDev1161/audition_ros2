# M2i canonical /sources publication order

## M2i: canonical source snapshot ordering (2026-11-02)

The public `<namespace>/sources` stream now carries a nonzero, strictly
increasing `AcousticSourceArray.sequence_number` assigned by `acoustic_scene`
on **every** publication, including timer-driven LOST/ENDED and event updates.
The internal spatial input may leave this field at zero. Semantic admission
rejects duplicate, out-of-order and zero-sequence canonical snapshots
atomically, including replays of older ACTIVE states after retirement and
stale snapshots from a previous generation. This is a per-process ordering
token, not a persistent source ID or a capture timestamp.

Do not use `header.stamp` as a transport ordering key: capture and timer
updates may have different timestamp origins, and DDS may reorder messages.
A restarted scene publisher resets its sequence; restart the semantic node
together with it, or implement an explicit scene-session handshake in a
future release. This change does not authenticate arbitrary DDS publishers.

The existing `GetSemanticStatus` service reports `last_source_sequence`.
Humble unit tests cover duplicated, reordered and cross-generation snapshots;
ROS graph smoke injects a zero-sequence canonical snapshot and checks that
live admission remains intact. This is ordering hardening, **not** the
cross-node transactional `ReloadPipeline`.
