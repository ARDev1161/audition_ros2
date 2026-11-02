# M2m — spatial producer incarnation handshake

## M2m: independent spatial-process restart fencing (2026-11-02)

Each mono or ODAS spatial process now generates a 128-bit
`spatial_session_id`, stamps internal source snapshots and separated PCM,
and assigns a strictly increasing per-process snapshot sequence.
`acoustic_scene` validates both before changing canonical source state,
rejects duplicated/reordered snapshots and replays from retired producer
incarnations (bounded history: 64). A new producer may reset its
`source_generation` to 1; prior source admissions and late-transcript grace
are invalidated atomically.

On spatial incarnation change, the scene rotates its `scene_session_id`,
cancels outstanding events with
`termination_reason=spatial_process_restart`, and republishes the new
canonical snapshot with the spatial ID. Semantic admission resets workers and
requires `SourceAudioFrame.spatial_session_id` to match the canonical
incarnation. Old PCM is rejected even with a future timestamp and reused
source generation/ID. Existing custom spatial publishers must supply a
nonzero strictly increasing internal sequence and valid 32-character
lowercase hexadecimal spatial ID; legacy empty/zero fields fail closed.

These IDs are ordering discriminators, **not DDS authentication**. This is
not a multi-node transactional `ReloadPipeline` or ReSpeaker calibration.

## Validation

`source_lifecycle_test` covers independent spatial restart with generation
reuse, old-session replay, duplicate internal sequence, malformed snapshot
rollback and transcript grace invalidation. `semantic_epoch_gate_test`
checks that a future-dated old PCM packet cannot cross a spatial restart.
The Humble scene graph fixture stamps its synthetic producer incarnation and
monotonic internal sequence. Native ODAS integration CI builds and exercises
the same spatial producer code. Physical hardware testing remains open.
