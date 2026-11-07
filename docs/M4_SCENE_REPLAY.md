# M4 canonical scene session/replay fence

The optional `pose_evidence_node` now validates canonical
`AcousticSourceArray.scene_session_id` (32 lowercase hex characters),
strictly increasing `sequence_number`, and increasing scene timestamps.
Duplicated, reordered and previously retired sessions do not increase
timestamp-match counters. New capture incarnations discard all buffered
pose samples and require fresh pose evidence before matching bearings.

Clock rollback clears the pose buffer and retires the active scene;
the previous incarnation cannot be silently resumed. This behavior is
deliberately fail closed and requires the canonical producer to issue a
fresh session ID after clock epoch changes.

The `SceneAdmission` helper retains up to 16 old session IDs by default,
so it cannot guarantee replay prevention across arbitrary process restarts
or indefinitely old history. Producer session UUID uniqueness, durable
authentication and timestamp/provenance validation are out of scope.

Diagnostic counters:
- `replayed_scenes`: invalid, repeated, out-of-order or retired snapshots
- `session_restarts`: accepted new incarnation with pose cache revocation
- `clock_resets`: ROS time rollback with state invalidation

Test with `python3 -m unittest discover -s audition_fusion/test -p 'test_scene_admission.py' -v`.
The existing DDS smoke also checks replay and session restart, but no
physical position or navigation output is enabled.
