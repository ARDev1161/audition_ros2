# M2j: canonical acoustic scene incarnation and semantic recovery


## M2j: canonical scene incarnation handshake (2026-11-02)

Each `acoustic_scene` process now generates a fresh 128-bit hexadecimal
`scene_session_id` and attaches it to every canonical `/sources` snapshot.
Internal spatial producers leave it empty. Semantic admission validates the
session ID, preserves per-session strict sequence ordering, and recognizes a
**previously unseen** scene incarnation even when its sequence or upstream
source generation restarts at 1. A session transition resets YAMNet and
Sherpa/Whisper source workers, invalidates previous admission, and fences
queued same-generation PCM until after the new canonical snapshot timestamp.
Previously accepted session IDs are retained (up to 64 per semantic process)
so a delayed old snapshot cannot restore a retired source. The 65th distinct
session fails closed; restart the semantic process. Status now reports
`scene_session_id` and `scene_session_resets`.

This repairs **scene-only process restart reconciliation**, not an atomic
capture/ODAS/scene/model reconfiguration transaction. A new scene process
must publish at least one valid canonical snapshot before semantic admission
can resume. The session ID is an ordering discriminator, **not DDS publisher
authentication**; arbitrary untrusted publishers must be isolated by ROS 2
security. PCM has no scene incarnation field, so independently restarted
capture/spatial publishers and clock resets still require coordinated
generation fencing or a complete pipeline restart. Model accuracy, HIL
calibration and a full cross-node `ReloadPipeline` remain unverified.


## Wire contract

- `AcousticSourceArray.scene_session_id`: 32 lowercase hexadecimal chars on
  canonical `/sources`; empty on `internal/spatial_sources`.
- `sequence_number` is strictly increasing **within** one session.
- `source_generation` remains spatial ownership; a new scene session does not
  fabricate a spatial generation.
- A new scene session invalidates all semantic workers, even when the
  spatial generation and frame name are unchanged.
- Previously accepted scene sessions cannot reappear; 64 distinct sessions
  per semantic lifetime is the fail-closed bound.
- `GetSemanticStatus` exposes the session and accepted restart count.

## Operational checks

```bash
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
ros2 topic echo /audition/sources --once
```

After restarting `acoustic_scene`, wait for its next valid canonical
`/sources` publication. The semantic status should show a different session
and one more `scene_session_resets`. Check that admission resumes only after
new-source observation; don't treat the scene UUID as security identity.

## Verification and limitations

The unit tests cover a scene sequence rollback (300 -> 1), generation
rollback (50 -> 1), same-generation scene replacement, old-session replay,
malformed IDs, stale PCM, and status counters. Humble ROS graph smoke checks
the live scene session and rejects an empty-session injected snapshot.
Native ODAS CI compiles the same message contract.

This handshake cannot by itself fence a separately restarted spatial process
that reuses source generation and timestamp. Use coordinated process restart
for that case until a full transaction protocol carries a shared incarnation
across every audio and semantic message.
