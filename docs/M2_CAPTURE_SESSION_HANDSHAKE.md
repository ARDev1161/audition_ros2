# M2l — capture-process incarnation fencing

**Status:** ROS 2 Humble integration; model-independent, no physical array validation.

## Contract

Every `audition_msgs/msg/AudioFrame` carries a required 32-character lowercase
hexadecimal `capture_session_id`. WAV capture creates it per node instance;
ALSA capture creates it per process and **keeps it across XRUN recovery,
watchdog reconnect and transient device failure**. The identifier is not a
secret, DDS publisher authentication or a hardware serial number.

The mono and ODAS ingress paths validate PCM and the incarnation before
changing stream state. A previously unseen session, even with a perfectly
continuous sequence number and timestamp, increments the source generation.
Mono reinitializes `first_seen`; ODAS discards partial hop staging, resets
the native backend and clears active tracks before accepting new PCM.
The disabled spatial route also increments its empty-stream generation.

Once a new session is accepted, delayed packets from retired capture sessions
are rejected without changing the generation, clock cursor or buffered PCM.
Up to 64 retired sessions are remembered per consumer process; further
incarnation changes fail closed until the consumer is restarted. A malformed
or absent ID is rejected; third-party capture publishers must upgrade to
the new message contract. A sequence gap within one ALSA session still fences
XRUN and reconnect continuity through the existing logic.

The session is a *process-incarnation discriminator*, not a globally ordered
counter. If an untrusted publisher can forge a fresh valid ID, it can still
inject data; use ROS 2/DDS security and appropriate graph isolation.

## Tests

- `mono_router_test`: continuous-sequence process replacement, first-seen
  reset, old-session replay, malformed ID, bounded history.
- `odas_ingress_test`: pending-hop flush on capture replacement, hop clock
  and generation reset, rejected replay with unchanged pending data.
- `test_generated_messages.py`: raw capture session serialization.
- `smoke_capture_session_restart.py`: a capture-free mono+scene graph
  fixture injects sessions A and B with continuous sequence/timestamps,
  rejects delayed A and malformed IDs, and confirms B can continue.

The ROS graph fixture is exercised by Humble CI; ODAS ingress regressions
are also built by the native ODAS workflow. This is not an atomic
cross-node `ReloadPipeline`, and does not solve independent *spatial process*
restart, model reconfiguration, physical ReSpeaker calibration or accuracy.

## Operational notes

A new capture process may reuse the same ALSA device and `frame_id`, but
its session ID changes and downstream sources/events are fenced through
the generation contract. Keep the spatial and scene processes alive during
a capture-only restart; do not reuse a retired session ID in a new process.
If the 64-session bound is reached, restart the downstream consumer rather
than disabling the fence.
