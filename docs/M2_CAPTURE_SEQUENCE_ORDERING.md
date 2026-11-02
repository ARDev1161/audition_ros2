# M2n — capture sequence replay fencing (2026-11-02)

## Problem

A delayed or duplicated `AudioFrame` with the **same**
`capture_session_id` previously looked like a sequence discontinuity to
mono/ODAS ingress. That incorrectly advanced `source_generation`, discarded
valid buffered ODAS hops, and could cancel active source events. Downstream
semantic anti-replay alone cannot prevent this upstream reset.

## Contract

- Validate the complete PCM payload and capture incarnation first.
- Within an unchanged incarnation, reject `sequence_number <= last accepted
  or deliberately dropped sequence` **without modifying** capture state,
  ODAS buffers, source generation, hop order or scene admission.
- Strictly increasing sequences with gaps are still **accepted with a
  generation reset**. Increasing sequences with backward capture timestamps,
  >250 ms unexpected gaps or channel changes retain existing reset semantics.
- A genuinely new `capture_session_id` may restart the sequence at zero.
  Retired capture sessions remain rejected by the bounded session history.
- A valid ODAS chunk dropped due to bounded-buffer overflow still consumes
  its capture sequence. A backend failure resets the ODAS generation, but
  retains the capture sequence high-water mark. Replays cannot revive a
  dropped chunk or create a second reset.
- The disabled spatial route also rejects duplicate/reordered capture
  sequences rather than publishing a new empty snapshot.

`GetIngressStatus.replayed_frames` counts same-session sequence drops on
the ODAS route; it is a subset of `rejected_frames`. The mono route reports
rejections in its throttled ROS warning log and has no separate status
service. A rejected frame cannot mutate the accepted high-water mark.

## Validation

`mono_router_test` and `odas_ingress_test` cover duplicates, delayed
frames with future timestamps, recovery after a real sequence gap, session
restart with sequence reuse, ODAS overflow and backend reset. The existing
Humble `smoke_capture_session_restart.py` ROS graph test now also injects
current-session duplicates/reordering and verifies that no extra PCM or
generation appears.

This is ordering and discontinuity protection, **not** cryptographic
publisher authentication, a global capture/ODAS/semantic transaction,
hardware clock synchronization or a ReSpeaker calibration claim.
