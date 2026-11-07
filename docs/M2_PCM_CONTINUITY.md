# M2k — semantic PCM continuity and anti-replay admission

**Scope:** ROS 2 Humble, optional semantic worker, backend-independent.

## Invariants

Canonical `/sources` authorizes source IDs and generations. For each admitted
source, semantic admission also remembers the last **accepted** source-audio
sequence and capture timestamp. A non-increasing sequence or regressing
timestamp is dropped before libaudition inference. The first sequence can be
zero. Gaps are allowed; no missing samples are fabricated.

Cursors are bounded by the admitted source set (maximum 256). Refresh snapshots
preserve them. Retirement prunes them; generation changes, new scene sessions
and successful worker reloads clear them. Rejected packets and snapshots never
update cursors. Source IDs must not be reused in one generation/session.
Sequence wrap is not inferred as valid rollover; a new generation is required.

## Diagnostics

```bash
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
```

`out_of_order_audio_frames` is cumulative since process startup and a subset
of `dropped_audio_frames`. It counts duplicates, backwards sequence and
capture-time regression for admitted sources. Unknown sources, invalid PCM,
old generations and reload-fenced PCM count only as general drops.

## Verification and limitations

`semantic_epoch_gate_test` covers duplicate/reordered PCM, clock regression,
independent sources, retirement, reload and scene restart. Humble ROS graph
smoke replays an actual mono PCM packet and checks the counter. No ML model
is needed. This does not authenticate DDS publishers or stop forged packets
with new sequence/timestamps. A capture publisher restarting without an
upstream generation/session transition remains outside this contract.
Multi-node transactional `ReloadPipeline` and physical ReSpeaker tests
are still outstanding.
