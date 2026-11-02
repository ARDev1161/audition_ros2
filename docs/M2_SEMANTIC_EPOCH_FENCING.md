# M2g — authoritative semantic source/epoch admission

The semantic component consumes the **canonical** namespaced `sources`
stream, not raw ODAS track IDs, as the authority for admitting per-source
audio into classifier and ASR workers. This is a safety/correctness boundary
and a prerequisite for a future transactional `ReloadPipeline`, **not** a
runtime reconfiguration service.

## Contract

- Until the first valid canonical snapshot arrives, incoming PCM is dropped.
  Subsequent PCM must match the active generation, sensor `frame_id` and a
  currently ACTIVE or LOST canonical source ID. Unknown and ENDED sources
  cannot start inference. `LOST` remains reversible.
- A higher generation atomically replaces the admission set. Old YAMNet
  sources are forgotten, their temporal detector occurrences retired, and
  delayed classifier results cannot be mapped back to live source names.
  The existing speech runtime resets all ASR worker generations. The
  canonical scene cancels old active events separately.
- A malformed snapshot (duplicate identity, wrong frame, invalid source
  state, regression, or inconsistent generation) changes **no** admission
  state and cannot tear down a working worker.
- A new snapshot may contain one-shot ENDED records from the immediately
  previous generation; these do not become admitted in the new epoch.
- Cross-topic DDS ordering is not guaranteed. For fail-closed admission,
  early audio may be dropped until the corresponding canonical source
  snapshot reaches semantic_perception. There is no fabricated PCM, no
  buffering of unknown audio, and no guarantee of zero first-sample loss.
- All counts are local to the semantic process. An accepted frame means
  admitted to the worker boundary, **not** successful ASR/classification.

## Inspect live status

~~~bash
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
# For an isolated robot:
ros2 service call /robot_a/audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
~~~

The response contains `have_stream`, `source_generation`, `frame_id`,
`admitted_sources`, `accepted_audio_frames`, `dropped_audio_frames`,
`rejected_source_snapshots`, and `stream_resets`. Counters are monotonic
since node startup. A malformed PCM frame also increments dropped frames.

## Validation

`semantic_epoch_gate_test` covers orphan/old-generation PCM, canonical
admission, source retirement, malformed snapshot atomicity, frame changes
and generation transitions containing old ENDED notifications. Humble CI
additionally runs `smoke_semantic_epoch_status.py` on a real replay graph,
checks admission and injects an unknown source packet that must be rejected.
Native ODAS CI builds the same gate and runs its deterministic test.

This does **not** implement transactional reconfiguration, measured ReSpeaker
geometry, model accuracy, complete DDS reordering recovery, or an ASR latency
benchmark. `GetCapabilities.runtime_reconfiguration_supported` remains
`false`.
