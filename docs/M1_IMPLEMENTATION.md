# M1 capture and scene scaffold — status

This implementation is **not** a complete spatial perception stack and is not
ready for a release gate.

## What works in this branch

- `audition_capture`: C++17 RIFF/WAV reader for uncompressed PCM16 or normalized
  IEEE float32, with bounded chunks and explicit channel metadata.
- `wav_capture` publishes timestamped `audition_msgs/msg/AudioFrame` at the
  file's nominal playback rate. Source timestamp means ROS replay time, not
  the original recording time (a generic WAV has no ROS clock identity).
- `spatial_perception`: with `backend=disabled`, validates PCM and publishes **empty** spatial source snapshots; with an ODAS-enabled libaudition build, it converts native spatial tracks and source-separated audio into ROS messages. Both modes require testing.
- `acoustic_scene`: republishes the empty spatial snapshots as canonical
  `AcousticScene` and `AcousticSourceArray` messages.
- `semantic_perception`: separately buildable and subscribes to
  `SourceAudioFrame`, but no classifier/ASR inference is available.
- Launch uses an isolated spatial process to reserve a safe boundary for a
  future ODAS backend; it does not start semantic inference.

## Build and run on ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

ros2 launch audition_bringup m1_wav_pipeline.launch.py wav_path:=/absolute/path/to/file.wav
# In another sourced shell:
ros2 topic echo /audition/scene audition_msgs/msg/AcousticScene --once
ros2 topic hz /audition/audio/raw
```

`wav_path` is mandatory. Supported WAV file layouts are interleaved 16-bit
signed integer and IEEE float32 little-endian RIFF, with 8–192 kHz sample
rate and at most 32 channels. Compressed/EXTENSIBLE WAV is not supported yet.
The default message chunk is 160 *frames*, not bytes. The final chunk may be
shorter. `loop=false` causes publication to stop at EOF without exiting the
node. The input ROS timestamp is assigned at replay callback time, not copied
from any external hardware clock.

**Important:** absence of sources is expected while the spatial backend is
disabled. The sample launch tests message transport and does not verify
DOA, separation, source IDs or scene lifecycle. Never interpret this scaffold
as a functional hearing or navigation pipeline.



## Native libaudition ODAS integration (experimental)

The `audition_perception` package can now be built with
`-DAUDITION_ROS2_WITH_ODAS=ON`. This is **not** the default. It consumes a
**previously installed** libaudition CMake package built with
`-DLIBAUDITION_WITH_ODAS=ON` (including its native ODAS dependency).
See the [pinned native integration workflow](../.github/workflows/odas-integration.yml)
for the exact tested upstream commit and build flags.

Example on a machine with a real calibrated array:

```bash
# Build/install libaudition with ODAS first, then:
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="/path/to/libaudition/install:$CMAKE_PREFIX_PATH"
colcon build --symlink-install --cmake-args -DAUDITION_ROS2_WITH_ODAS=ON
source install/setup.bash
ros2 launch audition_bringup m1_wav_pipeline.launch.py \
  wav_path:=/absolute/path/to/six-channel-recording.wav \
  spatial_backend:=odas spatial_config:=/absolute/path/to/calibrated-array.yaml
```

`spatial_config` must define `/**/spatial_perception.ros__parameters` with:

- `geometry_calibrated: true`, `microphone_frame` matching the input frame;
- `input_channel_indices` (zero-based capture channels, e.g. 1–4 for a
  six-channel ReSpeaker file);
- `microphone_positions_m` as a flat array of x,y,z coordinates in metres
  in the same order as the input channel indices;
- `sample_rate_hz`, `odas_hop_frames`, `odas_frame_size`, `max_tracks`,
  `enable_separation` and optional `respeaker_angular_profile`.

The production ReSpeaker example deliberately keeps
`geometry_calibrated: false` and no coordinates. It must not be enabled as
a working ODAS profile without physical calibration.

The optional CI-only configuration
[`odas_synthetic_ci.yaml`](../audition_bringup/config/odas_synthetic_ci.yaml)
uses **synthetic** four-microphone coordinates from the upstream offline test,
not physical measurements.

When the native backend is enabled, `spatial_perception` converts source
bearings, real ODAS track activity, and separated mono frames into ROS
messages. `AcousticSource.source_id` identifies a local source hypothesis
for one tracker lifetime; its `spatial_track_id` is the short-lived
libaudition/ODAS track. IDs are not speaker identities and are **not** claimed
to survive re-acquisition of a source. `source_generation` increments on
detected audio discontinuity/reset. Range, world position and angular variance
are left unavailable unless truly measured.

Frames are validated before backend processing; the runtime accepts
interleaved or planar input but supplies interleaved hops to ODAS. It avoids
inventing missing samples or inserting a phase-altering speech frontend before
spatial localization. Source results are published at the original capture
frame's timestamp. Capture callbacks share the isolated spatial component's
process; **backend CPU and long tail latency are not yet benchmarked**.

This first native integration is a bridge, not an M1 acceptance claim.
Remaining: bounded time-aware buffering/backpressure instrumentation, lifecycle
and reconnection handling, source retirement states, geometric calibration,
real-world DOA error testing and optional model inference. An ODAS-enabled
binary still uses an isolated process because libodas may call `exit()`
inside a fatal error path.


## What must happen next

1. Add deterministic audio frame validation tests and a source/clock generation
   reset contract.
2. Complete measured microphone geometry and physically validate the now-wired native
   libaudition ODAS adapter, including real-device hop/rate calibration.
3. Implement bounded ingestion and source ID retirement, real separated frames,
   `AcousticEvent` assembly, and explicit stream disconnect/restart semantics.
4. Build a ReSpeaker HIL corpus to measure real-room angular errors, ID switches,
   TTL expiration and multi-source coverage.
5. Add the optional per-source semantic runtime with real models and event
   boundaries after spatial ownership contracts are stable.

See [ROS 2 node architecture](ROS2_ARCHITECTURE.md) and [engineering roadmap](ROADMAP.md).


## Mono capture to semantic route (M2 no-ODAS increment, 2026-11-02)

`spatial_backend:=mono` in M1/M2 WAV or ALSA launch selects a **separate**
`MonoSourceNode` component instead of `SpatialPerceptionNode`. It accepts
only one-channel normalized PCM at exactly 16 kHz (max 4096 frames per
message). It forwards the *actual* capture PCM to
`/audition/internal/source_audio` and publishes a single capture-stream
source via `/audition/internal/spatial_sources`, admitted by the same
canonical scene lifecycle as ODAS. It never fabricates direction, track ID,
speaker ID, distance, position or calibrated activity.

The source is a capture stream, **not** a detected speaker. The epoch
increments after a capture sequence gap, timestamp regression, timestamp
discontinuity greater than 250 ms, or `frame_id` change. A malformed frame
is rejected without changing the epoch. On capture silence/stall, the
canonical scene's existing LOST/ENDED TTL applies; there is no invented
voice activity. DDS topic ordering is not guaranteed, but each valid frame
refreshes source admission before emitting mono PCM.

```bash
ros2 launch audition_bringup m2_wav_pipeline.launch.py \
  wav_path:=/absolute/path/to/mono-16k.wav spatial_backend:=mono \
  semantic_config:=/absolute/path/to/semantic_m2.yaml
# ALSA: channel_count:=1 sample_rate_hz:=16000 spatial_backend:=mono
```

`classifier_backend: disabled` and `speech_enabled: false` remain safe
defaults. Enabling YAMNet or Sherpa/Whisper requires the separately installed
native targets and real model files. The route has deterministic unit tests
for PCM, generations, invalid frames and non-spatial evidence, plus a ROS 2
graph smoke. This is not evidence of model accuracy, multi-speaker separation,
microphone calibration or hardware latency.


## M2c startup profile entry point

For an opt-in validated mono/ODAS startup profile use
m2_profile.launch.py with profile:=mono_speech or profile:=replay_ci,
plus wav_path:=/absolute/path/to/mono-16k.wav. The original M1/M2 launches
remain supported. Profile validation, model paths, immutable semantic
parameters and the typed capability service are documented in
[M2c configuration](M2_PROFILE_CONFIGURATION.md). Physical ReSpeaker
calibration and model accuracy are still unverified.

## M2d namespace isolation


## M2d: multi-instance ROS namespace isolation (implemented 2026-11-02)

All M1/M2 WAV/ALSA launch files and M2c profiles accept
`audition_namespace` (default `audition`). M2c accepts `frame_id`
override and validates relative namespaces before launching. A non-default
namespace cannot reuse the built-in `microphone_array` frame; provide a
unique frame for each robot/sensor. For ODAS, the measured geometry YAML
must specify the same `microphone_frame`.

~~~bash
ros2 launch audition_bringup m2_profile.launch.py \
  profile:=replay_ci wav_path:=/data/mic_a.wav \
  audition_namespace:=robot_a/audition frame_id:=robot_a_mic
ros2 launch audition_bringup m2_profile.launch.py \
  profile:=replay_ci wav_path:=/data/mic_b.wav \
  audition_namespace:=robot_b/audition frame_id:=robot_b_mic
ros2 service call /robot_a/audition/semantic_perception/get_capabilities \
  audition_msgs/srv/GetCapabilities "{}"
ros2 topic echo /robot_b/audition/scene
~~~

Capture, spatial/mono, scene and semantic components share the selected
namespace. All public and internal topics and the typed capability service
are scoped to it; no global `/audition` topic is intentionally created.
Source IDs and generations are local to one namespace. Cross-robot consumers
must key observations by (namespace, source_id, generation), not source_id
alone. Legacy M1/M2 launch files accept these arguments but do not perform
M2c preflight; operators must supply distinct TF frames and ALSA devices.

Humble CI starts two simultaneous mono graphs on one DDS domain with 440Hz
and 880Hz WAVs, validates PCM, canonical scene frames and independent
capability services, and rejects unexpected global /audition topics.
This is graph isolation, not physical ODAS calibration, a fleet registry
or transactional model reload. Runtime semantic parameters remain immutable.

## M2e: bounded ODAS ingress, generation fencing and observability

The ODAS adapter stages at most eight hops of normalized multichannel PCM,
processes fixed-size timestamped hops, and rejects malformed frames without
mutating buffered audio. Capture sequence gaps, backward time, >250 ms
unexpected forward gaps and channel-count changes reset the ODAS session and
increment the spatial source generation. Overflow drops the **whole** chunk
and resets the session; ODAS exceptions also fence the stream. An empty
new-generation snapshot is published before any replacement tracks.

Inspect `/audition/spatial_perception/get_ingress_status` with
`audition_msgs/srv/GetIngressStatus`; counters are monotonic since startup. `staged_hops` counts ingress-produced
hops; `processed_hops` counts only successful native ODAS process calls.
With a custom namespace, replace `/audition` accordingly. The helper has
deterministic unit tests in the default Humble build; the native ODAS workflow
tests the service while processing a synthetic multichannel WAV. No physical
array calibration or real-time performance is implied. Transactional
reconfiguration and HIL metrics remain outstanding.

## M2f: live ALSA watchdog, reconnect and capture diagnostics (2026-11-02)

Live ALSA capture now treats an unexpected read error or a sustained absence of
PCM as a transport failure. The worker closes the failed handle and retries the
**same configured device/rate/channel layout**, with bounded exponential
backoff (100–2000 ms). `stall_timeout_ms` defaults to 2000 ms (500–60000
allowed); `max_reconnect_attempts:=0` means unlimited retries, or a positive
cap marks a terminal failure. Initial device configuration still fails fast.

Every successful reconnect and delivered XRUN recovery creates a sequence
gap on the next valid `AudioFrame`, fencing downstream mono/ODAS source
generations without synthesizing audio. Capture time is still an estimate from
read completion, **not** a hardware timestamp. Shutdown interrupts retry sleep
within approximately 50 ms; no model worker runs in the capture thread.

Inspect the namespaced read-only service:

~~~bash
ros2 service call /audition/audio_capture/get_capture_status \\
  audition_msgs/srv/GetCaptureStatus "{}"
# With audition_namespace:=robot_a/audition, use
# /robot_a/audition/audio_capture/get_capture_status
~~~

It exposes connection/terminal state, published PCM count, last sequence,
delivered XRUN recovery episodes, fatal/stall events, reconnect attempts,
success/failure counts, retry delay and last error. These are process-local
counters, **not** evidence of successful physical unplug/replug, clock
synchronization or calibrated ReSpeaker accuracy. Unit tests inject failures
into the continuity/backoff state machine; Humble CI checks the service on
the virtual ALSA `null` device. Physical HIL remains required.

## M2g semantic source admission and generation reset

The semantic node now accepts source PCM only after the canonical scene
has admitted the matching source ID, frame and generation. An empty snapshot,
source retirement or generation change invalidates that admission. Stale
classifier jobs are forgotten and speech workers reset on epoch changes.
A namespaced `GetSemanticStatus` service reports accepted/dropped PCM,
active generation and source count, malformed snapshots and reset counts.
See [M2g details](M2_SEMANTIC_EPOCH_FENCING.md). DDS cross-topic ordering
may drop the earliest PCM chunk; no audio is invented to mask that gap.
The transactional `ReloadPipeline` remains unimplemented.

## M2h: transactional semantic worker restart (2026-11-02)

The namespaced `audition_msgs/srv/ReloadSemanticWorkers` service at
`<namespace>/semantic_perception/reload_workers` supports an explicit
**same-configuration** restart of classifier and speech workers. The
`dry_run` option builds candidate model workers without replacing live
instances. On commit, both candidates are constructed successfully before
the existing classifier/ASR workers are swapped. A model-load failure returns
`success=false` and leaves the old workers and canonical admission untouched.

After a successful commit, the semantic gate drops queued PCM and old
source snapshots until a strictly newer canonical source snapshot arrives.
Within the same upstream generation, PCM at or before that snapshot's capture
timestamp remains rejected. The old worker instances are destroyed without
draining their inference results to ROS, so stale classifications and ASR cannot
be published from the replaced workers. The upstream `source_generation`
is **not** incremented by this semantic-only operation. The read-only
`GetSemanticStatus` reports the local `reload_epoch`, successes, failures,
and whether a new source snapshot is awaited.

This is a complete **semantic worker restart transaction**, not a
`ReloadPipeline` across capture, ODAS and scene. It does not accept changed
models, paths, geometry or sample rate; ROS parameters remain immutable.
Model construction may temporarily duplicate model memory and block the
semantic executor. For hardware or model configuration changes, stop and
relaunch the validated profile. Full cross-node two-phase reload, upstream
generation bump, resource budgeting and failure-injection HIL remain open.

~~~bash
ros2 service call /audition/semantic_perception/reload_workers \
  audition_msgs/srv/ReloadSemanticWorkers "{dry_run: true}"
ros2 service call /audition/semantic_perception/reload_workers \
  audition_msgs/srv/ReloadSemanticWorkers "{dry_run: false}"
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
~~~


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


## M2k: per-source PCM anti-replay and capture-order fencing (2026-11-02)

The semantic admission gate retains a bounded (sequence, capture-time)
high-water mark per admitted source and rejects duplicate/reordered
`SourceAudioFrame.sequence_number` or backwards capture timestamps before
YAMNet/VAD/ASR. Sequence gaps are permitted; no audio is synthesized.
`GetSemanticStatus.out_of_order_audio_frames` counts such drops and is a
subset of `dropped_audio_frames`. Accepted source refreshes preserve cursors;
retirement prunes them; new generations/sessions and successful worker
reloads clear them. Rejected packets cannot move the high-water mark.

Humble tests exercise duplicate/reordered PCM, timestamp regressions,
source independence, retirement, reload and scene restarts. ROS graph smoke
replays a real mono source packet and verifies the diagnostic counter.
This is transport ordering protection, not cryptographic replay prevention,
publisher authentication or full multi-node `ReloadPipeline`. See
[M2k PCM continuity](M2_PCM_CONTINUITY.md).

## M2l: capture-process restart handshake (2026-11-02)

Every raw `AudioFrame` now requires a per-process 32-character lowercase
hexadecimal `capture_session_id`. WAV replay creates a new ID for each
capture node; ALSA preserves the same ID across XRUN and reconnect.
Mono/ODAS ingress validates the ID and resets its source generation when a
new capture process appears, **even if sequence numbers and timestamps
look continuous**. ODAS discards pending hop audio and resets native tracks;
old-session and malformed frames are rejected before modifying state.
The disabled spatial route also fences its empty snapshot generation.

Consumers retain up to 64 retired session IDs; subsequent transitions
fail closed and require a consumer restart. This is not DDS authentication.
Humble tests cover the mono+scene ROS graph and model-independent ODAS
ingress regressions; physical unplug/replug is not certified.
See [M2l capture-session contract](M2_CAPTURE_SESSION_HANDSHAKE.md).
Independent spatial-process restart and transactional `ReloadPipeline`
remain open.

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


## M2n: capture sequence replay fencing (2026-11-02)

Mono and ODAS ingress now reject same-capture-session duplicate or
out-of-order raw `AudioFrame.sequence_number` **before** applying gap/clock
resets or modifying buffered PCM. Genuine forward gaps still reset source
generations. ODAS retains its capture high-water mark across buffer overflow
and backend reset, and counts such drops in
`GetIngressStatus.replayed_frames` (subset of `rejected_frames`). A new
capture incarnation may start its sequence again at zero; retired sessions
remain rejected. The disabled spatial route applies the same replay check.

Humble C++ tests and the capture-session ROS graph regression cover replay,
reordering, overflow, restart and recovery. See
[M2n capture sequence ordering](M2_CAPTURE_SEQUENCE_ORDERING.md).
This does not implement cross-node `ReloadPipeline`, DDS authentication or
physical ReSpeaker acceptance.


## M2o: supervised process-level profile switch (2026-11-02)

The optional [M2o supervisor](M2_PIPELINE_SUPERVISOR.md) owns the full launch
process group, validates a candidate profile before shutdown, waits for old
DDS graph removal, starts a candidate and checks capture/spatial/scene/semantic
node discovery. If readiness fails it attempts a bounded rollback to the old
profile and reports rollback success separately. This **interrupts capture**;
it is not a zero-downtime cross-node two-phase commit. Session IDs prevent
old audio from being accepted after restart. Hardware calibration and latency
remain unverified.

## M2p: supervisor failure containment and manual recovery (2026-11-02)

The optional supervisor detects launch-parent death, cleans up surviving
process-group children and reports offline faults rather than silently
restarting. A failed cleanup retains ownership and blocks another launch.
Existing namespaces are rejected before startup. The typed GetPipelineStatus
service reports liveness, ownership, active/last-good profile, epoch and fault.
Explicit ReloadPipeline recovery requires cleanup and graph disappearance.
See [M2 supervisor](M2_PIPELINE_SUPERVISOR.md). Hardware/model health and
zero-downtime reload are not guaranteed.

## M2q: continuous supervisor graph watchdog (2026-11-02)

After initial readiness, the opt-in supervisor checks the four ROS pipeline
nodes and the semantic capability service at a one-second interval. Five
consecutive failed graph samples (including discovery exceptions) stop the
owned process group and require explicit recovery. Healthy samples reset the
counter. The typed pipeline status includes last-sampled `graph_healthy`
and `graph_miss_count`; launch-parent liveness alone no longer masks
sustained ROS graph loss. The watchdog cannot prove actual audio delivery,
model quality or microphone health. See [M2 supervisor](M2_PIPELINE_SUPERVISOR.md).

## M2r: supervised raw PCM progress (2026-11-02)

The optional supervisor subscribes to `audio/raw` and exposes a typed PCM
heartbeat in `GetPipelineStatus`. Valid session/sequence-ordered PCM advances
the counter; duplicates and stale sessions do not. Live ALSA profiles fault
and require explicit recovery after a sustained no-PCM condition, whereas
finite WAV playback is not faulted after EOF. See
[M2r PCM watchdog](M2_PIPELINE_SUPERVISOR.md). This checks transport progress,
not non-silence, ASR output or calibrated ReSpeaker performance.

## M2s: supervised downstream source PCM progress (2026-11-02)

The optional supervisor observes `internal/source_audio` with spatial
session, generation and sequence validation. `GetPipelineStatus` exposes
source PCM counters, freshness and debounce. Live ALSA + mono is monitored
for sustained missing source PCM **only when raw capture PCM is healthy**;
ODAS and finite WAV replay remain observation-only. See
[M2 supervisor](M2_PIPELINE_SUPERVISOR.md). This checks spatial transport,
not semantic model completion or physical separation quality.


## M2t: semantic ingress acknowledgement and supervised downstream stall (2026-11-02)

The semantic node publishes `internal/semantic_progress` as
`audition_msgs/msg/SemanticProgress` **only after** canonical source admission
and successful submission to both configured semantic workers. Each small
acknowledgement carries the spatial session, source ID, generation, PCM
sequence and frame count. It does not copy PCM and does not report completed
YAMNet classification, VAD or ASR inference.

The optional supervisor checks these acknowledgements with per-source
monotonic cursors, retired spatial-session fencing and bounded memory. It
exposes `semantic_watchdog_enabled`, `semantic_healthy`,
`semantic_miss_count`, `accepted_semantic_frames` and
`last_semantic_age_ms` in `GetPipelineStatus`. For live ALSA+mono only,
after 10 seconds without an initial acknowledgement or 5 seconds of silence
in the acknowledgement stream, five consecutive 1-second samples trigger
owned-process containment **only if both raw and spatial source PCM are
healthy**. An upstream failure is attributed to the earlier stage; finite
WAV and ODAS remain diagnostic-only because their source activity may stop
legitimately. Manual recovery and session retirement follow M2p/M2s.

The acknowledgement is a **semantic ingress/worker-submission** heartbeat,
not proof of asynchronous inference completion or model accuracy. Disabled
model backends still acknowledge valid admitted PCM, so this watchdog cannot
measure classifier or ASR throughput. It also does not authenticate DDS
publishers. CI tests invalid/replayed acknowledgements, spatial-session
restarts, debounce, upstream fault attribution, profile switching, typed
serialization and a live virtual ALSA ROS graph.


## M2u: asynchronous inference completion telemetry (2026-11-03)

`semantic_perception/get_semantic_status` now distinguishes **admitted PCM**
from **completed inference**. It reports enabled YAMNet and speech workers,
completed classifier windows, discarded stale classifier results, completed
ASR segments (including empty-text results), empty ASR segments, discarded
stale ASR results and caught inference polling failures. Separate
monotonic-clock ages report time since the last accepted classifier window
or ASR segment. Age is zero before the first completion. Counters persist
for the process lifetime, including same-process worker reloads.

This is diagnostic-only. **Missing inference completions do not trigger a
restart**: YAMNet needs a complete window, ASR needs VAD-triggered speech,
and silence or disabled models may legitimately produce no results. No
libaudition worker queue depth, per-job runtime, model accuracy or latency
bound is claimed. Native CI compiles both optional adapter paths; model-
independent tests validate counter/age semantics and the Humble ROS graph
verifies disabled workers never claim completed inference.
See [M2u completion telemetry](M2_INFERENCE_COMPLETION_TELEMETRY.md).

## M2v: native semantic workload snapshots (2026-11-03)

`GetSemanticStatus` exposes YAMNet per-source worker states and last
successful inference/queue-wait timing, plus native ASR queued jobs,
buffered results, retirement, drops and evictions. These are diagnostic
snapshots, not job deadlines or evidence of model accuracy. Disabled
workers return zero/false. See
[M2v workload telemetry](M2_INFERENCE_WORKLOAD_TELEMETRY.md).

## M2w: sampled classifier state residency (2026-11-03)

The semantic status service now reports the longest observed residence in native
YAMNet `Queued` and `Inferencing` states and counts sources exceeding 10s and
30s respectively. State changes reset their individual clocks; repeated
observations do not. Retired sources, worker reload and generation resets
clear the observations. Waiting/collecting/silence are never treated as stalls.
This is **diagnostic only**: the libaudition API reports a source state, not
an individual job identifier or native running-job start timestamp. A state
may cover multiple jobs. ASR currently exposes queued segment counts but no
running-job age, so no ASR deadline or automatic restart is claimed. See
[M2w state-residency diagnostics](M2_CLASSIFIER_STATE_DEADLINES.md).

## M2x: autonomous workload sampling (2026-11-03)

A 1 Hz timer samples native YAMNet source states independently of status-service callers. The status service returns cached workload and exposes sample freshness, success and failure counters. Reload resets these values. No automatic restart or individual inference-job deadline is claimed. See [M2x](M2_WORKLOAD_SAMPLING.md).

## M2y: source-incarnation and failed-sample fencing (2026-11-03)

M2x sampling now keys state residency by the native worker's unique source ID,
not the externally reusable source name. A retired/re-created source therefore
cannot inherit the prior generation's queued/inferencing age. Retirement,
stream reset and native sampling exceptions invalidate cached workload and
reset state-residency clocks. `classifier_workload_sample_valid` distinguishes
valid current-incarnation samples from stale last-success history; the existing
`classifier_has_workload_sample` still means at least one historical success.
On invalid samples, overdue counters are suppressed until the next successful
sample. This avoids false watchdog diagnostics during backend errors or source
turnover, but remains observation-only, not native per-job timing. See
[M2y sample fencing](M2_SAMPLE_FENCING.md).

## M2z: configurable, validated YAMNet diagnostic sampling (2026-11-03)

The classifier observer now accepts startup-only ROS parameters
`classifier_workload_sample_period_ms` (default 1000; range 200–10000),
`classifier_queued_threshold_ms` (default 10000; range 1000–600000) and
`classifier_inference_threshold_ms` (default 30000; range 1000–600000).
Each threshold must be at least the sample period. Invalid profiles fail
startup before workers are created; live parameter changes remain rejected.
The effective values are returned by `GetSemanticStatus`, including with a
disabled classifier. The diagnostics are source-state observations, not native
job timeouts or automatic worker restarts. See [M2z](M2_OBSERVER_CONFIGURATION.md).

## M3a: independently sampled ASR backlog telemetry (2026-11-03)

The semantic node now samples the native speech runtime's queued segments,
result mailbox and worker-error flag on its existing workload timer, rather
than synchronously calling the native ASR statistics API from every status
service request. The status response includes sample validity, historical
sample/failure counts, last-success age and high-water marks for queued
segments and buffered results. Sampling failures invalidate the current
snapshot; worker reload and stream generation reset clear observer history.
YAMNet and ASR sampling are isolated so an exception in one does not suppress
the other's sample. A queue high-water mark is **not** evidence of a hung ASR
job; silent input and VAD gating remain legitimate. See
[M3a ASR backlog telemetry](M3_ASR_BACKLOG_TELEMETRY.md).

## M3b: sustained ASR queue and mailbox occupancy (2026-11-03)

The independent ASR observer now tracks how long the pending queue and result
mailbox have remained continuously nonempty across valid samples. It reports
`asr_queue_nonempty_age_ms`, `asr_mailbox_nonempty_age_ms`, and a diagnostic
`asr_sustained_queue_pressure` flag when at least four queued segments are
currently present and the queue has stayed nonempty for at least ten seconds.
A drained queue resets only its own age; a failed sample, worker reload or
stream reset invalidates both clocks. No speech, zero pending work and
sampling failures never produce a pressure alert. This is not an inference
job deadline or automatic recovery. See [M3b](M3_ASR_QUEUE_PRESSURE.md).

### M3b CI correction (2026-11-03)

The queue/mailbox residency tracker now explicitly starts a new clock on an
empty-to-nonempty transition, not only on first sample or when becoming empty.
This fixes the CI regression where a drained queue inherited its previous
residency. Tests cover both queue and mailbox drain/reappearance.

## M3c: freshness-fenced ASR pressure evidence (2026-11-03)

A single ASR workload observation no longer becomes a pressure alarm merely
because wall time passes. The M3b pressure flag now requires at least two
consecutive nonempty samples and a fresh native observation (last sample
within three seconds), in addition to the existing >=4 pending segments and
>=10 seconds of observed nonempty queue. The status service exposes
`asr_workload_sample_fresh` and `asr_contiguous_queue_samples`.
Drain, sampling failure and worker/stream reset clear consecutive evidence;
a stopped timer cannot manufacture a new pressure alarm. See
[M3c freshness fencing](M3_ASR_FRESHNESS_FENCING.md).

## M3d: ASR sampling-gap continuity fence (2026-11-03)

The M3c freshness check now also fences **gaps between observations**. When
successive ASR workload samples are separated by more than three seconds, or
the monotonic observation clock moves backwards, the observer starts fresh
queue/mailbox occupancy clocks and consecutive-sample evidence. A resumed
sampling timer cannot treat the unobserved interval as continuous backlog,
even if its most recent sample is fresh. Normal observations at or below the
three-second boundary preserve continuity. Four model-independent regression
tests cover stalled polling, backwards time, the boundary and repeated gaps.
No ASR per-job deadline or automatic restart is implied. See
[M3d observation-gap fencing](M3_ASR_OBSERVATION_GAPS.md).

## M3e: native ASR loss-counter deltas (2026-11-03)

The periodic ASR observer now compares consecutive native cumulative dropped
segment and evicted result counters. The first successful sample establishes
a baseline, so historical native counts are not reported as newly observed
losses. A failed sample starts a new baseline; a native counter regression
cannot underflow. The status service exposes last-sample increments, observed
increments since stream/worker reset and a last-sample loss flag. These are
**sampled loss diagnostics**, not ASR accuracy, transcript quality or
individual inference-job timing. See [M3e](M3_ASR_LOSS_DELTAS.md).

## M3e CI follow-up: namespace discovery failure (2026-11-03)

The M3e PR Humble run passed unit tests and all earlier ROS graph smokes, but
its final two-namespace smoke timed out with no discovered graph nodes. Both
launch parents had started their WAV capture and component-container processes;
no cross-namespace PCM or scene leak was observed. This is a discovery/startup
failure, not evidence of an ASR loss-counter regression. The namespace smoke
now waits up to 35 seconds and reports graph topics, service readiness,
process status and both launch logs on failure. It still requires two valid
isolated PCM streams, scenes and capability services; no assertion was removed.

## M3f: independent DDS graph convergence after supervised reload (2026-11-03)

The M3e follow-up PR Humble run passed 183 tests but failed in the supervisor
reload smoke immediately after successful service and PCM readiness: the test
node temporarily could not discover `mono_source`, while the supervisor
reported a healthy graph. The test now waits up to 20 seconds for its **own**
DDS discovery cache to converge after the reload. It still requires exactly
one of each four expected namespaced nodes; failure includes the supervisor's
running/graph-health/miss/fault state and the test node's discovered graph.
This is a bounded test-only readiness barrier, not a weakened production
watchdog, changed reload contract or proof of hardware accuracy.

## M3g: configurable ASR pressure policy (2026-11-03)

The ASR pressure threshold is now a validated startup-only policy rather than
hardcoded service constants. Parameters are `asr_pressure_minimum_segments`
(default 4), `asr_pressure_minimum_age_ms` (default 10000), and
`asr_observation_max_sample_age_ms` (default 3000). The minimum age must be
at least twice the configured classifier/ASR sample period; freshness must be
at least that period. Invalid combinations fail node construction, and the
status service reports the effective values. The independent M3d continuity
gap remains capped at three seconds; consequently slow sampling cannot claim
continuous backlog, even with relaxed freshness. No automatic restart or
per-job ASR deadline is introduced. See [M3g](M3_ASR_OBSERVATION_CONFIGURATION.md).

## M3g CI follow-up: capture-session fixture discovery (2026-11-04)

The M3g push Humble run passed compilation and prior graph checks but its
capture-session restart fixture timed out after eight seconds before the
mono source subscriber appeared. The fixture launch had started its component
container; no assertion about session replay was reached. The smoke now waits
up to 35 seconds for both composable nodes and the PCM subscriber to be
visible in the test node's own DDS graph, with node/subscription diagnostics
on failure. Session-generation, old-session replay, malformed ID, duplicate,
reorder and recovery assertions are unchanged. This is a bounded test
readiness fix, not a change to production session fencing.

## M3h: opt-in advisory attention consumer (2026-11-04)

`audition_behaviors` adds a standalone ROS 2 attention candidate selector
with a new typed `AttentionTarget` interface and unit tests. It consumes
canonical `scene`, never raw unvalidated ODAS tracks, and is not activated
by M1/M2 presets. It is not a robot action or metric localization. See
[M3h attention policy](M3_ATTENTION_POLICY.md).

## M3i: attention snapshot ordering and ambiguous-ID fencing (2026-11-04)

The opt-in attention policy now rejects delayed scene generations and
out-of-order scene header stamps within the same generation/frame without
changing its retained candidate or refreshing the freshness timer. Source IDs
repeated in a single scene are excluded from selection entirely; the policy
does not guess which conflicting bearing is authoritative. A frame change
resets the incumbent. The node now records whether its immediate scene
publication was invalid, avoiding a redundant timer invalidation before the
next valid target. Four pure-policy regression tests cover generation rollback,
timestamp rollback and expiry, duplicate IDs, and frame transition. This
remains advisory only: no robot movement or TF-derived position.

## M3j: bounded attention challenger confirmation (2026-11-04)

The opt-in attention manager supports startup-only `switch_confirmations` (default 1)
and `switch_confirmation_window_ms` (default 1000) to prevent a single transient
challenger from immediately replacing an eligible incumbent. Interrupted,
expired, stale or generation/frame-changed challengers cannot inherit prior
confirmation. Initial acquisition remains immediate. See
[M3j confirmation](M3_ATTENTION_CONFIRMATION.md). This is advisory only.

## M3k: opt-in angular uncertainty gate (2026-11-04)

The attention manager can reject bearings without bounded DOA uncertainty
using startup-only `maximum_angular_variance_rad2` (default 0, disabled).
When enabled, missing, nonfinite, negative or excessive variance excludes a
source before hysteresis and confirmation. See
[M3k uncertainty gate](M3_ATTENTION_UNCERTAINTY.md). No motion control is added.

## M3l: opt-in per-source observation freshness (2026-11-04)

The attention manager supports `maximum_source_age_ms` (default 0, disabled),
rejecting sources whose `last_seen` is missing, future, malformed or older
than the configured age relative to the scene producer's own header stamp.
This complements the independent steady-clock scene timeout and does not
compare remote source timestamps to local wall time. See
[M3l source freshness](M3_ATTENTION_SOURCE_FRESHNESS.md).

## M3m: optional attention bearing continuity gate (2026-11-04)

The opt-in attention manager can reject an abrupt direction change of the
currently selected source via `maximum_direction_jump_rad` (default 0,
disabled). A rejected bearing invalidates the target rather than retaining
an actionable stale direction; new source identities and generations are
independently eligible. See [M3m direction continuity](M3_ATTENTION_DIRECTION_CONTINUITY.md).

## M3n: opt-in attention launch integration (2026-11-04)

The validated M2 profile launch can now explicitly enable the standalone
bearing-only attention manager using `enable_attention:=true` and an optional
validated `attention_parameters` YAML file. Default remains disabled; the
consumer shares the selected namespace and canonical scene topic. Standalone
launch and fail-closed parameter preflight are provided with five tests.
See [M3n launch integration](M3_ATTENTION_LAUNCH.md). No robot motion added.

## M3o: ROS graph attention integration smoke (2026-11-04)

ROS 2 Humble CI now launches the installed standalone attention node in a
non-default namespace and verifies canonical scene to typed advisory delivery,
empty-scene invalidation, new-generation acquisition and wall-timer expiry
through a separate rclpy process. The bringup package declares its runtime
dependency on `audition_behaviors`. See
[M3o graph smoke](M3_ATTENTION_GRAPH_SMOKE.md). No robot motion is exercised.

## M3p: ROS graph negative cases and Humble readiness (2026-11-04)

M3o CI failed at graph readiness because Humble rclpy Subscription lacks
`get_publisher_count()`. The smoke now uses `Node.count_publishers`
and tests generation replay, invalid DOA rejection, reacquisition and expiry
against the real attention node. See [M3p](M3_ATTENTION_GRAPH_SMOKE.md).

## M3q: optional strict scene timestamp replay fencing (2026-11-04)

The opt-in attention manager can require strictly advancing nonzero scene
header timestamps within a generation/frame using startup-only
`require_advancing_scene_stamp` (default false). Duplicate or malformed
snapshots cannot count toward challenger confirmation or extend freshness.
See [M3q scene replay fencing](M3_ATTENTION_SCENE_REPLAY.md).

## M3r: strict replay launch parity and ROS graph integration (2026-11-04)

The YAML preflight now accepts `require_advancing_scene_stamp` as a strict
boolean, matching the C++ attention node. A new ROS 2 Humble graph smoke
runs strict replay fencing and three-observation challenger confirmation
against a real namespaced attention node. See
[M3r strict graph](M3_ATTENTION_STRICT_GRAPH.md).

## M3s: strict attention graph asynchronous correlation (2026-11-04)

The M3r PR Humble smoke failed when a timer-originated `no_scene` advisory
arrived before the first scene response. The graph test now matches replies
to the published scene header and generation and retries initial admission
across DDS startup. All replay and challenger assertions remain enforced.
See [M3s strict graph](M3_ATTENTION_STRICT_GRAPH.md).
