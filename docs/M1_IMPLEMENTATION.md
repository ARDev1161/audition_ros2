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
