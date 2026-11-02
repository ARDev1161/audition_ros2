# libaudition Integration and User Configuration

**Status:** Architecture and proposed public configuration contract, with M0–M2c implementation notes in sections 9–11. The section 3 example is still future YAML, not a runnable preset.  
**Baseline:** libaudition `72fc4040f0e21f1271a6b14c787cc8fa73cb3ad3` (main, 2026-11-01); ROS 2 Humble.  
**Companions:** [ROS node architecture](ROS2_ARCHITECTURE.md), [roadmap](ROADMAP.md), [M1 implementation status](M1_IMPLEMENTATION.md).

## 1. Architecture rule: reuse the library, not its implementation

`libaudition` is a C++ library with backend-independent algorithms, data types and optional native backend packages. It must remain free of ROS 2 dependencies. `audition_ros2` links installed libaudition targets inside the appropriate ROS nodes; adapters translate timestamps, frames, IDs, audio and classification/speech/event results into ROS interfaces. Do **not** build separate ROS nodes for each classifier, VAD, AEC, resampler or tracker.

A small number of capability-oriented nodes is sufficient:

| ROS node | Library integration | Configuration ownership |
| --- | --- | --- |
| `audio_capture` | `AudioBuffer`, optional `dsp::AudioFrontend` (channel routing/calibration) | Device, WAV or ALSA, capture clocks, channel mapping |
| `spatial_perception` | `ISpatialAudioEngine`, `OdasSpatialEngine`, `SourceTrackLifecycle`, `SpatialIdentityCoordinator` as justified | Array geometry and direction/tracking/separation, source generations |
| `semantic_perception` | `IAudioClassifier`, `ProgressiveSourceClassificationRuntime`, `IVoiceActivityDetector`, `PerSourceSpeechRuntime`, `IAsrEngine`, optional CLAP, speaker and sound features | Model instances, model queues, source-local postprocessing, speech settings |
| `acoustic_scene` | Acoustic event assembly and temporal processing; ROS-side canonical reconciliation | Snapshot/transition publication, TTL, event provenance and retirement |
| `attention_manager` | Sound-derived observations (no mandatory direct library link) | Attention scores, investigation policy and motion constraints |
| `multimodal_fusion` | Optional acoustic `IRangeEstimator` / `ISpatialFusion`, plus TF2 and cross-modal logic outside libaudition | Time alignment, association, uncertainty and metric localization |
| `acoustic_memory` | Optional source/speaker fingerprint comparison or registry abstractions | Persistence, room context, retention and privacy |
| Optional `speech_output` | `ISpeechSynthesizer` and audio playback reference for AEC | TTS model, audio output, playback/reference clock |

`audio_capture` -> `spatial_perception` -> `semantic_perception` is the multi-microphone route. A mono/no-ODAS configuration must have a **separate supported routing path** from capture to semantic processing rather than pretending to have spatial source tracking. Do not apply phase-altering AEC, AGC or NS to the ODAS multi-channel path. Send a separate per-source mono branch to speech enhancement; WebRTC AEC needs a *true pre-DAC playback reference*.

A slow ASR, model load or DDS subscriber must never block capture/ODAS processing. Use bounded inference queues and source generation fences; isolate failure-prone ODAS in its own process when enabled.

## 2. Three distinct types of user configuration

### A. Build-time capabilities

Optional native dependencies are included when building/installing libaudition. Examples: `LIBAUDITION_WITH_ODAS`, `LIBAUDITION_WITH_SHERPA`, `LIBAUDITION_WITH_YAMNET`, `LIBAUDITION_WITH_CLAP`, `LIBAUDITION_WITH_WEBRTC_APM`, `LIBAUDITION_WITH_WHISPERCPP`, `LIBAUDITION_WITH_WORLD`, `LIBAUDITION_WITH_GTSAM`, and TTS options. Distinguish the installed CMake target from a model file: enabling a backend does **not** download or validate model weights unless separately configured.

`audition_ros2` should discover installed targets with `find_package(libaudition CONFIG REQUIRED)` and expose only available adapters. The current M1 CMake option is `AUDITION_ROS2_WITH_ODAS=ON`; it requires the libaudition ODAS backend target. Other optional adapters are **not yet implemented**. Some optional WebRTC APM targets require C++20 even though libaudition's core is C++17; keep these behind explicit isolated components/build targets.

An invalid requested backend must produce an actionable configuration error, never silently fall back to `disabled` while reporting spatial localization.

### B. Startup pipeline profile (ROS parameters + launch)

Choose input, channel map, microphone frame, backend names, absolute model paths and resource budgets in YAML. Keep common presets:

- `mono_speech`: one channel, VAD/ASR, no ODAS, no fabricated directions.
- `spatial_events`: multi-channel array, ODAS, classifications, event state, no ASR required.
- `robot_full`: spatial + semantic + optional attention/fusion; only explicitly enabled models run.
- `replay_ci`: pinned WAV, synthetic geometry/test models, no physical accuracy claim.

Presets compose one or more node-specific YAML files. The launch system should let users override input path, model directory and feature preset without editing package source files. Support multiple namespaced sensor instances and robot IDs. Do not assume `microphone_array` is unique across a fleet.

### C. Runtime reconfiguration

A parameter callback can safely change bounded scalar values such as class thresholds, source selection priorities and some timeouts. Geometry, sample rate, channel routing, backend selection, VAD/ASR model path, inference provider and audio processing graph typically require an explicit lifecycle deactivation/reconfiguration transaction. A hot change **must** invalidate stale jobs and source generations, with a clear observable transition (no silent use of old model results).

A future `GetCapabilities` service should report compiled/available backends, model availability, selected provider and active features. A `ReloadPipeline` service or action should validate the new configuration, quiesce consumers and rebuild affected modules with rollback on failure. These services do not exist yet.

## 3. Proposed user-facing YAML: full robot profile

The following illustrates the **target user experience**, not a file the current M1 runtime can parse. The existing WAV/ODAS implementation accepts only its currently documented parameters.

```yaml
# PROPOSED FUTURE CONFIG — not yet runnable
"/**/audio_capture":
  ros__parameters:
    source: alsa                  # alsa | wav | ros_topic
    device: "hw:ReSpeaker,0"      # illustrative; discover actual ALSA device
    frame_id: "microphone_array"
    sample_rate_hz: 16000
    channels: 6
    channel_layout: interleaved
    queue_depth: 4

"/**/spatial_perception":
  ros__parameters:
    enabled: true
    backend: odas
    sample_rate_hz: 16000
    input_channel_indices: [1, 2, 3, 4]
    geometry_calibrated: false   # requires verified geometry before enabling
    microphone_positions_m: []  # intentionally unknown, do not substitute a fixture
    odas_hop_frames: 128
    odas_frame_size: 256
    enable_separation: true
    max_tracks: 4

"/**/semantic_perception":
  ros__parameters:
    classification.enabled: true
    classification.backend: yamnet
    classification.model_path: "/opt/robot/models/yamnet.onnx"
    classification.window_ms: 1000
    classification.hop_ms: 500
    classification.max_sources: 8
    classification.max_pending: 16

    speech.enabled: true
    speech.vad_backend: sherpa
    speech.asr_backend: whispercpp
    speech.asr_model_path: "/opt/robot/models/asr.gguf"
    speech.language: "ru"
    speech.native_initial_context_ms: 1000
    speech.normalize_asr_segments: false
    speech.max_pending_segments: 16

    clap.enabled: false
    speaker_id.enabled: false
    noise_suppression.enabled: false

"/**/acoustic_scene":
  ros__parameters:
    publish_rate_hz: 5.0
    source_stale_timeout_ms: 1500
    max_sources: 8
    event_history_persist: false
```

**Critical:** `geometry_calibrated: false` deliberately makes ODAS startup fail when requested, until coordinates have been measured and supplied. A synthetic test geometry must never be treated as real calibration. `whispercpp` path/format/provider are adapter-dependent and subject to implementation and runtime validation; this YAML does not demonstrate working Whisper ASR.

An additional small `attention.yaml` and `fusion.yaml` should configure Nav2 and sensor connections independently; audio-only perception must continue working without them. For the first usable version, a single preset selector and 10–20 meaningful parameters are preferable to exposing every underlying algorithm knob.

## 4. Feature coverage and priority

| Functionality in libaudition | ROS adapter/consumer | Priority | User-visible result | Implementation status |
| --- | --- | --- | --- | --- |
| AudioBuffer, PCM, format and time handling | capture / scene | M0–M1 | `audio/raw`; capture diagnostics | **WAV replay implemented** |
| ODAS SSL/SST/SSS | spatial | M1 | `sources`, `scene`, `internal/source_audio` | **Optional native adapter and synthetic CI implemented; real array unvalidated** |
| Channel calibration via AudioFrontend | capture / spatial input | M1 | array calibration and quality diagnostics | Proposed |
| Spatial identity and lifecycle | spatial / scene | M1–M2 | coherent source IDs, LOST/ENDED, generations | Partial ad-hoc IDs; source retirement pending |
| YAMNet or sherpa audio tagging | semantic | M2 | per-source `SoundClassification`, `AcousticEvent` | Proposed |
| Progressive 1s/5s classification | semantic | M2 | fast event first, refined later | Proposed; available upstream |
| Temporal hysteresis, events and change detection | semantic / scene | M2 | STARTED/UPDATED/ENDED/CANCELLED | Proposed |
| Sherpa VAD and Whisper.cpp ASR | semantic | M2 | `Transcript`, optional KWS events | Proposed |
| Native VAD segments, first-utterance recovery, ASR gain | semantic (speech runtime) | M2 | improved segment completeness and explicit ASR gain diagnostics | **New in libaudition; ROS adapter pending** |
| Noise suppression / AGC, WebRTC AEC | semantic mono branch | M2 optional | cleaner speech and quality metrics | Proposed, optional dependencies |
| CLAP and acoustic embeddings | semantic | M2–M3 optional | open-vocabulary candidates / embeddings | Proposed |
| Speaker diarization / identification | semantic / memory | M3 optional | distinct `SpeakerId` and verification evidence | Proposed; no unconditional identity guarantee |
| TTS | optional speech_output | M3 optional | `SpeakText` action and playback reference | Proposed |
| Attention and novelty logic | attention | M3 | `LookAtSound`, `InvestigateSound` actions | Proposed |
| Acoustic range/fusion | multimodal_fusion | M4 | `LocalizedSourceArray` with covariance | Proposed; never from DOA alone |
| WORLD voice characteristics, AASIST authenticity | semantic plugins | M4+ optional | feature observations, not commands | Proposed; specialized demand |
| DTW, motifs, EMA/CUSUM, recurrent patterns | semantic / memory | M4–M6 optional | recurring event and change hypotheses | Proposed |
| Acoustic memory and room summaries | memory | M6 | history and room evidence | Proposed |
| Nav2 costmap | existing Nav2 plugin process | M5 | optional soft costs from metric localization | Proposed; no audio-only lethal obstacles |

**Do not activate all algorithms by default.** A classification stage, speech stage and enhancement stage have independent model dependencies, latency, memory cost and failure modes. Backend availability is not equivalent to benchmarked performance on the target RK3588/Qualcomm.

## 5. Public ROS outputs vs internal transport

The user's default entry point is the **canonical scene**, not raw inference streams:

| Public API (proposed names, some messages already exist) | Purpose |
| --- | --- |
| `/audition/scene` (`AcousticScene`) | Snapshot of known acoustic sources and active events |
| `/audition/sources` (`AcousticSourceArray`) | Direction and activity, with age and source identity |
| `/audition/events` (`AcousticEvent`) | Changes in environmental acoustic events |
| `/audition/transcripts` (`Transcript`) | Per-source speech results and timestamps |
| `/audition/localized_sources` (`LocalizedSourceArray`) | Metric positions only from validated estimators |
| `/diagnostics` (`DiagnosticArray`) | Capture drops, model health, queue depth, backend versions |

`/audition/internal/spatial_sources`, `/audition/internal/source_audio` and `/audition/internal/classifications` are transport between our own nodes. Consumers should not depend on these unstable internal topics. `acoustic_scene` alone publishes the canonical public scene/event/transcript streams.

Actions worth implementing: `ListenForEvent`, `LookAtSound`, `InvestigateSound`, optional `SpeakText`. They are not equivalent to autonomous movement based on raw speech or classifier labels. Action servers must check authorization and independently enforced robot safety.

Events/classification scores must preserve the distinction between **uncalibrated model scores** and calibrated probabilities. Speaker ID, physical source ID and ODAS track ID are different identifiers. ROS `header.frame_id`, capture timestamp and source generation must be preserved through asynchronous workers.

## 6. Runtime model/plugin selection

Use two complementary mechanisms:

1. **Installed native CMake targets** determine which backend factories are present. They must be version-pinned and tested; if missing, fail startup with a precise error.
2. **Configuration-driven factories** in `audition_ros2` instantiate the requested `libaudition` interface (`IAudioClassifier`, `IAsrEngine`, `IVoiceActivityDetector`, etc.) with a model/provider and validate `BackendInfo` and `capabilities()`.

For custom algorithms, prefer a documented backend factory interface and a ROS `pluginlib` loader **if external binary extension becomes a user requirement**. Do not introduce ROS `pluginlib` into libaudition itself. For initial M2, explicit factory registration compiled into the package is simpler and easier to test than arbitrary dynamic plugin loading.

A "disabled" module is a first-class state, not a fake classifier producing zero scores. When a model is unavailable, report degraded health and continue spatial perception.

## 7. Development and validation order

1. **M1:** retain successful ODAS + WAV route; add real mic geometry, bounded worker queues, timestamp/track retirement correctness and ALSA capture.
2. **M2a:** implement one optional YAMNet classifier adapter, source-generation-safe runtime and `AcousticEvent` lifecycle; verify deterministic WAV fixture before enabling other models.
3. **M2b:** integrate native VAD + ASR via `PerSourceSpeechRuntime` and source retirement/flush semantics. Compare beginning-of-utterance preservation and overlap against recorded ReSpeaker captures.
4. **M2c:** user-facing pipeline YAML schema, preset validation, model discovery/capabilities, service API and runtime config restrictions.
5. **M3:** attention actions first; then CLAP, speech enhancement and optional TTS/speaker features as actual use cases demand.
6. **M4–M6:** cross-modal localization, optional Nav2 plugin and acoustic memory only after actual measurement evidence.

Acceptance: each enabled module needs build-time availability test, malformed-config rejection, model-file validation, deterministic replay, stale-generation regression, bounded resource metrics and device-specific tests. Native ODAS CI smoke is not a substitute for measured DOA accuracy. Production must never treat unknown source distance or silence as safe geometric evidence.

## 8. Current implementation vs this proposal

**Implemented:** `audition_msgs`, WAV/ALSA capture, safe disabled-spatial scaffold, strict mono capture-to-semantic routing, optional native ODAS, YAMNet classification/events and Sherpa/Whisper speech adapters. Canonical scene source/event/transcript generation fences are tested. ReSpeaker geometry is still uncalibrated and real-model accuracy has not been established.

**Not implemented:** generic runtime backend registry, transactional reload, hardware-validated DOA/speech accuracy, confidence calibration, multi-sensor fusion and navigation consumers. An opt-in validated startup profile selector and typed semantic capability service are implemented in M2c.

This document describes the desired developer/user-facing experience and gives a concrete contract for further implementation; it must not be misread as completed functionality.


## 9. Implemented canonical source/semantic provenance (M2 increment)

The `acoustic_scene` component now validates spatial source generation and
sensor frame identity before mutating canonical state. A malformed spatial
snapshot cannot cancel legitimate events or switch the active generation.

Only **known, not-yet-ended** sources may introduce classifications and
acoustic events to public topics. Source `LOST` remains provisional; a later
ACTIVE observation can recover it. When a source reaches `ENDED`, every
still-active event for that source is published once as `CANCELLED` with
`termination_reason=source_ended` and removed from `/audition/scene`.
A generation reset similarly cancels events with
`termination_reason=spatial_stream_reset`. A delayed worker result cannot
recreate an event on a retired source.

A *final* ASR transcript may arrive after source retirement: to avoid dropping
the last spoken words, `acoustic_scene` retains a bounded, generation-specific
retirement marker for `late_transcript_grace_ms` (default 5000 ms). Only known
retired source IDs in the **current** spatial generation are allowed; reset
invalidates the grace. Unknown IDs, future/old generations, non-final
transcripts and invalid scores are discarded. The grace is limited in time
and in memory (up to 4× the configured `max_sources`).

Set policy via the installed
[`audition_bringup/config/acoustic_scene.yaml`](../audition_bringup/config/acoustic_scene.yaml).
Pass `scene_config:=/path/to/scene.yaml` to M1/M2 WAV or ALSA launch.
The lifecycle uses wall-monotonic arrival time; ROS `header.stamp` remains
the source observation timestamp and may use simulated time.

These contracts have unit tests and an end-to-end ROS graph regression with
an explicit synthetic source, source expiry, late final ASR, stale rejection
and stream reset. The regression uses **injected messages**, not a claim of
real classifier accuracy.



## 10. Implemented mono/no-ODAS routing (2026-11-02)

The M2 launch parameter `spatial_backend:=mono` now chooses a dedicated
`MonoSourceNode`, not a disabled spatial engine. One real 16 kHz mono
capture stream becomes one source-local PCM stream for the existing semantic
runtime. The canonical source has `has_direction=false`,
`has_range=false`, `has_position=false`, empty spatial/speaker IDs and
an explicit generation fence. This is **not** speaker diarization or a
localization estimate. Sequence gaps, capture-clock regressions and long
timestamp gaps reset the generation; invalid PCM never advances it.
See [M1 implementation](M1_IMPLEMENTATION.md) for launch commands, limits
and test coverage.

The mono route now also has an opt-in validated M2c profile selector. The
section 3 example is still future YAML, not the supported schema; see
[M2c configuration](M2_PROFILE_CONFIGURATION.md). Transactional reload
remains unimplemented.


## 11. Implemented M2c startup profiles and typed capability service (2026-11-02)

The new opt-in m2_profile.launch.py uses a versioned startup YAML schema,
explicit model paths, WAV header/channel/rate checks and ODAS calibration
preflight. It rejects unknown keys/backends and invalid model configuration
before starting any pipeline nodes. Four profiles are installed; spatial_events
and robot_full are intentionally blocked by the uncalibrated ReSpeaker example.

The semantic node exposes audition_msgs/srv/GetCapabilities at
/audition/semantic_perception/get_capabilities. It reports compiled backends,
active semantic stages and loaded model *names* after successful startup.
It cannot certify upstream ODAS health or actual model accuracy. Semantic
parameters are currently immutable: set_parameters rejects hot changes
rather than silently retaining stale model workers. ReloadPipeline with
transactional generation reset and rollback is **not** implemented.

See [M2c startup profile contract](M2_PROFILE_CONFIGURATION.md) for runnable
commands, negative tests, scope limits and the exact supported schema.

## 12. M2d namespace isolation


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

## 13. M2e ODAS bounded ingress and capture continuity (2026-11-02)

The native ODAS adapter now uses a model-independent `OdasIngress` stage.
It validates complete normalized PCM frames (including channel mapping, time,
calibrated frame, rate and chunk size) **before** changing its state. Accepted
frames are converted to fixed-size, timestamped interleaved hops without
resampling, channel mixing or phase-altering enhancement. The default buffer
limit is eight hops, with at most four hops in one capture message.

Sequence gaps, backwards capture time, gaps greater than 250 ms beyond the
expected next sample and capture-channel changes reset the ingress and ODAS
state. The adapter emits an empty **new-generation** spatial snapshot before
any new results, allowing canonical scene/event and speech workers to reject
stale generations. On buffer overflow, the entire incoming chunk is dropped
and the epoch resets; on an ODAS exception, the epoch also resets and the
failure propagates to the ROS error log. Malformed frames are rejected without
mutating the generation or pending audio.

A typed, namespaced read-only service exposes counters:
```bash
ros2 service call /audition/spatial_perception/get_ingress_status \
  audition_msgs/srv/GetIngressStatus "{}"
# or /robot_a/audition/spatial_perception/get_ingress_status
```
Counters include accepted/rejected frames, overflow drops, discontinuity and
backend resets, staged hops, successfully processed native ODAS hops,
buffered frames and current source generation.
They are since node startup, not DDS transport drop counters. The service
exists for `spatial_backend:=disabled` too, returning zero ingress counts.
The mono route has its own generation logic and does not expose this service.

Deterministic C++ tests cover interleaving, timestamps, malformed input,
reset causes, buffer overflow and recovery. Native ODAS CI additionally
checks the service against actual synthetic WAV processing. Neither test
certifies ReSpeaker calibration, measured DOA accuracy or end-to-end latency.
A full transactional `ReloadPipeline` is still future work.

## 14. M2f: resilient ALSA capture transport (2026-11-02)

The live ALSA worker now has a bounded-backoff reopen loop and a no-PCM
watchdog, with a typed `GetCaptureStatus` service. A recovered XRUN or a
successful device reopen skips one sequence number before the next valid
capture frame. Existing mono/ODAS ingress detects the gap and advances its
source generation; no samples, speaker identities or timestamps are fabricated.
The device and negotiated rate/channels remain fixed; failure to open at
startup is fatal. This is a transport-level recovery increment, not the
transactional semantic model `ReloadPipeline` or a field validation.

See [M1 capture and scene status](M1_IMPLEMENTATION.md) for launch arguments,
service fields and testing limitations.

## 15. M2g semantic epoch admission (2026-11-02)

The semantic component now gates classifier and ASR input on canonical
`/audition/sources` admission, including active source generation, frame
and live source identity. Unknown, retired, old-generation and pre-admission
PCM cannot create inference work. A valid generation transition forgets
old classifier jobs and resets speech workers; malformed snapshots leave
the current generation untouched. The namespaced read-only
`semantic_perception/get_semantic_status` service exposes drops, admissions,
resets and rejected snapshots. See
[M2g semantic epoch fencing](M2_SEMANTIC_EPOCH_FENCING.md).
This is a prerequisite for, not an implementation of, transactional reload.

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
