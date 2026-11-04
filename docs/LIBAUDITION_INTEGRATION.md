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


## M2o: process-owned profile reload with verified failback (2026-11-02)

A standalone ROS 2 supervisor now exposes a typed pipeline/reload service.
It validates M2c profiles and ODAS geometry before stopping the old graph,
then starts the candidate and waits for all core ROS nodes and the semantic
capability service. A failed candidate is cleaned up and the prior profile
is restarted; rollback is reported as successful **only** after readiness.
This is an opt-in **stop/start operation with an audio gap**, not a full
atomic multi-node in-place ReloadPipeline. See
[M2o supervisor details](M2_PIPELINE_SUPERVISOR.md). ODAS native integration
and physical ReSpeaker performance still require separate validation.

## M2p: supervised process failure containment (2026-11-02)

The optional ReloadPipeline supervisor now detects exited launch parents,
attempts surviving process-group cleanup and exposes typed GetPipelineStatus
with last-good profile and faults. Recovery is explicit, guarded by preflight
and graph disappearance; it never adopts an occupied namespace. This is not
two-phase resource reservation or a guarantee of continued model/hardware
health. See [supervisor contract](M2_PIPELINE_SUPERVISOR.md).

## M2q: supervised ROS graph health (2026-11-02)

The opt-in process supervisor now samples expected capture/spatial/scene/semantic
ROS nodes and the semantic capability service once per second after startup.
Five consecutive missing/duplicate discovery results or probe errors cause
owned process-group cleanup and an explicit-recovery fault, even if the launch
parent remains alive. A transient miss resets on the next healthy sample.
`GetPipelineStatus` exposes `graph_healthy` and `graph_miss_count`.
This is not an audio heartbeat, hardware health guarantee or zero-downtime
reload. See [M2q supervisor watchdog](M2_PIPELINE_SUPERVISOR.md).

## M2r: supervisor PCM progress watchdog (2026-11-02)

The opt-in supervisor now observes valid namespaced raw capture PCM, exposes
freshness and accepted-frame counters via `GetPipelineStatus`, and contains a
live ALSA pipeline after a sustained capture stall (10-second initial grace,
5-second inactivity timeout and five failed one-second samples). Finite WAV
replay is observed but exempt from stall enforcement after EOF. Old session
replays and duplicate sequence numbers cannot refresh the heartbeat. See
[M2r supervisor flow contract](M2_PIPELINE_SUPERVISOR.md). This is not
proof of ASR inference, speaker separation, or physical microphone health.

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

## M2v: native worker workload snapshots (2026-11-03)

The semantic status service now samples libaudition's per-source YAMNet
worker states/timing and native Sherpa/Whisper ASR queue/mailbox/drop
statistics. Classifier states count sources, not queued jobs; ASR queue
excludes the running job. Silence does not trigger a watchdog.
See [M2v workload telemetry](M2_INFERENCE_WORKLOAD_TELEMETRY.md).

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

## M3h: opt-in bearing-only attention policy (2026-11-04)

The standalone `audition_behaviors/attention_manager` consumes canonical
`scene` and publishes an advisory `attention/target` with generation/frame
fencing, bounded freshness, deterministic selection and switch hysteresis.
It never commands motion or invents range; existing launch presets are
unchanged. See [M3h attention policy](M3_ATTENTION_POLICY.md).

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
