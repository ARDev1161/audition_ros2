# libaudition Integration and User Configuration

**Status:** Proposed public configuration contract; do not assume that the parameters in the examples already exist.  
**Baseline:** libaudition \`72fc4040f0e21f1271a6b14c787cc8fa73cb3ad3\` (main, 2026-11-01); ROS 2 Humble.  
**Companions:** [ROS node architecture](ROS2_ARCHITECTURE.md), [roadmap](ROADMAP.md), [M1 implementation status](M1_IMPLEMENTATION.md).

## 1. Architecture rule: reuse the library, not its implementation

\`libaudition\` is a C++ library with backend-independent algorithms, data types and optional native backend packages. It must remain free of ROS 2 dependencies. \`audition_ros2\` links installed libaudition targets inside the appropriate ROS nodes; adapters translate timestamps, frames, IDs, audio and classification/speech/event results into ROS interfaces. Do **not** build separate ROS nodes for each classifier, VAD, AEC, resampler or tracker.

A small number of capability-oriented nodes is sufficient:

| ROS node | Library integration | Configuration ownership |
| --- | --- | --- |
| \`audio_capture\` | \`AudioBuffer\`, optional \`dsp::AudioFrontend\` (channel routing/calibration) | Device, WAV or ALSA, capture clocks, channel mapping |
| \`spatial_perception\` | \`ISpatialAudioEngine\`, \`OdasSpatialEngine\`, \`SourceTrackLifecycle\`, \`SpatialIdentityCoordinator\` as justified | Array geometry and direction/tracking/separation, source generations |
| \`semantic_perception\` | \`IAudioClassifier\`, \`ProgressiveSourceClassificationRuntime\`, \`IVoiceActivityDetector\`, \`PerSourceSpeechRuntime\`, \`IAsrEngine\`, optional CLAP, speaker and sound features | Model instances, model queues, source-local postprocessing, speech settings |
| \`acoustic_scene\` | Acoustic event assembly and temporal processing; ROS-side canonical reconciliation | Snapshot/transition publication, TTL, event provenance and retirement |
| \`attention_manager\` | Sound-derived observations (no mandatory direct library link) | Attention scores, investigation policy and motion constraints |
| \`multimodal_fusion\` | Optional acoustic \`IRangeEstimator\` / \`ISpatialFusion\`, plus TF2 and cross-modal logic outside libaudition | Time alignment, association, uncertainty and metric localization |
| \`acoustic_memory\` | Optional source/speaker fingerprint comparison or registry abstractions | Persistence, room context, retention and privacy |
| Optional \`speech_output\` | \`ISpeechSynthesizer\` and audio playback reference for AEC | TTS model, audio output, playback/reference clock |

\`audio_capture\` -> \`spatial_perception\` -> \`semantic_perception\` is the multi-microphone route. A mono/no-ODAS configuration must have a **separate supported routing path** from capture to semantic processing rather than pretending to have spatial source tracking. Do not apply phase-altering AEC, AGC or NS to the ODAS multi-channel path. Send a separate per-source mono branch to speech enhancement; WebRTC AEC needs a *true pre-DAC playback reference*.

A slow ASR, model load or DDS subscriber must never block capture/ODAS processing. Use bounded inference queues and source generation fences; isolate failure-prone ODAS in its own process when enabled.

## 2. Three distinct types of user configuration

### A. Build-time capabilities

Optional native dependencies are included when building/installing libaudition. Examples: \`LIBAUDITION_WITH_ODAS\`, \`LIBAUDITION_WITH_SHERPA\`, \`LIBAUDITION_WITH_YAMNET\`, \`LIBAUDITION_WITH_CLAP\`, \`LIBAUDITION_WITH_WEBRTC_APM\`, \`LIBAUDITION_WITH_WHISPERCPP\`, \`LIBAUDITION_WITH_WORLD\`, \`LIBAUDITION_WITH_GTSAM\`, and TTS options. Distinguish the installed CMake target from a model file: enabling a backend does **not** download or validate model weights unless separately configured.

\`audition_ros2\` should discover installed targets with \`find_package(libaudition CONFIG REQUIRED)\` and expose only available adapters. The current M1 CMake option is \`AUDITION_ROS2_WITH_ODAS=ON\`; it requires the libaudition ODAS backend target. Other optional adapters are **not yet implemented**. Some optional WebRTC APM targets require C++20 even though libaudition's core is C++17; keep these behind explicit isolated components/build targets.

An invalid requested backend must produce an actionable configuration error, never silently fall back to \`disabled\` while reporting spatial localization.

### B. Startup pipeline profile (ROS parameters + launch)

Choose input, channel map, microphone frame, backend names, absolute model paths and resource budgets in YAML. Keep common presets:

- \`mono_speech\`: one channel, VAD/ASR, no ODAS, no fabricated directions.
- \`spatial_events\`: multi-channel array, ODAS, classifications, event state, no ASR required.
- \`robot_full\`: spatial + semantic + optional attention/fusion; only explicitly enabled models run.
- \`replay_ci\`: pinned WAV, synthetic geometry/test models, no physical accuracy claim.

Presets compose one or more node-specific YAML files. The launch system should let users override input path, model directory and feature preset without editing package source files. Support multiple namespaced sensor instances and robot IDs. Do not assume \`microphone_array\` is unique across a fleet.

### C. Runtime reconfiguration

A parameter callback can safely change bounded scalar values such as class thresholds, source selection priorities and some timeouts. Geometry, sample rate, channel routing, backend selection, VAD/ASR model path, inference provider and audio processing graph typically require an explicit lifecycle deactivation/reconfiguration transaction. A hot change **must** invalidate stale jobs and source generations, with a clear observable transition (no silent use of old model results).

A future \`GetCapabilities\` service should report compiled/available backends, model availability, selected provider and active features. A \`ReloadPipeline\` service or action should validate the new configuration, quiesce consumers and rebuild affected modules with rollback on failure. These services do not exist yet.

## 3. Proposed user-facing YAML: full robot profile

The following illustrates the **target user experience**, not a file the current M1 runtime can parse. The existing WAV/ODAS implementation accepts only its currently documented parameters.

\`\`\`yaml
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
\`\`\`

**Critical:** \`geometry_calibrated: false\` deliberately makes ODAS startup fail when requested, until coordinates have been measured and supplied. A synthetic test geometry must never be treated as real calibration. \`whispercpp\` path/format/provider are adapter-dependent and subject to implementation and runtime validation; this YAML does not demonstrate working Whisper ASR.

An additional small \`attention.yaml\` and \`fusion.yaml\` should configure Nav2 and sensor connections independently; audio-only perception must continue working without them. For the first usable version, a single preset selector and 10–20 meaningful parameters are preferable to exposing every underlying algorithm knob.

## 4. Feature coverage and priority

| Functionality in libaudition | ROS adapter/consumer | Priority | User-visible result | Implementation status |
| --- | --- | --- | --- | --- |
| AudioBuffer, PCM, format and time handling | capture / scene | M0–M1 | \`audio/raw\`; capture diagnostics | **WAV replay implemented** |
| ODAS SSL/SST/SSS | spatial | M1 | \`sources\`, \`scene\`, \`internal/source_audio\` | **Optional native adapter and synthetic CI implemented; real array unvalidated** |
| Channel calibration via AudioFrontend | capture / spatial input | M1 | array calibration and quality diagnostics | Proposed |
| Spatial identity and lifecycle | spatial / scene | M1–M2 | coherent source IDs, LOST/ENDED, generations | Partial ad-hoc IDs; source retirement pending |
| YAMNet or sherpa audio tagging | semantic | M2 | per-source \`SoundClassification\`, \`AcousticEvent\` | Proposed |
| Progressive 1s/5s classification | semantic | M2 | fast event first, refined later | Proposed; available upstream |
| Temporal hysteresis, events and change detection | semantic / scene | M2 | STARTED/UPDATED/ENDED/CANCELLED | Proposed |
| Sherpa VAD and Whisper.cpp ASR | semantic | M2 | \`Transcript\`, optional KWS events | Proposed |
| Native VAD segments, first-utterance recovery, ASR gain | semantic (speech runtime) | M2 | improved segment completeness and explicit ASR gain diagnostics | **New in libaudition; ROS adapter pending** |
| Noise suppression / AGC, WebRTC AEC | semantic mono branch | M2 optional | cleaner speech and quality metrics | Proposed, optional dependencies |
| CLAP and acoustic embeddings | semantic | M2–M3 optional | open-vocabulary candidates / embeddings | Proposed |
| Speaker diarization / identification | semantic / memory | M3 optional | distinct \`SpeakerId\` and verification evidence | Proposed; no unconditional identity guarantee |
| TTS | optional speech_output | M3 optional | \`SpeakText\` action and playback reference | Proposed |
| Attention and novelty logic | attention | M3 | \`LookAtSound\`, \`InvestigateSound\` actions | Proposed |
| Acoustic range/fusion | multimodal_fusion | M4 | \`LocalizedSourceArray\` with covariance | Proposed; never from DOA alone |
| WORLD voice characteristics, AASIST authenticity | semantic plugins | M4+ optional | feature observations, not commands | Proposed; specialized demand |
| DTW, motifs, EMA/CUSUM, recurrent patterns | semantic / memory | M4–M6 optional | recurring event and change hypotheses | Proposed |
| Acoustic memory and room summaries | memory | M6 | history and room evidence | Proposed |
| Nav2 costmap | existing Nav2 plugin process | M5 | optional soft costs from metric localization | Proposed; no audio-only lethal obstacles |

**Do not activate all algorithms by default.** A classification stage, speech stage and enhancement stage have independent model dependencies, latency, memory cost and failure modes. Backend availability is not equivalent to benchmarked performance on the target RK3588/Qualcomm.

## 5. Public ROS outputs vs internal transport

The user's default entry point is the **canonical scene**, not raw inference streams:

| Public API (proposed names, some messages already exist) | Purpose |
| --- | --- |
| \`/audition/scene\` (\`AcousticScene\`) | Snapshot of known acoustic sources and active events |
| \`/audition/sources\` (\`AcousticSourceArray\`) | Direction and activity, with age and source identity |
| \`/audition/events\` (\`AcousticEvent\`) | Changes in environmental acoustic events |
| \`/audition/transcripts\` (\`Transcript\`) | Per-source speech results and timestamps |
| \`/audition/localized_sources\` (\`LocalizedSourceArray\`) | Metric positions only from validated estimators |
| \`/diagnostics\` (\`DiagnosticArray\`) | Capture drops, model health, queue depth, backend versions |

\`/audition/internal/spatial_sources\`, \`/audition/internal/source_audio\` and \`/audition/internal/classifications\` are transport between our own nodes. Consumers should not depend on these unstable internal topics. \`acoustic_scene\` alone publishes the canonical public scene/event/transcript streams.

Actions worth implementing: \`ListenForEvent\`, \`LookAtSound\`, \`InvestigateSound\`, optional \`SpeakText\`. They are not equivalent to autonomous movement based on raw speech or classifier labels. Action servers must check authorization and independently enforced robot safety.

Events/classification scores must preserve the distinction between **uncalibrated model scores** and calibrated probabilities. Speaker ID, physical source ID and ODAS track ID are different identifiers. ROS \`header.frame_id\`, capture timestamp and source generation must be preserved through asynchronous workers.

## 6. Runtime model/plugin selection

Use two complementary mechanisms:

1. **Installed native CMake targets** determine which backend factories are present. They must be version-pinned and tested; if missing, fail startup with a precise error.
2. **Configuration-driven factories** in \`audition_ros2\` instantiate the requested \`libaudition\` interface (\`IAudioClassifier\`, \`IAsrEngine\`, \`IVoiceActivityDetector\`, etc.) with a model/provider and validate \`BackendInfo\` and \`capabilities()\`.

For custom algorithms, prefer a documented backend factory interface and a ROS \`pluginlib\` loader **if external binary extension becomes a user requirement**. Do not introduce ROS \`pluginlib\` into libaudition itself. For initial M2, explicit factory registration compiled into the package is simpler and easier to test than arbitrary dynamic plugin loading.

A "disabled" module is a first-class state, not a fake classifier producing zero scores. When a model is unavailable, report degraded health and continue spatial perception.

## 7. Development and validation order

1. **M1:** retain successful ODAS + WAV route; add real mic geometry, bounded worker queues, timestamp/track retirement correctness and ALSA capture.
2. **M2a:** implement one optional YAMNet classifier adapter, source-generation-safe runtime and \`AcousticEvent\` lifecycle; verify deterministic WAV fixture before enabling other models.
3. **M2b:** integrate native VAD + ASR via \`PerSourceSpeechRuntime\` and source retirement/flush semantics. Compare beginning-of-utterance preservation and overlap against recorded ReSpeaker captures.
4. **M2c:** user-facing pipeline YAML schema, preset validation, model discovery/capabilities, service API and runtime config restrictions.
5. **M3:** attention actions first; then CLAP, speech enhancement and optional TTS/speaker features as actual use cases demand.
6. **M4–M6:** cross-modal localization, optional Nav2 plugin and acoustic memory only after actual measurement evidence.

Acceptance: each enabled module needs build-time availability test, malformed-config rejection, model-file validation, deterministic replay, stale-generation regression, bounded resource metrics and device-specific tests. Native ODAS CI smoke is not a substitute for measured DOA accuracy. Production must never treat unknown source distance or silence as safe geometric evidence.

## 8. Current implementation vs this proposal

**Implemented:** \`audition_msgs\`, WAV replay, safe no-backend scene scaffold, and opt-in native ODAS conversion (tested with a synthetic six-channel WAV). An initial ReSpeaker profile exists but intentionally has no calibrated coordinates. Basic ROS Humble and native ODAS integration workflows have been run.

**Not implemented:** the generic runtime backend registry, automatic user YAML presets, ALSA capture in audition_ros2, a native semantic pipeline, class/event reconciliation, lifecycle reset/reload service, confidence calibration, multi-sensor fusion and navigation consumers.

This document describes the desired developer/user-facing experience and gives a concrete contract for further implementation; it must not be misread as completed functionality.
