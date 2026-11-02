# M2c — validated startup profiles and capabilities (ROS 2 Humble)

This is an **opt-in startup-only configuration layer**. Existing M1/M2
launch files remain available. The resolver rejects invalid configuration
*before* starting capture, spatial or semantic nodes; it does not silently
substitute disabled backends, synthetic microphone coordinates or model files.

## Run a mono WAV replay

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch audition_bringup m2_profile.launch.py \
  profile:=mono_speech wav_path:=/absolute/path/to/mono-16k.wav
ros2 service call /audition/semantic_perception/get_capabilities \
  audition_msgs/srv/GetCapabilities "{}"
~~~

The built-in mono profile has speech_enabled: false until model weights are
installed. mono_speech means the mono **route is ready**, not that ASR is
enabled or working without weights. replay_ci is a model-free deterministic
transport test, not an acoustic quality benchmark.

## Enable native models

Install libaudition with the relevant optional native targets and compile
audition_perception with AUDITION_ROS2_WITH_SEMANTIC=ON,
AUDITION_ROS2_WITH_YAMNET=ON and/or AUDITION_ROS2_WITH_SPEECH=ON.
Create a copy of the YAML profile with, for example:

~~~yaml
schema_version: 1
name: custom_mono
capture:
  kind: wav
  wav_path: ""                # overridden on the command line
  frame_id: microphone_array
  channels: 1
  sample_rate_hz: 16000
  frames_per_message: 160
spatial:
  backend: mono
semantic:
  classifier_backend: yamnet
  yamnet_model: yamnet.onnx
  yamnet_labels: labels.txt
  speech_enabled: true
  speech_vad_backend: sherpa
  speech_asr_backend: whispercpp
  speech_vad_model: silero_vad.onnx
  speech_asr_model: ggml-base.bin
  speech_language: ru
~~~

Then:

~~~bash
ros2 launch audition_bringup m2_profile.launch.py \
  profile:=/absolute/path/to/custom_mono.yaml \
  wav_path:=/absolute/path/to/mono-16k.wav \
  models_dir:=/absolute/path/to/models
~~~

Model names must be explicit. Relative names resolve under models_dir,
which must exist. Absolute model paths are also supported. A missing file,
unknown key/backend, invalid numeric limit, unsupported sample rate, wrong
WAV channel count or incompatible mono route aborts preflight. Native model
load can still fail if the model is corrupt or the backend was not compiled;
the semantic node then fails rather than pretending to run a classifier.
No model download is performed.

The discover_models(directory) helper lists at most 128 candidate
.onnx, .gguf, .bin, .txt files for user-facing inspection;
**it never guesses which file to use**. The launch does not scan and
auto-select a model.

## ODAS and calibrated geometry

spatial_events and robot_full are **intentionally not runnable out of
the box**: the provided ReSpeaker example has geometry_calibrated: false.
Supply a separate spatial ROS YAML file with measured microphone_positions_m,
unique input_channel_indices, matching microphone_frame and 16 kHz rate,
then point spatial.config at that file. Preflight checks structural geometry
and non-collinearity. This does not establish measured DOA accuracy.

For ALSA, set capture.kind: alsa and supply device:=hw:...; channel count
and sample rate are explicitly configured. For WAV, the RIFF fmt header is
read and checked against the profile without loading the recording.
Only the existing 16 kHz mono and ODAS source-audio routes support semantic
inference. spatial.backend: disabled does not generate source audio.

## Typed capability service

The running semantic component serves:

- /audition/semantic_perception/get_capabilities
- Type: audition_msgs/srv/GetCapabilities (empty request)

The response reports compiled ODAS/YAMNet/Sherpa+Whisper support, currently
active semantic stages, model *names* loaded at successful startup, profile
name, declared spatial backend and runtime_reconfiguration_supported=false.
It does not expose filesystem paths or assert spatial engine health,
classifier calibration, model accuracy or ReSpeaker performance. A disabled
stage has no fabricated inference result.

**All semantic parameters are immutable after construction.** ROS
set_parameters returns a failure explaining that restart is required;
changing model paths, provider, graph or thresholds cannot silently leave
old workers active. A transactional ReloadPipeline service with source
generation invalidation/rollback remains future work. This also means scalar
thresholds are not yet hot-changeable.

## Known scope limitations

The M2c launcher currently uses the existing fixed /audition namespace,
not fleet-wide namespaced instances. It does not validate actual ALSA hardware
availability before launch, and it does not benchmark inference queues,
physical localization, ASR accuracy or model load latency. It is a safer
configuration and observability entry point, **not a production readiness gate**.

Tests cover malformed YAML, missing models, channel/rate mismatch, uncalibrated
ODAS geometry, explicit model resolution, service serialization, immutable
parameters and a live model-free ROS graph. Native backend CI remains separate.
