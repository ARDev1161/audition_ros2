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
