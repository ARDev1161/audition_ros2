# M1 capture and scene scaffold — status

This implementation is **not** a complete spatial perception stack and is not
ready for a release gate.

## What works in this branch

- `audition_capture`: C++17 RIFF/WAV reader for uncompressed PCM16 or normalized
  IEEE float32, with bounded chunks and explicit channel metadata.
- `wav_capture` publishes timestamped `audition_msgs/msg/AudioFrame` at the
  file's nominal playback rate. Source timestamp means ROS replay time, not
  the original recording time (a generic WAV has no ROS clock identity).
- `spatial_perception`: validates PCM, explicitly requires
  `backend=disabled`, and publishes **empty** spatial source snapshots.
  It does not claim sound direction, identities or separation.
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

## What must happen next

1. Add deterministic audio frame validation tests and a source/clock generation
   reset contract.
2. Wire a real libaudition `ISpatialAudioEngine` instance to the spatial
   worker, with measured microphone geometry and exact backend hop size.
3. Implement bounded ingestion and source ID retirement, real separated frames,
   `AcousticEvent` assembly, and explicit stream disconnect/restart semantics.
4. Build a ReSpeaker HIL corpus to measure real-room angular errors, ID switches,
   TTL expiration and multi-source coverage.
5. Add the optional per-source semantic runtime with real models and event
   boundaries after spatial ownership contracts are stable.

See [ROS 2 node architecture](ROS2_ARCHITECTURE.md) and [engineering roadmap](ROADMAP.md).
