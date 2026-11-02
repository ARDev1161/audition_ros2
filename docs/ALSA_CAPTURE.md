# Live ALSA capture and source lifecycle (experimental M1)

> This page documents the **implementation branch**, not files present in
> this documentation-only orphan branch. Source and launch scripts reside in
> [PR #1](https://github.com/ARDev1161/audition_ros2/pull/1).

## What was implemented

The `audition_capture` package now offers a native ALSA capture executable
beside the existing WAV replay program. ALSA support is **optional**, with
`AUDITION_ROS2_WITH_ALSA=OFF` by default. It requires `libasound2-dev`
at build time and a real ALSA capture device to run on the robot.

```bash
# In a checkout of the implementation branch, not documentation:
sudo apt-get install libasound2-dev
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DAUDITION_ROS2_WITH_ALSA=ON -DBUILD_TESTING=ON
source install/setup.bash

# Discover your real capture device (name and card index vary):
arecord -l
arecord -L

ros2 launch audition_bringup m1_alsa_pipeline.launch.py \
  device:=hw:2,0 sample_rate_hz:=16000 channel_count:=6 \
  spatial_backend:=disabled
```

The ALSA device `hw:2,0` above is **illustrative**, not a guaranteed
ReSpeaker card index. `spatial_backend:=disabled` intentionally publishes
an empty acoustic scene. Replace it with `odas` only after installing
native libaudition ODAS and supplying physically measured microphone geometry
in the `spatial_config` YAML.

## Data and timestamp contract

- Explicit 16-bit little-endian interleaved PCM, exact configured channel
  count and rate, with conversion to normalized finite float32.
- Every published `AudioFrame` contains the actual read frame count,
  original channel indices `0..N-1` and an increasing sequence number.
- A successful ALSA XRUN/suspend recovery skips a sequence number to signal
  a discontinuity to downstream ODAS. No missing samples are invented.
- Current ROS `header.stamp` is an **estimated first-frame time**, computed
  from the ROS clock at read completion minus chunk duration. It is not a
  calibrated USB hardware timestamp. Device latency, drift and clock
  synchronization remain unmeasured.
- The capture reader uses a nonblocking ALSA stream, bounded chunks and
  short waits; capture operates in its own thread. The real robot still needs
  performance, restart and unplug/replug tests.
- A `null` ALSA device in CI exercises message transport but is **not**
  evidence of microphone health or usable spatial measurements.

## Canonical source state transitions

`acoustic_scene` uses a separate bounded lifecycle policy:

| State | Meaning | When published |
| --- | --- | --- |
| `ACTIVE` | Source present in current spatial snapshot | Source array and scene |
| `LOST` | Source absent, or no fresh observation within timeout | Source array and scene |
| `ENDED` | Source expired or stream generation changed | One-shot on `/audition/sources`; removed from `/audition/scene` |

Initial node defaults are `source_lost_after_ms=300`,
`source_end_after_ms=1500`, `max_sources=16`. These are conservative
starting values and not tuned on ReSpeaker recordings. A 100-ms wall timer
updates lifetime even if upstream stops publishing. Reappearance before
`ENDED` restores the same source hypothesis. Source generation changes
invalidate the previous stream, while `SpeakerId` remains independent.

Tests include clock-driven state changes, reacquisition, generation resets
and invalid snapshot rejection. This logic is a ROS-side canonical lifecycle,
not a learned acoustic presence detector or a substitute for actual
`libaudition` persistent source resolution.

## Validation and limits

The code branch tests compile ALSA support, use a virtual `null` capture
device in ROS CI and run C++ unit tests. Native ODAS also has a separate
synthetic WAV CI. Check current [PR checks](https://github.com/ARDev1161/audition_ros2/pull/1/checks)
for the exact pass/fail status.

Before declaring M1 ready: run physical ReSpeaker S16_LE capture, measure
dropouts and end-to-end latency, calibrate geometry, test multi-source
separation, and assess source ID switches and presence at varying levels.
The `documentation` branch intentionally contains no C++, tests, ROS
packages or native binaries.
