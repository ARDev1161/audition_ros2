# audition_ros2

**ROS 2 integration for [libaudition](https://ardev1161.github.io/libaudition/)** — acoustic perception, spatial observations, speech events and robot-facing interfaces.

Audio algorithms remain in `libaudition`. This repository owns ROS 2 transport, lifecycle, configuration, TF2 and optional robot behavior. A sound bearing is **not** a metric position or an authorization to move.

## Architecture

```mermaid
flowchart TB
  CAP["audio_capture"] -->|Multichannel PCM| SP["spatial_perception"]
  SP -->|Separated audio per source| SEM["semantic_perception"]
  SP -->|Sources and bearings| SC["acoustic_scene"]
  SEM -->|Classes, events, transcripts| SC
  SC --> ATT["attention_manager"]
  SC --> FUS["multimodal_fusion (planned)"]
  CAM["OAK-D / radar / TF2 (optional)"] --> FUS
  FUS --> NAV["Nav2 consumers (planned)"]
  ATT --> ACT["Robot actions (planned)"]
```

## Start here

- [Engineering Roadmap](ROADMAP.md) — staged milestones and acceptance gates.
- [Architecture](ROS2_ARCHITECTURE.md) — ROS node ownership and data flow.
- [libaudition integration and configuration](LIBAUDITION_INTEGRATION.md) — supported and proposed backends.
- [M0 implementation](M0_IMPLEMENTATION.md) — message and interface baseline.
- [M1 implementation and subsequent increments](M1_IMPLEMENTATION.md) — capture, perception, semantics and attention.
- [Human spatial hearing](HUMAN_SPATIAL_HEARING.md) — perceptual rationale.

## Documentation status

This website documents work in progress. Green CI confirms automated software
checks, **not** real-microphone calibration, ASR accuracy or robot motion safety.
Features marked planned in the roadmap are not claimed as implemented.
