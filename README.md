# audition_ros2

ROS 2 integration and robot-facing acoustic perception built on [libaudition](https://github.com/ARDev1161/libaudition).

**Status:** M0 messages and an experimental M1 WAV replay/scaffold exist. ODAS localization, semantic inference, live hardware capture and navigation integration are **not yet implemented**.

## Interaction overview

![audition_ros2 node interaction diagram](docs/images/acoustic_interaction_flow.svg)

The M1 perception path has three ROS nodes: `audio_capture`, `spatial_perception`, and `acoustic_scene`. M2 adds `semantic_perception`; later components add attention, fusion and memory. A Nav2 costmap layer is a plugin, not an additional ROS node.

[Editable Mermaid diagram](docs/diagrams/acoustic_interaction_flow.mmd) · [Node responsibilities and interaction contracts](docs/ROS2_ARCHITECTURE.md)

## Design documents

- [Node architecture and interaction contracts](docs/ROS2_ARCHITECTURE.md) — how many nodes, who owns data, internal/public topics, process isolation, QoS and lifecycle.
- [M0 implementation notes](docs/M0_IMPLEMENTATION.md) — implemented messages, example ReSpeaker profile, build and pending gates.
- [Engineering roadmap](docs/ROADMAP.md) — near-term ROS 2 integration, future capabilities, dependencies, explicit non-goals, validation gates, and safety boundaries.
- [Human spatial hearing and robot perception](docs/HUMAN_SPATIAL_HEARING.md) — how biological hearing uses acoustic cues, motion, vision, and prior knowledge; what is worth reproducing in robotics and where engineering alternatives are preferable.

## Experimental M1 WAV replay

With ROS 2 Humble installed:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch audition_bringup m1_wav_pipeline.launch.py wav_path:=/absolute/path/to/audio.wav
# New sourced shell:
ros2 topic echo /audition/scene audition_msgs/msg/AcousticScene --once
```

Only uncompressed PCM16 and normalized float32 RIFF/WAV are currently supported. The spatial backend is explicitly **disabled**: it produces empty source observations, not fake DOA measurements. This runs transport and scene-adapter plumbing rather than a functional source localization pipeline.

See [M1 implementation status](docs/M1_IMPLEMENTATION.md).

## Intended boundaries

- **libaudition:** application-independent C++17 DSP, spatial acoustic estimation, classification, speech processing, source identity, and temporal analysis.
- **audition_ros2:** ROS 2 capture adapters, scheduling, interfaces, transforms, diagnostics, and acoustic-scene publication.
- **Robot applications:** perception fusion, active investigation, navigation, planning, and safety policies.

The presence of an acoustic event or an estimated direction does not establish a physical obstacle or a metric source position. See both design documents before implementing navigation integration.

Target baseline for initial design: ROS 2 Humble and Nav2, with cross-distribution compatibility considered where practical. Hardware-specific accuracy, latency, and reliability are **not yet validated**.
