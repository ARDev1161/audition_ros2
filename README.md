# audition_ros2

ROS 2 integration and robot-facing acoustic perception built on [libaudition](https://github.com/ARDev1161/libaudition).

**Status:** documentation and architecture planning; no ROS 2 implementation is claimed by this repository yet.

## Design documents

- [Engineering roadmap](docs/ROADMAP.md) — near-term ROS 2 integration, future capabilities, dependencies, explicit non-goals, validation gates, and safety boundaries.
- [Human spatial hearing and robot perception](docs/HUMAN_SPATIAL_HEARING.md) — how biological hearing uses acoustic cues, motion, vision, and prior knowledge; what is worth reproducing in robotics and where engineering alternatives are preferable.

## Intended boundaries

- **libaudition:** application-independent C++17 DSP, spatial acoustic estimation, classification, speech processing, source identity, and temporal analysis.
- **audition_ros2:** ROS 2 capture adapters, scheduling, interfaces, transforms, diagnostics, and acoustic-scene publication.
- **Robot applications:** perception fusion, active investigation, navigation, planning, and safety policies.

The presence of an acoustic event or an estimated direction does not establish a physical obstacle or a metric source position. See both design documents before implementing navigation integration.

Target baseline for initial design: ROS 2 Humble and Nav2, with cross-distribution compatibility considered where practical. Hardware-specific accuracy, latency, and reliability are **not yet validated**.
