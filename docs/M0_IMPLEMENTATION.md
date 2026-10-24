# M0 implementation notes

This branch contains a **message-only ROS 2 package** and a proposed ReSpeaker
bringup profile. It does not yet implement capture, ODAS, semantic workers,
robot behaviors, TF fusion or a Nav2 plugin.

The node topology and future topic ownership are defined in
[ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md).

## Build on Ubuntu 22.04 with ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select audition_msgs --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select audition_msgs --event-handlers console_direct+
colcon test-result --verbose
```

The `audition_msgs` package should compile and expose messages using:

```bash
ros2 interface show audition_msgs/msg/AcousticScene
ros2 interface show audition_msgs/msg/SourceAudioFrame
```

## Contract notes

- `AudioFrame` is normalized float32 PCM with a capture timestamp.
- `SourceAudioFrame` is mono and carries a source ID plus generation.
- `AcousticSourceArray` provides spatial source observations and latest-state snapshots.
- `SoundClassification` carries model provenance and score semantics.
- `AcousticScene` is a snapshot, not an event log.
- `LocalizedSourceArray` is reserved for independently estimated metric locations.
- For both source identity and events, source generations prevent late inference
  from being attached to an unrelated newly allocated track.

**Important:** ROS message definitions do not automatically enforce invariants
such as unit bearing, finite PCM, non-negative covariance or
`pcm.size() == frame_count * channel_count`. They must be validated by
node ingress/egress adapters in M1. Contract round-trip tests only establish
that schemas serialize, not that physical or semantic constraints are proven.

The included ReSpeaker profile deliberately omits unmeasured microphone
coordinates. Until the geometry is measured, a spatial backend must refuse
to publish physically meaningful direction estimates.

## M0 exit checklist

- [ ] Humble CI proves generated messages build and round-trip.
- [x] Stable draft ROS messages for audio, identities, classes and events.
- [x] Written node ownership and topic topology; no fake extra nodes.
- [x] ReSpeaker channel selection profile with uncalibrated geometry.
- [ ] Independent libaudition C++17 installed-package adaptation test.
- [ ] Actual ReSpeaker sensor geometry + channel orientation measured.
- [ ] Replay fixtures and negative frame-length / timestamp validation.

Green CI will prove only the implemented contracts; it will not complete
the remaining measurement and integration gates.
