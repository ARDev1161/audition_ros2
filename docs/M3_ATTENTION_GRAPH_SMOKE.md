# M3o — real ROS graph attention integration smoke

The Humble workflow now launches the installed `m3_attention.launch.py` in
`/robot_ci/audition` with a 500 ms timeout. A separate rclpy process publishes
canonical `AcousticScene` messages on the namespaced `scene` topic and
subscribes to the actual `attention/target` output.

The test waits for DDS publisher/subscriber discovery, then verifies:
- a valid ACTIVE unit-bearing source produces a typed advisory with matching
  source identity, generation, frame and direction;
- an accepted empty scene immediately publishes `valid=false`;
- a newer generation is independently selectable;
- after publishing stops, the wall-clock expiry invalidates the last target
  and no valid target reappears without new observations.

This is a real interprocess ROS2 launch/serialization/QoS/namespace/timer
check, **not** a unit-only policy test or evidence of microphone direction
accuracy. The source is synthetic, injected at the canonical scene boundary;
no motion or Nav2 actions are exercised. The CI uses a separate ROS_DOMAIN_ID
and dumps launch logs and graph state on failure. Existing defaults stay off.

## M3p: real ROS graph negative cases and Humble discovery

ROS 2 Humble rclpy Subscription has no `get_publisher_count()`. The readiness
barrier now queries `node.count_publishers()` for the fully-qualified
advisory topic, retaining the publisher's subscription count and bounded
35-second wait. The previous M3o CI failure occurred before message assertions.
The smoke also rejects replayed old generations and zero-length bearings,
checks that neither becomes actionable, reacquires a valid source, and verifies
timer expiry. No motion or Nav2 commands are exercised.
