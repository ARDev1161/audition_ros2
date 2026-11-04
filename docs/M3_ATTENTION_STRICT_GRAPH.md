# M3r — strict attention launch parity and end-to-end ROS smoke

The M3q `require_advancing_scene_stamp` option was accepted by the C++ node
but omitted from the M3n YAML preflight allowlist. The launch preflight now
accepts this option as a **real YAML boolean only**, rejecting integers,
strings and null. This makes the opt-in replay fence reachable through both
`m3_attention.launch.py` and `m2_profile.launch.py`.

The Humble CI additionally launches the real attention node with a strict
parameter YAML in a dedicated namespace and ROS domain. A separate rclpy
process publishes canonical scenes and checks the typed advisory output:
initial acquisition, three-snapshot challenger confirmation, duplicate
timestamp rejection without counting confirmation, reversal to the incumbent,
invalid zero timestamp rejection, and generation reset.

The test is deliberately independent of ODAS hardware, ASR models, or robot
actuators. No Nav2 action or motion command is introduced. The strict policy
is still opt-in; existing profile defaults remain unchanged.
