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

## M3s: asynchronous advisory correlation in the strict graph smoke

The first PR Humble M3r run passed 225 unit tests and the earlier attention
smoke, but the strict smoke saw an unsolicited `no_scene` timer advisory
immediately after DDS graph discovery. The test incorrectly treated the first
received message as the response to its first scene. It now correlates
advisories by scene header stamp, frame and generation, and retries the first
scene until the real node has accepted it. This does not weaken the strict
replay or challenger assertions; the production policy is unchanged.
