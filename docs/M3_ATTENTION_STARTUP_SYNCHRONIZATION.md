# M3ab: strict attention graph startup synchronization

The ROS 2 Humble PR workflow on M3aa failed the strict replay graph smoke
at its startup handshake: valid source `a` advisories were arriving but the
probe advanced its expected timestamp every 100 ms, faster than the
asynchronous ROS executor could correlate the response. The production
policy and GTests passed on the same SHA in the push workflow.

The probe now waits up to 750 ms for a response matching each published
startup scene's stamp and generation before trying the next unique stamp.
The overall handshake remains bounded at 15 seconds. Strict replay safety
is preserved: retries always advance stamps, and later replay/confirmation
assertions still run through the same ROS graph. This is a test harness
synchronization correction, not a change to production attention behavior.
