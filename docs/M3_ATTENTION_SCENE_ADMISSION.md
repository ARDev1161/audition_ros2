# M3ag — bounded scene admission

The advisory attention policy accepts a new optional startup parameter,
`maximum_scene_sources` (0 by default, meaning disabled; valid range 0–4096).
When enabled, an incoming `AcousticScene` with more source records than the
limit is rejected **before** any source-ID allocation, duplicate detection,
or quarantine reconciliation. Every record counts, including duplicate IDs
and records with invalid provenance.

The policy publishes an invalid advisory with reason `scene_source_limit`,
clears the active target and pending confirmations, and **does not advance**
the accepted scene generation/frame/timestamp watermark. Consequently a
later, correctly bounded and newer-than-last-accepted scene can recover
without a restart. The previously accepted stream fence still applies.

This is a resource-admission guard, not a hard upper bound on ROS message
deserialization or DDS transport memory. For stronger end-to-end resource
limits configure DDS/ROS middleware and producer-side limits as well.

Three policy GTests exercise fail-closed rejection and recovery, duplicate
record accounting, default compatibility and parameter validation.
The ROS node validates the parameter at startup. No robot motion is issued.
