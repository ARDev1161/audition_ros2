# M3ai — aggregate identity admission budget

`attention_manager` supports `maximum_total_source_id_bytes` (default 0,
disabled; valid range 0–1048576). This opt-in budget sums source-ID byte
lengths across **all records in a single scene**, including duplicate,
inactive and otherwise invalid sources. Unlike the per-ID limit, this bounds
aggregate identity processing per scene before duplicate-ID hash tables and
quarantine reconciliation are constructed.

If the budget is exceeded, the policy publishes an invalid advisory with
`total_source_id_limit`, invalidates the incumbent and pending confirmations,
and preserves the accepted generation/frame/timestamp watermark. A subsequent
bounded accepted scene can recover. The sum is checked by subtracting from
a remaining budget, avoiding integer overflow.

This limit does not constrain DDS transport, ROS message deserialization,
source array allocation, or producer-side memory. Combine with
`maximum_scene_sources`, `maximum_source_id_bytes`, and middleware resource
limits where appropriate. Default behavior remains unchanged.

Three GTests cover exact-boundary acceptance, rejection/recovery and watermark
preservation, duplicate/ineligible accounting, and disabled/invalid config.
No motion commands are issued.
