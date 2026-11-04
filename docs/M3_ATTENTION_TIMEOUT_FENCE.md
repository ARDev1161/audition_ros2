# M3t — persistent attention replay fence across timeout

The attention policy now keeps the last accepted scene generation, microphone
frame, and header timestamp after a wall-clock scene timeout or malformed
scene. Only the actionable advisory and pending challenger confirmation are
invalidated. A delayed old generation (or a repeated/older timestamp in strict
mode) therefore cannot re-arm an expired target.

A **newer** scene in the same generation/frame can reacquire a target after
timeout. A new generation can also acquire normally. An explicit
`AttentionPolicy::reset()` remains the only API that deliberately clears the
high-watermark, for a new trusted session. The ROS node does not expose a
remote reset service.

This closes a replay window; it does **not** authenticate a scene publisher.
The behavior is always active, independent of optional strict timestamp
validation. Without `require_advancing_scene_stamp=true`, same-generation
identical timestamps retain their pre-existing compatibility semantics.

Four GTests cover timeout/recovery, malformed scene fencing, pending
confirmation cancellation, and deliberate explicit reset. No new ROS
parameters, motion commands, or changes to default launch activation.
