# M3u: implicit attention expiry on scene admission

The policy previously expired an incumbent only from its periodic timer. A
new scene delivered after a stalled ROS executor could be processed before
the timer callback, retaining an obsolete incumbent through switch hysteresis
or carrying pending challenger confirmations across an unobserved gap.

Before validating a newly delivered scene, the policy now invalidates the
current target and pending challenger if the steady observation clock moved
backwards or the time since the previous accepted scene exceeds
`scene_timeout_ms`. The accepted generation/frame/timestamp watermark is
preserved: older generations and (when strict mode is enabled) repeated or
older stamps still cannot reactivate a stale stream. A valid new scene can
immediately acquire a fresh target; no prior bearing remains actionable.

The timeout boundary is strictly greater than the configured duration,
matching the existing timer behavior. Four pure-policy regressions cover
executor stall, clock rollback, generation replay and the exact boundary.
No motion control or Nav2 action is introduced.
