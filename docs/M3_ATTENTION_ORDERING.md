# M3i — Attention scene ordering and identity fencing

## M3i — ordered scene acceptance and duplicate-ID fencing

An old scene generation or an earlier header stamp in the same generation and
frame is rejected with an invalid advisory result, without refreshing the
retained target's freshness timer. The policy never reactivates a previous
stream generation. Repeated source IDs in a scene are treated as ambiguous
and excluded, rather than trusting the first or last occurrence. Changing
microphone frame clears the incumbent before selecting a new candidate.
The ROS node records invalid scene publications so the timer does not emit an
immediate duplicate invalidation. Four policy tests exercise these fences.
The policy is still an opt-in, bearing-only advisory with no Nav2 action.
