# M3ak: monotonic minimum confirmation duration

Two optional attention-manager startup parameters add an elapsed-time requirement
to confirmation: `acquisition_minimum_duration_ms` and
`switch_minimum_duration_ms`. Both default to 0, preserving existing behavior.
Each must be nonnegative and no greater than its respective confirmation
window. A candidate must satisfy **both** the configured observation count
and the elapsed steady-clock duration from the first accepted observation.
A rapid burst of scene snapshots cannot immediately satisfy a nonzero time
gate. Once the count reaches the threshold, additional accepted observations
continue to be required to complete the elapsed-time gate; no timer promotes
an old candidate without new evidence.

Candidate changes, confirmation window expiry, direction discontinuity,
stream resets and invalidation restart the corresponding confirmation run.
This is advisory temporal stability only; it does not establish physical
speaker identity or authorize robot movement.

Four deterministic GTests cover acquisition, switching, candidate restart and
invalid duration/window combinations.
