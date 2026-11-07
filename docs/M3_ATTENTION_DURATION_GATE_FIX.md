# M3al — acquisition minimum-duration regression fix

M3ak introduced a minimum steady-clock acquisition duration, but the policy
returned `acquisition_pending` only while the count was insufficient. Once
the count reached its threshold, the target was selected prematurely.

The acquisition state machine now waits until **both** the configured number
of accepted observations and the configured elapsed duration have been met.
It keeps the first observation's steady-clock timestamp as the reference,
without incrementing the confirmation counter beyond the configured maximum.
A nonzero duration also activates the gate when `acquisition_confirmations=1`.
The default (one confirmation, zero duration) remains unchanged.

CI failures reproduced by `AcquisitionNeedsElapsedTimeAsWellAsCount` and
`AcquisitionTimeGateRestartsOnCandidateChange` motivated this fix. Added
regressions cover duration-only acquisition and repeated early observations.
No motion commands are generated.
