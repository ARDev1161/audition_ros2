# M3aa — quarantine release requires actual identity absence

The direction quarantine is released only after an **accepted scene snapshot**
contains no record with the quarantined `source_id`. A record with the same
ID but an incorrect `source_generation` or microphone `frame_id` does
not prove the identity disappeared and must not release the quarantine.

This closes a bypass in which a foreign-generation record was interpreted as
absence, allowing a subsequent correctly tagged source to reacquire attention
without a genuinely absent scene. Regressed scene snapshots are also unable
to release the quarantine because they are rejected before reconciliation.

A genuinely absent ID releases independently of other quarantined sources.
Generation or scene-frame transition still clears all quarantines as an
explicit stream boundary. The feature remains opt-in via
`maximum_direction_jump_rad`, is bearing-only and cannot command robot
motion. Three new GTests exercise foreign-generation, foreign-frame and
regressed-scene sequences.
