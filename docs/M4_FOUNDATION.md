# M4: multimodal fusion foundation

Branch: `feat/m4-multimodal-fusion`, based on merged M0–M3 main.

Initial foundation: pure geometry gate for two **independently associated**
world-frame 2-D bearings. A single bearing does not supply metric range.
Two observations at the same position do not supply triangulation baseline.
Parallel or near-parallel rays, non-unit/nonfinite directions, and
behind-sensor intersections are rejected.

This is deliberately **not** a localization publisher: it does not know
association, timestamps, TF calibration, covariance, reflection likelihood,
or source motion. Those gates must be implemented before exposing metric
`LocalizedSource` results. In particular, no bearing-only source may
become a Nav2 obstacle. Tests exercise intersection, insufficient baseline,
poor conditioning, reversed rays and invalid inputs.

Next: explicit observation contract, time and source association, stamped
TF2 transformation, covariance and ambiguity preservation, then benchmark
against independently annotated audio/visual ground truth.
