# M3g — startup-only ASR pressure observation policy

`semantic_perception` now declares three startup-only integer parameters:

| Parameter | Default | Validation |
| --- | ---: | --- |
| `asr_pressure_minimum_segments` | 4 | 1..1000000 |
| `asr_pressure_minimum_age_ms` | 10000 | 1000..600000 and >= twice sample period |
| `asr_observation_max_sample_age_ms` | 3000 | sample period..30000 |

The sampling period is the existing
`classifier_workload_sample_period_ms` (200..10000). Invalid combinations
throw before the node begins publishing; live changes remain prohibited.
`GetSemanticStatus` exposes all three effective values.

The existing independent ASR observer, generation fences, contiguous-sample
requirement and three-second maximum continuity gap remain unchanged.
A sampling period above three seconds therefore cannot accumulate a sustained
pressure indication, even if the freshness threshold is increased. Choose a
period at or below three seconds for this diagnostic.

The pressure flag is only evidence of sampled queue occupancy; it does not
identify a hung inference job, trigger recovery or establish speech quality.
Four unit tests cover defaults, invalid cadence, bounds and configured
freshness/pressure behavior.
