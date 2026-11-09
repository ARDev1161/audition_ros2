# Diagnostic geometry-compatible pair only. NOT verified emitter identity.
string audio_source_id
uint64 audio_source_generation
string radar_track_id
uint64 radar_track_generation
builtin_interfaces/Time audio_observed_at
builtin_interfaces/Time radar_observed_at
# Range ALONG acoustic ray to the independently detected radar object.
# Not acoustic source range and not a source position estimate.
float64 radar_range_along_audio_ray_m
float64 lateral_residual_m
float64 normalized_residual
bool unique_match
uint32 supporting_synchronized_observations
# This means independently refreshed correlation, NOT speaker identity.
bool temporally_correlated
