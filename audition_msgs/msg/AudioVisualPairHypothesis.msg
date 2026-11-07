# A plausible pair, NOT verified speaker/face/person/source identity.
string audio_scene_session_id
string audio_source_id
uint64 audio_source_generation
string vision_session_id
string visual_track_id
uint64 visual_track_generation
builtin_interfaces/Time audio_observed_at
builtin_interfaces/Time visual_observed_at
# Only unique among presently eligible pair hypotheses; NOT a probability.
bool unique_match
float64 angle_difference_rad
# A relative residual, not calibrated confidence or probability.
float64 normalized_residual
float64 sensor_baseline_m
