# Diagnostic 2-D bearing intersection only. NOT LocalizedSource, range or occupancy.
# World XY in header.frame_id; height and calibrated covariance are UNKNOWN.
std_msgs/Header header
string scene_session_id
string source_id
uint64 source_generation
builtin_interfaces/Time first_observed
builtin_interfaces/Time last_observed
float64 x_m
float64 y_m
float64 baseline_m
float64 parallax_rad
float64 maximum_ray_residual_m
uint32 supporting_observations
