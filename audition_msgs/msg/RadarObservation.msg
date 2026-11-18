# Validated radar-origin metric observation transformed into world_frame.
# NOT an acoustic position or proof the radar track made the sound.
string radar_session_id
string radar_track_id
uint64 radar_track_generation
geometry_msgs/Point position_m
# Row-major 3x3 covariance rotated into the world frame (translation
# / extrinsic or pose uncertainty is NOT included; do not interpret as a
# calibrated global localization covariance).
float64[9] conditional_position_covariance_m2
builtin_interfaces/Time observed_at
