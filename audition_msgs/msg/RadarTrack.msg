# Independent radar tracker measurement in the snapshot's sensor frame.
# Radar range/position is measured by radar, never inferred from acoustic DOA.
string radar_track_id
uint64 radar_track_generation
bool active
geometry_msgs/Point position_m
# Row-major 3x3 metric position covariance in sensor frame (m^2).
float64[9] position_covariance_m2
builtin_interfaces/Time observed_at
