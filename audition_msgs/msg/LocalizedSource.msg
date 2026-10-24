# Position belongs in header.frame_id (usually map/odom) and is separately
# established by range or multi-sensor/multi-pose fusion, NOT DOA alone.
std_msgs/Header header
string source_id
uint64 source_generation
geometry_msgs/Point position
float64[9] covariance
string localization_method
builtin_interfaces/Time last_observed
