# Advisory bearing-only attention proposal. NEVER a motion command or metric location.
std_msgs/Header header
bool valid
string source_id
uint64 source_generation
uint64 scene_generation
# True only when a validated unit bearing is available in header.frame_id.
geometry_msgs/Vector3 direction_unit
# Relative policy utility, not a calibrated probability.
float32 priority
string reason
