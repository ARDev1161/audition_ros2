# Advisory camera aim, not an actuator command. A separate authorized controller
# must validate this proposal against camera limits, safety and current state.
std_msgs/Header header
bool valid
string source_id
uint64 source_generation
uint64 scene_generation
float64 pan_rad
float64 tilt_rad
string reason
