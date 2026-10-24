# Spatial estimates are expressed in header.frame_id (microphone array).
# An acoustic source ID, transient spatial track ID and speaker ID differ.
std_msgs/Header header
string source_id
uint64 source_generation
string spatial_track_id
string speaker_id

uint8 ACTIVE=0
uint8 LOST=1
uint8 ENDED=2
uint8 state

# Direction is a unit 3-D bearing, NOT a source position.
bool has_direction
geometry_msgs/Vector3 direction_unit
bool has_angular_variance
float64 angular_variance_rad2

# No distance/position should be inferred from DOA alone.
bool has_range
float64 range_m
float64 range_variance_m2
bool has_position
geometry_msgs/Point position
# Row-major 3x3 covariance in m^2, only when has_position.
float64[9] position_covariance

# Activity is an algorithm-native score, not a calibrated probability.
float32 activity_score
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
