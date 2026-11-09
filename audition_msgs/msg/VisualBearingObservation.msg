# Sightline supplied by an independent camera tracker, not speaker identity.
string visual_track_id
uint64 visual_track_generation
bool active
# Activity must be independently observed, e.g. mouth motion; class != proof.
bool activity_evidence
geometry_msgs/Vector3 direction_unit
bool has_angular_variance
float64 angular_variance_rad2
builtin_interfaces/Time observed_at
