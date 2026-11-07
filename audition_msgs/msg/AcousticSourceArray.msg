# Spatial or canonical source snapshot (all source bearings in header.frame_id).
std_msgs/Header header
uint64 generation
# Canonical /sources snapshots have a strictly increasing process-local order.
# Internal spatial producers may leave zero; acoustic_scene assigns this.
uint64 sequence_number
# Internal spatial producers set a nonzero sequence and incarnation ID.
# Canonical scene preserves this ID and assigns its own sequence_number.
string spatial_session_id
AcousticSource[] sources

# Canonical acoustic_scene incarnation (32 lowercase hex chars).
# Internal spatial producers leave empty; only /sources sets this field.
string scene_session_id
