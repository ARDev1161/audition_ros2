# Separated mono PCM associated with a persistent acoustic source.
# Loss/reorder is tolerated by the semantic worker; source_generation
# rejects late frames after device/ODAS reset and ID retirement.
std_msgs/Header header
string source_id
uint64 source_generation
# Spatial producer incarnation, checked against canonical admission.
string spatial_session_id
uint64 sequence_number
uint32 sample_rate_hz
uint32 frame_count
float32[] pcm
