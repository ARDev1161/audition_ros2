# Sensor-native radar track snapshot. header.frame_id is the radar frame.
# Each snapshot completely replaces the prior set from this incarnation.
std_msgs/Header header
string radar_session_id
uint64 sequence_number
RadarTrack[] tracks
