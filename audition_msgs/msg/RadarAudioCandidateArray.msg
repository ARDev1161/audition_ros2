# Diagnostic unlatched snapshot. Empty candidates withdraw previous matches.
# No consumer may convert this message into LocalizedSource/obstacles by default.
std_msgs/Header header
uint64 sequence_number
string audio_scene_session_id
string radar_session_id
RadarAudioCandidate[] candidates
