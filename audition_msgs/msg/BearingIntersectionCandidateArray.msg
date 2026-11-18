# Unlatched diagnostic snapshots; empty array clears preceding candidates.
# This message is NOT an input to a Nav2 costmap or actuator controller.
std_msgs/Header header
uint64 sequence_number
string scene_session_id
BearingIntersectionCandidate[] candidates
