# Current acoustic perception snapshot, not a stored physical occupancy map.
std_msgs/Header header
uint64 generation
AcousticSource[] sources
# Active acoustic event hypotheses only; past events use the event topic.
AcousticEvent[] active_events
