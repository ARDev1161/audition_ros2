# Normalized, finite float32 PCM. The stamp is the capture time of frame 0.
# header.stamp MUST be normalized into the active ROS time domain. A separate
# acquisition clock should be mapped explicitly before publication.
# header.frame_id identifies the physical capture device / microphone array.
std_msgs/Header header
uint64 sequence_number
uint32 sample_rate_hz
uint32 channel_count
uint32 frame_count
uint8 INTERLEAVED=0
uint8 PLANAR=1
uint8 layout
# Original capture channel indices, one entry per channel in the PCM.
uint32[] channel_indices
# Exactly frame_count * channel_count samples, in the declared layout.
float32[] pcm
