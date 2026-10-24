# Observed speech segments; speaker ID is optional and unverified by default.
std_msgs/Header header
string source_id
string speaker_id
string model_id
string language
string text
bool is_final
builtin_interfaces/Time observed_start
builtin_interfaces/Time observed_end
uint8 UNCALIBRATED_SCORE=0
uint8 CALIBRATED_PROBABILITY=1
uint8 score_kind
bool has_score
float32 score
