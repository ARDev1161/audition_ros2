# Events describe observed sound activity; NOT object existence or hazards.
std_msgs/Header header
string event_id
string source_id
uint64 source_generation
string class_label
string taxonomy_version
string model_id

uint8 STARTED=0
uint8 UPDATED=1
uint8 ENDED=2
uint8 CANCELLED=3
uint8 transition

uint8 UNCALIBRATED_SCORE=0
uint8 CALIBRATED_PROBABILITY=1
uint8 score_kind
float32 score
builtin_interfaces/Time observed_start
builtin_interfaces/Time observed_end
string termination_reason
