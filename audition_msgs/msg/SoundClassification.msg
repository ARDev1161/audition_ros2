# One observed source-local class score; not proof of a physical object.
std_msgs/Header header
string source_id
uint64 source_generation
string class_label
string taxonomy_version
string model_id

uint8 UNCALIBRATED_SCORE=0
uint8 CALIBRATED_PROBABILITY=1
uint8 score_kind
float32 score
