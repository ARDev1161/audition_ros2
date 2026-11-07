#include "audition_behaviors/attention_tf_replay_fence.hpp"
#include <gtest/gtest.h>
using audition_behaviors::AttentionTfReplayFence;
using audition_msgs::msg::AttentionTarget;

static AttentionTarget sample(uint64_t generation, int seconds, const char * frame="mic") {
  AttentionTarget msg;
  msg.valid = true;
  msg.scene_generation = generation;
  msg.header.stamp.sec = seconds;
  msg.header.frame_id = frame;
  return msg;
}
TEST(AttentionTfReplayFence, RejectsDuplicateAndOlderStamp) {
  AttentionTfReplayFence gate;
  std::string reason;
  auto first = sample(4, 100);
  EXPECT_TRUE(gate.admissible(first, reason));
  gate.accept(first);
  EXPECT_FALSE(gate.admissible(first, reason));
  EXPECT_EQ(reason, "tf_replayed_observation");
  EXPECT_FALSE(gate.admissible(sample(4, 99), reason));
  EXPECT_TRUE(gate.admissible(sample(4, 101), reason));
}
TEST(AttentionTfReplayFence, RejectsOldGenerationEvenAfterLongSilence) {
  AttentionTfReplayFence gate;
  std::string reason;
  gate.accept(sample(10, 100));
  EXPECT_FALSE(gate.admissible(sample(9, 1000), reason));
  EXPECT_EQ(reason, "tf_regressed_generation");
  EXPECT_TRUE(gate.admissible(sample(11, 1), reason));
}
TEST(AttentionTfReplayFence, FrameChangeRequiresGenerationAdvance) {
  AttentionTfReplayFence gate;
  std::string reason;
  gate.accept(sample(7, 100));
  EXPECT_FALSE(gate.admissible(sample(7, 101, "other_mic"), reason));
  EXPECT_EQ(reason, "tf_frame_changed_without_generation");
  EXPECT_TRUE(gate.admissible(sample(8, 101, "other_mic"), reason));
}
TEST(AttentionTfReplayFence, RejectedInputDoesNotAdvanceWatermark) {
  AttentionTfReplayFence gate;
  std::string reason;
  gate.accept(sample(3, 100));
  EXPECT_FALSE(gate.admissible(sample(3, 99), reason));
  EXPECT_TRUE(gate.admissible(sample(3, 101), reason));
  // No accept() call: failed TF lookup cannot consume the timestamp.
  EXPECT_TRUE(gate.admissible(sample(3, 101), reason));
  gate.accept(sample(3, 101));
  EXPECT_FALSE(gate.admissible(sample(3, 101), reason));
}
