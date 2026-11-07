#include "audition_behaviors/attention_tf_geometry.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <string>

using audition_msgs::msg::AttentionTarget;
using audition_behaviors::valid_attention_observation;
using audition_behaviors::rotate_attention_direction;

static AttentionTarget sample() {
  AttentionTarget t;
  t.valid = true;
  t.header.frame_id = "microphone";
  t.header.stamp.sec = 10;
  t.direction_unit.x = 1.0;
  return t;
}
TEST(AttentionTfGeometry, FreshAndStale) {
  auto t = sample();
  std::string reason;
  EXPECT_TRUE(valid_attention_observation(t, 10200000000LL, 500, reason));
  EXPECT_FALSE(valid_attention_observation(t, 10600000000LL, 500, reason));
  EXPECT_EQ(reason, "stale_or_missing_stamp");
  EXPECT_FALSE(valid_attention_observation(t, 9000000000LL, 500, reason));
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 0, reason));
  t.header.stamp.sec = 0;
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 500, reason));
}
TEST(AttentionTfGeometry, InvalidInput) {
  auto t = sample();
  std::string reason;
  t.valid = false;
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 500, reason));
  EXPECT_EQ(reason, "upstream_invalid");
  t.valid = true;
  t.direction_unit.x = 2.0;
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 500, reason));
  EXPECT_EQ(reason, "invalid_bearing");
  t.direction_unit.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 500, reason));
  t.direction_unit.x = 1.0;
  t.header.frame_id.clear();
  EXPECT_FALSE(valid_attention_observation(t, 10200000000LL, 500, reason));
}
TEST(AttentionTfGeometry, RotationOnlyNoTranslation) {
  geometry_msgs::msg::Vector3 input, output;
  geometry_msgs::msg::Quaternion q;
  input.x = 1.0;
  q.z = std::sqrt(0.5);
  q.w = std::sqrt(0.5);
  ASSERT_TRUE(rotate_attention_direction(input, q, output));
  EXPECT_NEAR(output.x, 0.0, 1e-9);
  EXPECT_NEAR(output.y, 1.0, 1e-9);
  EXPECT_NEAR(output.z, 0.0, 1e-9);
  q.w = 0.0;
  EXPECT_FALSE(rotate_attention_direction(input, q, output));
  q.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rotate_attention_direction(input, q, output));
}
