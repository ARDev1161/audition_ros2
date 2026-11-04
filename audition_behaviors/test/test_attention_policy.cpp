#include "audition_behaviors/attention_policy.hpp"

#include <gtest/gtest.h>

using audition_behaviors::AttentionPolicy;
using audition_msgs::msg::AcousticScene;
using audition_msgs::msg::AcousticSource;
using namespace std::chrono_literals;

static AcousticSource source(const char * id, float score, std::uint64_t generation = 1) {
  AcousticSource s;
  s.source_id = id;
  s.source_generation = generation;
  s.header.frame_id = "mic";
  s.state = AcousticSource::ACTIVE;
  s.has_direction = true;
  s.direction_unit.x = 1.0;
  s.activity_score = score;
  return s;
}
static AcousticScene scene(std::uint64_t generation = 1) {
  AcousticScene s;
  s.header.frame_id = "mic";
  s.generation = generation;
  return s;
}
TEST(AttentionPolicy, SelectAndHysteresis) {
  AttentionPolicy p({});
  auto s = scene();
  s.sources = {source("a", 0.7f), source("b", 0.6f)};
  auto now = AttentionPolicy::Clock::now();
  EXPECT_EQ(p.observe(s, now).source_id, "a");
  s.sources[1].activity_score = 0.8f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
}
TEST(AttentionPolicy, InvalidAndStaleAreNotActionable) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, now).valid);
  EXPECT_FALSE(p.expire(now + 1501ms).valid);
  EXPECT_FALSE(p.expire(now + 1600ms).valid);
  s.sources[0].has_direction = false;
  EXPECT_FALSE(p.observe(s, now + 1700ms).valid);
}
TEST(AttentionPolicy, GenerationAndFrameFence) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, now).valid);
  s.generation = 2;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources = {source("b", 0.8f, 2)};
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
  s.header.frame_id = "other";
  EXPECT_FALSE(p.observe(s, now + 300ms).valid);
}
TEST(AttentionPolicy, RejectMalformedBearingAndEnded) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  auto a = source("a", 0.9f);
  a.direction_unit.x = 0.0;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, now).valid);
  a = source("a", 0.9f);
  a.state = AcousticSource::ENDED;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
}
TEST(AttentionPolicy, ConfigValidation) {
  AttentionPolicy::Config c;
  c.scene_timeout = 0ms;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
  c.scene_timeout = 1500ms;
  c.switch_margin = -0.1;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
}
