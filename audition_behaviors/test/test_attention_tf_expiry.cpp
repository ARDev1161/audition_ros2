#include "audition_behaviors/attention_tf_expiry.hpp"
#include <gtest/gtest.h>
using audition_behaviors::AttentionTfExpiry;
using namespace std::chrono_literals;
TEST(AttentionTfExpiry, RevokesOnceAfterSilence) {
  AttentionTfExpiry gate(500ms);
  auto start = AttentionTfExpiry::Clock::time_point{};
  EXPECT_FALSE(gate.expire(start + 10s));
  gate.observe(true, start);
  EXPECT_FALSE(gate.expire(start + 499ms));
  EXPECT_TRUE(gate.expire(start + 500ms));
  EXPECT_FALSE(gate.expire(start + 1s));
}
TEST(AttentionTfExpiry, FreshBearingExtendsDeadline) {
  AttentionTfExpiry gate(500ms);
  auto start = AttentionTfExpiry::Clock::time_point{};
  gate.observe(true, start);
  gate.observe(true, start + 400ms);
  EXPECT_FALSE(gate.expire(start + 700ms));
  EXPECT_TRUE(gate.expire(start + 900ms));
}
TEST(AttentionTfExpiry, InvalidInputImmediatelyRevokesAndDoesNotRepeat) {
  AttentionTfExpiry gate(500ms);
  auto start = AttentionTfExpiry::Clock::time_point{};
  gate.observe(true, start);
  gate.observe(false, start + 10ms);
  EXPECT_FALSE(gate.active());
  EXPECT_FALSE(gate.expire(start + 2s));
  gate.observe(true, start + 3s);
  EXPECT_TRUE(gate.expire(start + 3500ms));
}
TEST(AttentionTfExpiry, MonotonicClockRegressionDoesNotRevoke) {
  AttentionTfExpiry gate(500ms);
  auto start = AttentionTfExpiry::Clock::time_point{};
  gate.observe(true, start + 2s);
  EXPECT_FALSE(gate.expire(start + 1s));
}
