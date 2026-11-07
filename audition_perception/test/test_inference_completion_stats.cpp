#include "audition_perception/inference_completion_stats.hpp"
#include <gtest/gtest.h>
#include <chrono>
using audition_perception::InferenceCompletionStats;
using Clock=InferenceCompletionStats::Clock;
TEST(InferenceCompletionStats, NoModelDoesNotFabricateProgress) {
  InferenceCompletionStats stats;
  auto value=stats.snapshot(Clock::time_point{}+std::chrono::hours(12));
  EXPECT_EQ(value.classifier_completed_windows,0U);
  EXPECT_EQ(value.classifier_last_completion_age_ms,0U);
  EXPECT_EQ(value.asr_completed_segments,0U);
  EXPECT_EQ(value.asr_last_completion_age_ms,0U);
}
TEST(InferenceCompletionStats, ClassifierCompletionAndStaleResultsDiffer) {
  InferenceCompletionStats stats;
  auto t=Clock::time_point{}+std::chrono::seconds(10);
  stats.classifierDiscarded(); stats.classifierCompleted(t); stats.classifierDiscarded();
  auto v=stats.snapshot(t+std::chrono::milliseconds(425));
  EXPECT_EQ(v.classifier_completed_windows,1U);
  EXPECT_EQ(v.classifier_discarded_results,2U);
  EXPECT_EQ(v.classifier_last_completion_age_ms,425U);
  stats.classifierCompleted(t+std::chrono::milliseconds(500));
  EXPECT_EQ(stats.snapshot(t+std::chrono::milliseconds(510)).classifier_last_completion_age_ms,10U);
}
TEST(InferenceCompletionStats, EmptyAsrResultIsStillCompletion) {
  InferenceCompletionStats stats;
  auto t=Clock::time_point{}+std::chrono::seconds(3);
  stats.asrCompleted(2,1,3,t);
  auto v=stats.snapshot(t+std::chrono::seconds(2));
  EXPECT_EQ(v.asr_completed_segments,2U);
  EXPECT_EQ(v.asr_empty_segments,1U);
  EXPECT_EQ(v.asr_discarded_results,3U);
  EXPECT_EQ(v.asr_last_completion_age_ms,2000U);
}
TEST(InferenceCompletionStats, StaleAsrCannotRefreshHeartbeat) {
  InferenceCompletionStats stats;
  auto t=Clock::time_point{}+std::chrono::seconds(3);
  stats.asrCompleted(1,0,0,t);
  stats.asrCompleted(0,0,5,t+std::chrono::seconds(4));
  EXPECT_EQ(stats.snapshot(t+std::chrono::seconds(5)).asr_last_completion_age_ms,5000U);
}
TEST(InferenceCompletionStats, FailureCountAndBackwardClock) {
  InferenceCompletionStats stats;
  auto t=Clock::time_point{}+std::chrono::seconds(3);
  stats.pollFailed();stats.pollFailed();stats.classifierCompleted(t);
  EXPECT_EQ(stats.snapshot(t-std::chrono::seconds(1)).classifier_last_completion_age_ms,0U);
  EXPECT_EQ(stats.snapshot().inference_poll_failures,2U);
}
