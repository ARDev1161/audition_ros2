#include "audition_perception/inference_workload_stats.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace audition_perception;
TEST(InferenceWorkload, DisabledWorkersHaveNoFabricatedJobsOrTimings) {
  ClassifierWorkloadAccumulator acc;
  const auto s=acc.snapshot();
  EXPECT_EQ(s.active_sources,0U);
  EXPECT_EQ(s.queued_sources,0U);
  EXPECT_EQ(s.inferencing_sources,0U);
  EXPECT_EQ(s.dropped_windows_active,0U);
  EXPECT_FALSE(s.have_inference_timing);
  EXPECT_FALSE(s.have_queue_wait_timing);
  SpeechWorkloadSnapshot speech;
  EXPECT_EQ(speech.queued_segments,0U);
  EXPECT_EQ(speech.completed_segments_native,0U);
  EXPECT_FALSE(speech.worker_error);
}
TEST(InferenceWorkload, AggregatesCurrentSourceStatesWithoutInventingQueueDepth) {
  ClassifierWorkloadAccumulator acc;
  acc.observe(ClassifierWorkState::Collecting,2,{},{});
  acc.observe(ClassifierWorkState::Queued,4,12.5,5.25);
  acc.observe(ClassifierWorkState::Inferencing,1,31.0,7.0);
  acc.observe(ClassifierWorkState::Classified,3,20.0,6.0);
  const auto s=acc.snapshot();
  EXPECT_EQ(s.active_sources,4U);
  EXPECT_EQ(s.collecting_sources,1U);
  EXPECT_EQ(s.queued_sources,1U);
  EXPECT_EQ(s.inferencing_sources,1U);
  EXPECT_EQ(s.classified_sources,1U);
  EXPECT_EQ(s.dropped_windows_active,10U);
  EXPECT_TRUE(s.have_inference_timing);
  EXPECT_TRUE(s.have_queue_wait_timing);
  EXPECT_DOUBLE_EQ(s.max_last_inference_ms,31.0);
  EXPECT_DOUBLE_EQ(s.max_last_queue_wait_ms,7.0);
}
TEST(InferenceWorkload, ErrorAndWaitingAreNotInferenceProgress) {
  ClassifierWorkloadAccumulator acc;
  acc.observe(ClassifierWorkState::Waiting,0,{},{});
  acc.observe(ClassifierWorkState::Error,9,{},{});
  const auto s=acc.snapshot();
  EXPECT_EQ(s.waiting_sources,1U);
  EXPECT_EQ(s.error_sources,1U);
  EXPECT_EQ(s.queued_sources,0U);
  EXPECT_EQ(s.inferencing_sources,0U);
  EXPECT_EQ(s.dropped_windows_active,9U);
  EXPECT_FALSE(s.have_inference_timing);
}
TEST(InferenceWorkload, MissingOrInvalidTimingIsNotMeasuredZero) {
  ClassifierWorkloadAccumulator acc;
  acc.observe(ClassifierWorkState::Queued,0,std::numeric_limits<double>::quiet_NaN(),-2.0);
  acc.observe(ClassifierWorkState::Inferencing,0,std::numeric_limits<double>::infinity(),{});
  EXPECT_FALSE(acc.snapshot().have_inference_timing);
  EXPECT_FALSE(acc.snapshot().have_queue_wait_timing);
  acc.observe(ClassifierWorkState::Classified,0,0.0,0.0);
  EXPECT_TRUE(acc.snapshot().have_inference_timing);
  EXPECT_TRUE(acc.snapshot().have_queue_wait_timing);
  EXPECT_DOUBLE_EQ(acc.snapshot().max_last_inference_ms,0.0);
}
TEST(InferenceWorkload, ReloadStartsNewSnapshotWithoutStaleCounters) {
  ClassifierWorkloadAccumulator old;
  old.observe(ClassifierWorkState::Queued,100,300.0,100.0);
  ClassifierWorkloadAccumulator fresh;
  EXPECT_EQ(fresh.snapshot().dropped_windows_active,0U);
  EXPECT_FALSE(fresh.snapshot().have_inference_timing);
}
