#include "audition_perception/speech_workload_observer.hpp"
#include <gtest/gtest.h>
using namespace audition_perception;
using Clock = SpeechWorkloadObserver::Clock;
TEST(SpeechObserver, NoSampleDoesNotInventHealth) {
  SpeechWorkloadObserver o;
  EXPECT_FALSE(o.valid());
  EXPECT_FALSE(o.everSampled());
  EXPECT_EQ(o.current().queued_segments,0U);
  EXPECT_EQ(o.ageMs(Clock::time_point{}+std::chrono::hours(1)),0U);
}
TEST(SpeechObserver, TracksBacklogAndPeakIndependentlyOfStatusRequests) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot first; first.queued_segments=8; first.buffered_results=3;
  o.sample(first,t);
  first.queued_segments=2; first.buffered_results=1;
  o.sample(first,t+std::chrono::seconds(2));
  EXPECT_EQ(o.samples(),2U);
  EXPECT_EQ(o.current().queued_segments,2U);
  EXPECT_EQ(o.peakQueued(),8U);
  EXPECT_EQ(o.peakBuffered(),3U);
  EXPECT_EQ(o.ageMs(t+std::chrono::seconds(3)),1000U);
}
TEST(SpeechObserver, SamplingFailureInvalidatesCurrentButKeepsHistory) {
  SpeechWorkloadObserver o;
  SpeechWorkloadSnapshot s; s.queued_segments=5; s.worker_error=true;
  o.sample(s);
  o.failed();
  EXPECT_FALSE(o.valid());
  EXPECT_TRUE(o.everSampled());
  EXPECT_EQ(o.failures(),1U);
  EXPECT_EQ(o.peakQueued(),5U);
  EXPECT_EQ(o.current().queued_segments,0U);
  EXPECT_FALSE(o.current().worker_error);
}
TEST(SpeechObserver, ResetFencesOldWorkerIncarnation) {
  SpeechWorkloadObserver o;
  SpeechWorkloadSnapshot s; s.queued_segments=10;
  o.sample(s);
  o.reset();
  EXPECT_FALSE(o.valid());
  EXPECT_FALSE(o.everSampled());
  EXPECT_EQ(o.peakQueued(),0U);
  EXPECT_EQ(o.samples(),0U);
  EXPECT_EQ(o.failures(),0U);
}
TEST(SpeechObserver, ClockRegressionIsClamped) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{}+std::chrono::seconds(10);
  o.sample({},t);
  EXPECT_EQ(o.ageMs(t-std::chrono::seconds(1)),0U);
}
