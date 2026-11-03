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

TEST(SpeechObserver, ContinuousBacklogAgeSurvivesSizeFluctuation) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=4; s.buffered_results=2;
  o.sample(s,t);
  s.queued_segments=1; s.buffered_results=3;
  o.sample(s,t+std::chrono::seconds(3));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(4)),4000U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(4)),4000U);
  EXPECT_FALSE(o.queuePressure(2,3000,t+std::chrono::seconds(4)));
  EXPECT_TRUE(o.queuePressure(1,3000,t+std::chrono::seconds(4)));
}
TEST(SpeechObserver, DrainedQueueResetsAgeButNotMailbox) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=3; s.buffered_results=1;
  o.sample(s,t);
  s.queued_segments=0;
  o.sample(s,t+std::chrono::seconds(5));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(6)),0U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(6)),6000U);
  s.queued_segments=2;
  o.sample(s,t+std::chrono::seconds(7));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(8)),1000U);
}
TEST(SpeechObserver, FailedSampleAndResetFencePressure) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=10;
  o.sample(s,t);
  o.failed();
  EXPECT_FALSE(o.queuePressure(1,1,t+std::chrono::hours(1)));
  o.sample(s,t+std::chrono::hours(1));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::hours(1)),0U);
  o.reset();
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::hours(2)),0U);
}
TEST(SpeechObserver, ZeroThresholdsAndSilenceDoNotSignalPressure) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  o.sample({},t);
  EXPECT_FALSE(o.queuePressure(1,1000,t+std::chrono::hours(1)));
  SpeechWorkloadSnapshot s; s.queued_segments=10;
  o.sample(s,t);
  EXPECT_FALSE(o.queuePressure(0,1000,t+std::chrono::hours(1)));
  EXPECT_FALSE(o.queuePressure(1,0,t+std::chrono::hours(1)));
}

TEST(SpeechObserver, MailboxReappearanceStartsNewResidencyClock) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.buffered_results=3;
  o.sample(s,t);
  s.buffered_results=0;
  o.sample(s,t+std::chrono::seconds(2));
  s.buffered_results=1;
  o.sample(s,t+std::chrono::seconds(4));
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(5)),1000U);
}
TEST(SpeechObserver, RepeatedEmptySamplesDoNotCreateAge) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  o.sample({},t);
  o.sample({},t+std::chrono::seconds(10));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::hours(1)),0U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::hours(1)),0U);
}

TEST(SpeechObserver, StaleSingleSampleCannotBecomePressureByElapsedTime) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=8;
  o.sample(s,t);
  EXPECT_FALSE(o.queuePressure(4,10000,t+std::chrono::seconds(20)));
  EXPECT_FALSE(o.fresh(3000,t+std::chrono::seconds(20)));
  EXPECT_EQ(o.contiguousQueueSamples(),1U);
}
TEST(SpeechObserver, MultipleFreshSamplesProducePressure) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=5;
  for (int i=0;i<=10;++i) o.sample(s,t+std::chrono::seconds(i));
  EXPECT_TRUE(o.queuePressure(4,10000,t+std::chrono::seconds(10)));
  EXPECT_FALSE(o.queuePressure(4,10000,t+std::chrono::seconds(14)));
  EXPECT_EQ(o.contiguousQueueSamples(),11U);
}
TEST(SpeechObserver, DrainFailureAndResetClearEvidence) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=5;
  o.sample(s,t);
  o.sample(s,t+std::chrono::seconds(1));
  s.queued_segments=0;
  o.sample(s,t+std::chrono::seconds(2));
  EXPECT_EQ(o.contiguousQueueSamples(),0U);
  s.queued_segments=5;
  o.sample(s,t+std::chrono::seconds(3));
  EXPECT_EQ(o.contiguousQueueSamples(),1U);
  o.failed();
  EXPECT_EQ(o.contiguousQueueSamples(),0U);
  o.sample(s,t+std::chrono::seconds(4));
  o.reset();
  EXPECT_EQ(o.contiguousQueueSamples(),0U);
}
TEST(SpeechObserver, ClockRegressionCannotClaimFreshness) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{}+std::chrono::seconds(10);
  SpeechWorkloadSnapshot s; s.queued_segments=5;
  o.sample(s,t);
  EXPECT_FALSE(o.fresh(3000,t-std::chrono::seconds(1)));
}
