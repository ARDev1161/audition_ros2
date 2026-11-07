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
  // Keep every observation within M3d's three-second continuity window.
  // The queue drains, but the mailbox remains continuously occupied.
  s.queued_segments=0;
  o.sample(s,t+std::chrono::seconds(2));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(3)),0U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(3)),3000U);
  s.queued_segments=2;
  o.sample(s,t+std::chrono::seconds(4));
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(5)),1000U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(5)),5000U);
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

TEST(SpeechObserver, LongSamplingGapStartsNewQueueAndMailboxEvidence) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=8; s.buffered_results=2;
  o.sample(s,t);
  o.sample(s,t+std::chrono::seconds(1));
  o.sample(s,t+std::chrono::seconds(20));
  EXPECT_EQ(o.contiguousQueueSamples(),1U);
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::seconds(20)),0U);
  EXPECT_EQ(o.mailboxNonemptyAgeMs(t+std::chrono::seconds(20)),0U);
  EXPECT_FALSE(o.queuePressure(4,10000,t+std::chrono::seconds(20)));
}
TEST(SpeechObserver, BackwardSamplingClockFencesContinuity) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{}+std::chrono::seconds(10);
  SpeechWorkloadSnapshot s; s.queued_segments=8;
  o.sample(s,t);
  o.sample(s,t-std::chrono::seconds(1));
  EXPECT_EQ(o.contiguousQueueSamples(),1U);
  EXPECT_EQ(o.queueNonemptyAgeMs(t-std::chrono::seconds(1)),0U);
}
TEST(SpeechObserver, SamplingAtGapBoundaryKeepsEvidence) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=4;
  o.sample(s,t);
  o.sample(s,t+std::chrono::milliseconds(3000));
  EXPECT_EQ(o.contiguousQueueSamples(),2U);
  EXPECT_EQ(o.queueNonemptyAgeMs(t+std::chrono::milliseconds(3000)),3000U);
}
TEST(SpeechObserver, RepeatedGapsNeverAccumulatePressure) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=8;
  for (int i=0;i<5;++i) o.sample(s,t+std::chrono::seconds(10*i));
  EXPECT_EQ(o.contiguousQueueSamples(),1U);
  EXPECT_FALSE(o.queuePressure(4,10000,t+std::chrono::seconds(40)));
}

TEST(SpeechObserver, NativeLossDeltasExcludeInitialCounterBaseline) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.dropped_segments_native=100; s.evicted_results_native=7;
  o.sample(s,t);
  EXPECT_FALSE(o.lossOnLastSample());
  s.dropped_segments_native=103; s.evicted_results_native=9;
  o.sample(s,t+std::chrono::seconds(1));
  EXPECT_EQ(o.lastDroppedDelta(),3U);
  EXPECT_EQ(o.lastEvictedDelta(),2U);
  EXPECT_EQ(o.observedDroppedDeltaTotal(),3U);
  EXPECT_EQ(o.observedEvictedDeltaTotal(),2U);
  EXPECT_TRUE(o.lossOnLastSample());
}
TEST(SpeechObserver, NativeCounterRegressionDoesNotUnderflow) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.dropped_segments_native=100;
  o.sample(s,t);
  s.dropped_segments_native=2;
  o.sample(s,t+std::chrono::seconds(1));
  EXPECT_EQ(o.lastDroppedDelta(),0U);
  s.dropped_segments_native=4;
  o.sample(s,t+std::chrono::seconds(2));
  EXPECT_EQ(o.lastDroppedDelta(),2U);
  EXPECT_EQ(o.observedDroppedDeltaTotal(),2U);
}
TEST(SpeechObserver, SamplingFailureRestartsNativeCounterBaseline) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.evicted_results_native=10;
  o.sample(s,t);
  o.failed();
  s.evicted_results_native=1000;
  o.sample(s,t+std::chrono::seconds(1));
  EXPECT_EQ(o.lastEvictedDelta(),0U);
  s.evicted_results_native=1001;
  o.sample(s,t+std::chrono::seconds(2));
  EXPECT_EQ(o.lastEvictedDelta(),1U);
  o.reset();
  EXPECT_EQ(o.observedEvictedDeltaTotal(),0U);
}
TEST(SpeechObserver, UnchangedNativeCountersAreNotLoss) {
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.dropped_segments_native=3; s.evicted_results_native=4;
  o.sample(s,t);
  o.sample(s,t+std::chrono::seconds(1));
  EXPECT_FALSE(o.lossOnLastSample());
}

#include "audition_perception/asr_observation_config.hpp"
TEST(AsrObservationConfig, DefaultsMatchExistingPressurePolicy) {
  AsrObservationConfig c;
  EXPECT_NO_THROW(c.validate(1000));
  EXPECT_EQ(c.minimum_segments,4);
  EXPECT_EQ(c.minimum_age_ms,10000);
  EXPECT_EQ(c.max_sample_age_ms,3000);
}
TEST(AsrObservationConfig, RejectsUnobservableSamplingCadence) {
  AsrObservationConfig c;
  EXPECT_THROW(c.validate(5000),std::invalid_argument);
  c.max_sample_age_ms=10000;
  EXPECT_THROW(c.validate(10000),std::invalid_argument);
  c.minimum_age_ms=20000;
  EXPECT_NO_THROW(c.validate(10000));
}
TEST(AsrObservationConfig, RejectsInvalidThresholds) {
  AsrObservationConfig c;
  c.minimum_segments=0;
  EXPECT_THROW(c.validate(1000),std::invalid_argument);
  c.minimum_segments=1000001;
  EXPECT_THROW(c.validate(1000),std::invalid_argument);
  c.minimum_segments=4; c.minimum_age_ms=0;
  EXPECT_THROW(c.validate(1000),std::invalid_argument);
  c.minimum_age_ms=10000; c.max_sample_age_ms=0;
  EXPECT_THROW(c.validate(1000),std::invalid_argument);
}
TEST(AsrObservationConfig, ConfiguredPressureUsesFreshnessFence) {
  AsrObservationConfig c;
  c.minimum_segments=2; c.minimum_age_ms=4000; c.max_sample_age_ms=2000;
  c.validate(1000);
  SpeechWorkloadObserver o;
  const auto t=Clock::time_point{};
  SpeechWorkloadSnapshot s; s.queued_segments=2;
  for (int i=0;i<=4;++i) o.sample(s,t+std::chrono::seconds(i));
  EXPECT_TRUE(o.queuePressure(c.minimum_segments,c.minimum_age_ms,t+std::chrono::seconds(4),c.max_sample_age_ms));
  EXPECT_FALSE(o.queuePressure(c.minimum_segments,c.minimum_age_ms,t+std::chrono::seconds(7),c.max_sample_age_ms));
}
