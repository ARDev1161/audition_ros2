#include "audition_perception/classifier_state_deadline_tracker.hpp"
#include <gtest/gtest.h>
using namespace audition_perception;
using Clock = ClassifierStateDeadlineTracker::Clock;
TEST(ClassifierDeadlines, QueuedAndInferencingAgeIndependently) {
  ClassifierStateDeadlineTracker t;
  const auto now = Clock::time_point{};
  t.observe("a", ClassifierWorkState::Queued, now);
  t.observe("b", ClassifierWorkState::Inferencing, now);
  t.finishSample();
  const auto s = t.snapshot(1000, 2000, now + std::chrono::milliseconds(2500));
  EXPECT_EQ(s.overdue_queued_sources, 1U);
  EXPECT_EQ(s.overdue_inferencing_sources, 1U);
  EXPECT_EQ(s.max_queued_age_ms, 2500U);
  EXPECT_EQ(s.max_inferencing_age_ms, 2500U);
}
TEST(ClassifierDeadlines, TransitionResetsOnlyItsOwnClock) {
  ClassifierStateDeadlineTracker t;
  const auto now = Clock::time_point{};
  t.observe("a", ClassifierWorkState::Queued, now);
  t.finishSample();
  t.observe("a", ClassifierWorkState::Inferencing, now + std::chrono::milliseconds(2000));
  t.finishSample();
  const auto s = t.snapshot(1000, 2000, now + std::chrono::milliseconds(2500));
  EXPECT_EQ(s.overdue_queued_sources, 0U);
  EXPECT_EQ(s.overdue_inferencing_sources, 0U);
  EXPECT_EQ(s.max_inferencing_age_ms, 500U);
}
TEST(ClassifierDeadlines, SilenceAndRetirementDoNotCreateStalls) {
  ClassifierStateDeadlineTracker t;
  const auto now = Clock::time_point{};
  t.observe("a", ClassifierWorkState::Waiting, now);
  t.finishSample();
  EXPECT_EQ(t.snapshot(1,1,now + std::chrono::hours(1)).overdue_queued_sources,0U);
  t.finishSample();
  EXPECT_EQ(t.snapshot(1,1,now + std::chrono::hours(1)).max_queued_age_ms,0U);
}
TEST(ClassifierDeadlines, ResetAndBackwardClockAreSafe) {
  ClassifierStateDeadlineTracker t;
  const auto now = Clock::time_point{};
  t.observe("a", ClassifierWorkState::Queued, now + std::chrono::seconds(10));
  t.finishSample();
  EXPECT_EQ(t.snapshot(1,1,now).max_queued_age_ms,0U);
  t.reset();
  EXPECT_EQ(t.snapshot(1,1,now + std::chrono::hours(1)).overdue_queued_sources,0U);
}
TEST(ClassifierDeadlines, SameStateDoesNotRefreshHeartbeat) {
  ClassifierStateDeadlineTracker t;
  const auto now = Clock::time_point{};
  t.observe("a", ClassifierWorkState::Inferencing, now);
  t.finishSample();
  t.observe("a", ClassifierWorkState::Inferencing, now + std::chrono::seconds(5));
  t.finishSample();
  EXPECT_EQ(t.snapshot(0,1000,now + std::chrono::seconds(6)).max_inferencing_age_ms,6000U);
}

TEST(ClassifierDeadlines, RepeatedIndependentSamplesPreserveAge) {
  ClassifierStateDeadlineTracker tracker;
  const auto t0 = Clock::time_point{};
  for (int second=0; second<4; ++second) {
    tracker.observe("source", ClassifierWorkState::Queued, t0+std::chrono::seconds(second));
    tracker.finishSample();
  }
  const auto s=tracker.snapshot(2000,30000,t0+std::chrono::seconds(3));
  EXPECT_EQ(s.overdue_queued_sources,1U);
  EXPECT_EQ(s.max_queued_age_ms,3000U);
}
TEST(ClassifierDeadlines, RetirementIsPrunedAtNextSample) {
  ClassifierStateDeadlineTracker tracker;
  const auto t0=Clock::time_point{};
  tracker.observe("old",ClassifierWorkState::Inferencing,t0);
  tracker.finishSample();
  tracker.observe("new",ClassifierWorkState::Collecting,t0+std::chrono::seconds(1));
  tracker.finishSample();
  EXPECT_EQ(tracker.snapshot(1,1,t0+std::chrono::seconds(5)).overdue_inferencing_sources,0U);
}

TEST(ClassifierDeadlines, DistinctNativeIdsDoNotInheritOldSourceAge) {
  ClassifierStateDeadlineTracker t;
  const auto t0=Clock::time_point{};
  t.observe("native-id-1",ClassifierWorkState::Inferencing,t0);
  t.finishSample();
  t.observe("native-id-2",ClassifierWorkState::Inferencing,t0+std::chrono::seconds(5));
  t.finishSample();
  EXPECT_EQ(t.snapshot(1000,1000,t0+std::chrono::seconds(5)).overdue_inferencing_sources,0U);
}
TEST(ClassifierDeadlines, InvalidatedSamplesMustResetStateHistory) {
  ClassifierStateDeadlineTracker t;
  const auto t0=Clock::time_point{};
  t.observe("id",ClassifierWorkState::Queued,t0);
  t.finishSample();
  t.reset();
  t.observe("id",ClassifierWorkState::Queued,t0+std::chrono::seconds(20));
  t.finishSample();
  EXPECT_EQ(t.snapshot(1000,1000,t0+std::chrono::seconds(20)).overdue_queued_sources,0U);
}
