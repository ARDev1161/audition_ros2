#include "audition_perception/source_lifecycle.hpp"
#include <gtest/gtest.h>
#include <chrono>

using audition_perception::SourceLifecycle;
using audition_msgs::msg::AcousticSource;
using audition_msgs::msg::AcousticSourceArray;
using namespace std::chrono_literals;

namespace {
AcousticSourceArray source(std::uint64_t generation = 1, const char * id = "a") {
  AcousticSourceArray snapshot;
  snapshot.header.frame_id = "mic";
  snapshot.generation = generation;
  AcousticSource s;
  s.source_id = id;
  s.source_generation = generation;
  s.state = AcousticSource::ACTIVE;
  s.has_direction = true;
  s.direction_unit.x = 1;
  snapshot.sources.push_back(s);
  return snapshot;
}
}

TEST(SourceLifecycle, MissingTrackIsLostThenEndedExactlyOnce) {
  SourceLifecycle lifecycle(200ms, 1000ms, 8);
  const auto t = SourceLifecycle::TimePoint{};
  auto first = lifecycle.observe(source(), t);
  ASSERT_EQ(first.scene.sources.size(), 1U);
  EXPECT_EQ(first.scene.sources[0].state, AcousticSource::ACTIVE);
  AcousticSourceArray empty;
  empty.header.frame_id = "mic";
  empty.generation = 1;
  auto lost = lifecycle.observe(empty, t + 50ms);
  ASSERT_EQ(lost.scene.sources.size(), 1U);
  EXPECT_EQ(lost.scene.sources[0].state, AcousticSource::LOST);
  auto no_transition = lifecycle.tick(t + 250ms, empty.header);
  EXPECT_FALSE(no_transition.changed);
  auto ended = lifecycle.tick(t + 1100ms, empty.header);
  ASSERT_EQ(ended.sources.sources.size(), 1U);
  EXPECT_EQ(ended.sources.sources[0].state, AcousticSource::ENDED);
  EXPECT_TRUE(ended.scene.sources.empty());
  auto again = lifecycle.tick(t + 1200ms, empty.header);
  EXPECT_TRUE(again.sources.sources.empty());
  EXPECT_FALSE(again.changed);
}

TEST(SourceLifecycle, ReacquisitionBeforeDeadlineRestoresActiveState) {
  SourceLifecycle lifecycle(200ms, 1000ms, 8);
  const auto t = SourceLifecycle::TimePoint{};
  lifecycle.observe(source(), t);
  AcousticSourceArray empty;
  empty.header.frame_id = "mic";
  empty.generation = 1;
  lifecycle.observe(empty, t + 100ms);
  auto recovered = lifecycle.observe(source(), t + 150ms);
  ASSERT_EQ(recovered.scene.sources.size(), 1U);
  EXPECT_EQ(recovered.scene.sources[0].state, AcousticSource::ACTIVE);
  auto later = lifecycle.tick(t + 400ms, empty.header);
  EXPECT_TRUE(later.changed);
  EXPECT_EQ(later.scene.sources[0].state, AcousticSource::LOST);
}

TEST(SourceLifecycle, StreamResetEndsOldGeneration) {
  SourceLifecycle lifecycle(100ms, 1000ms, 8);
  const auto t = SourceLifecycle::TimePoint{};
  lifecycle.observe(source(1, "old"), t);
  auto next = lifecycle.observe(source(2, "new"), t + 20ms);
  ASSERT_EQ(next.scene.sources.size(), 1U);
  EXPECT_EQ(next.scene.sources[0].source_id, "new");
  ASSERT_EQ(next.sources.sources.size(), 2U);
  EXPECT_EQ(next.sources.sources[0].source_id, "new");
  EXPECT_EQ(next.sources.sources[1].state, AcousticSource::ENDED);
  EXPECT_EQ(next.sources.sources[1].source_id, "old");
  EXPECT_FALSE(next.scene.sources[0].has_position);
}

TEST(SourceLifecycle, CaptureStallExpiresWithoutNewSnapshots) {
  SourceLifecycle lifecycle(200ms, 1000ms, 8);
  const auto t = SourceLifecycle::TimePoint{};
  lifecycle.observe(source(), t);
  const auto header = source().header;
  auto lost = lifecycle.tick(t + 250ms, header);
  EXPECT_TRUE(lost.changed);
  ASSERT_EQ(lost.scene.sources.size(), 1U);
  EXPECT_EQ(lost.scene.sources[0].state, AcousticSource::LOST);
  auto ended = lifecycle.tick(t + 1000ms, header);
  EXPECT_TRUE(ended.changed);
  EXPECT_TRUE(ended.scene.sources.empty());
  EXPECT_EQ(ended.sources.sources.back().state, AcousticSource::ENDED);
}

TEST(SourceLifecycle, RejectsDuplicateAndUnboundedInputWithoutMutation) {
  SourceLifecycle lifecycle(200ms, 1000ms, 1);
  auto invalid = source();
  invalid.sources.push_back(invalid.sources.front());
  EXPECT_THROW(lifecycle.observe(invalid, SourceLifecycle::TimePoint{}),
               std::invalid_argument);
  auto valid = lifecycle.observe(source(), SourceLifecycle::TimePoint{});
  EXPECT_EQ(valid.scene.sources.size(), 1U);
}


TEST(SourceLifecycle, FinalASRGraceIsBoundedAndGenerationScoped) {
  SourceLifecycle lifecycle(100ms, 600ms, 8, 2000ms);
  const auto t = SourceLifecycle::TimePoint{};
  lifecycle.observe(source(1, "speech"), t);
  EXPECT_TRUE(lifecycle.isLive("speech", 1));
  EXPECT_TRUE(lifecycle.allowsFinalTranscript("speech", 1, t));
  EXPECT_FALSE(lifecycle.isLive("unseen", 1));

  const auto header = source().header;
  auto ended = lifecycle.tick(t + 600ms, header);
  ASSERT_EQ(ended.sources.sources.size(), 1U);
  EXPECT_EQ(ended.sources.sources[0].state, AcousticSource::ENDED);
  EXPECT_FALSE(lifecycle.isLive("speech", 1));
  EXPECT_TRUE(lifecycle.allowsFinalTranscript("speech", 1, t + 2100ms));
  EXPECT_FALSE(lifecycle.allowsFinalTranscript("speech", 1, t + 2601ms));
  EXPECT_FALSE(lifecycle.allowsFinalTranscript("unseen", 1, t + 700ms));

  lifecycle.observe(source(2, "new"), t + 2700ms);
  EXPECT_FALSE(lifecycle.allowsFinalTranscript("speech", 1, t + 2700ms));
  EXPECT_TRUE(lifecycle.isLive("new", 2));
}

TEST(SourceLifecycle, MalformedSnapshotsCannotCorruptStreamOrSources) {
  SourceLifecycle lifecycle(100ms, 600ms, 8);
  const auto t = SourceLifecycle::TimePoint{};
  lifecycle.observe(source(3, "known"), t);
  const auto original = source(3, "known");
  auto invalid = source(3, "mismatch");
  invalid.sources.front().source_generation = 4;
  EXPECT_THROW(lifecycle.observe(invalid, t + 1ms), std::invalid_argument);
  invalid = source(3, "mismatch");
  invalid.sources.front().header.frame_id = "other";
  EXPECT_THROW(lifecycle.observe(invalid, t + 2ms), std::invalid_argument);
  invalid = source(3, "mismatch");
  invalid.header.frame_id = "other";
  EXPECT_THROW(lifecycle.observe(invalid, t + 3ms), std::invalid_argument);
  invalid = source(2, "stale");
  EXPECT_THROW(lifecycle.observe(invalid, t + 4ms), std::invalid_argument);
  EXPECT_TRUE(lifecycle.isLive("known", 3));
  EXPECT_FALSE(lifecycle.isLive("mismatch", 3));
  const auto recovered = lifecycle.observe(original, t + 5ms);
  ASSERT_EQ(recovered.scene.sources.size(), 1U);
  EXPECT_EQ(recovered.scene.sources.front().source_id, "known");
}
