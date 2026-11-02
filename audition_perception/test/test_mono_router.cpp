#include "audition_perception/mono_router.hpp"
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using audition_msgs::msg::AudioFrame;
using audition_perception::MonoRouter;

namespace {
AudioFrame frame(std::uint64_t seq = 0, std::uint32_t nanosec = 0) {
  AudioFrame f;
  f.header.frame_id = "mic";
  f.capture_session_id = std::string(32, 'a');
  f.header.stamp.sec = 5;
  f.header.stamp.nanosec = nanosec;
  f.sequence_number = seq;
  f.sample_rate_hz = 16000;
  f.channel_count = 1;
  f.frame_count = 160;
  f.layout = AudioFrame::INTERLEAVED;
  f.channel_indices = {0};
  f.pcm.assign(160, 0.25f);
  return f;
}
}  // namespace

TEST(MonoRouter, CarriesRealSamplesAndNeverInventsSpatialEvidence) {
  MonoRouter router;
  const auto first = router.process(frame());
  ASSERT_EQ(first.sources.sources.size(), 1U);
  const auto & source = first.sources.sources.front();
  EXPECT_FALSE(source.source_id.empty());
  EXPECT_EQ(source.source_generation, 1U);
  EXPECT_TRUE(source.spatial_track_id.empty());
  EXPECT_TRUE(source.speaker_id.empty());
  EXPECT_FALSE(source.has_direction);
  EXPECT_FALSE(source.has_angular_variance);
  EXPECT_FALSE(source.has_range);
  EXPECT_FALSE(source.has_position);
  EXPECT_EQ(source.state, audition_msgs::msg::AcousticSource::ACTIVE);
  EXPECT_EQ(first.audio.source_id, source.source_id);
  EXPECT_EQ(first.audio.source_generation, first.sources.generation);
  EXPECT_EQ(first.audio.pcm, frame().pcm);
  EXPECT_EQ(first.audio.sequence_number, 0U);
  auto next = router.process(frame(1, 10000000));
  EXPECT_EQ(next.sources.generation, 1U);
  EXPECT_EQ(next.sources.sources.front().source_id, source.source_id);
  EXPECT_EQ(next.sources.sources.front().first_seen.sec, 5);
  EXPECT_EQ(next.sources.sources.front().first_seen.nanosec, 0U);
  EXPECT_EQ(next.sources.sources.front().last_seen.nanosec, 10000000U);
}

TEST(MonoRouter, SequenceLossAndClockResetFenceSemanticGenerations) {
  MonoRouter router;
  router.process(frame(0));
  const auto lost = router.process(frame(3, 30000000));
  EXPECT_EQ(lost.sources.generation, 2U);
  EXPECT_EQ(lost.audio.source_generation, 2U);
  EXPECT_EQ(lost.sources.sources.front().first_seen.nanosec, 30000000U);
  auto backward = router.process(frame(4, 10000000));
  EXPECT_EQ(backward.sources.generation, 3U);
  auto stalled = router.process(frame(5, 600000000));
  EXPECT_EQ(stalled.sources.generation, 4U);
  EXPECT_EQ(router.generation(), 4U);
}

TEST(MonoRouter, RejectsInvalidFramesWithoutMutatingEpoch) {
  MonoRouter router;
  const auto initial = router.process(frame(0));
  auto bad = frame(1, 10000000);
  bad.sample_rate_hz = 8000;
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.channel_count = 2;
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.channel_indices = {1};
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.pcm[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.header.stamp.nanosec = 1000000000U;
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.frame_count = 4097;
  EXPECT_THROW(router.process(bad), std::invalid_argument);
  const auto next = router.process(frame(1, 10000000));
  EXPECT_EQ(next.sources.generation, 1U);
  EXPECT_EQ(next.audio.source_id, initial.audio.source_id);
}

TEST(MonoRouter, FrameChangeAndPlanarMonoAreExplicit) {
  MonoRouter router;
  router.process(frame(0));
  auto second = frame(1, 10000000);
  second.layout = AudioFrame::PLANAR;
  EXPECT_EQ(router.process(second).sources.generation, 1U);
  auto new_device = frame(2, 20000000);
  new_device.header.frame_id = "other_mic";
  const auto changed = router.process(new_device);
  EXPECT_EQ(changed.sources.generation, 2U);
  EXPECT_EQ(changed.sources.header.frame_id, "other_mic");
  EXPECT_EQ(changed.sources.sources.front().header.frame_id, "other_mic");
}

TEST(MonoRouter, CaptureRestartWithContinuousClockAndSequenceStillFencesEpoch) {
  MonoRouter router;
  const auto original = router.process(frame(0));
  auto next_session = frame(1, 10000000);
  next_session.capture_session_id = std::string(32, 'b');
  const auto restarted = router.process(next_session);
  EXPECT_EQ(restarted.sources.generation, 2U);
  EXPECT_EQ(restarted.audio.source_generation, 2U);
  EXPECT_EQ(restarted.sources.sources.front().first_seen.nanosec, 10000000U);
  EXPECT_EQ(restarted.audio.source_id, original.audio.source_id);

  // A delayed packet from the previous process cannot steal the active epoch.
  EXPECT_THROW(router.process(frame(2, 20000000)), std::invalid_argument);
  EXPECT_EQ(router.generation(), 2U);
  auto malformed = frame(2, 20000000);
  malformed.capture_session_id = "not-a-session";
  EXPECT_THROW(router.process(malformed), std::invalid_argument);
  EXPECT_EQ(router.generation(), 2U);
  next_session.sequence_number = 2;
  next_session.header.stamp.nanosec = 20000000;
  EXPECT_EQ(router.process(next_session).sources.generation, 2U);
}

TEST(MonoRouter, CaptureSessionHistoryFailsClosedAfterBound) {
  MonoRouter router;
  auto input = frame(0);
  for (int i = 0; i < 65; ++i) {
    input.capture_session_id = std::string(30, '0') +
      "0123456789abcdef"[i / 16] + "0123456789abcdef"[i % 16];
    input.sequence_number = static_cast<std::uint64_t>(i);
    EXPECT_NO_THROW(router.process(input));
  }
  input.capture_session_id = std::string(30, '0') + "ff";
  EXPECT_THROW(router.process(input), std::invalid_argument);
  EXPECT_EQ(router.generation(), 65U);
}
