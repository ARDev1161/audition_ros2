#include "audition_perception/odas_ingress.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <vector>

using audition_msgs::msg::AudioFrame;
using audition_perception::OdasIngress;
namespace {
AudioFrame frame(std::uint64_t sequence, std::int64_t time_ns,
                 std::uint32_t frames = 160, std::uint32_t channels = 6) {
  AudioFrame f;
  f.header.frame_id = "microphone_array";
  f.header.stamp.sec = static_cast<std::int32_t>(time_ns / 1000000000LL);
  f.header.stamp.nanosec = static_cast<std::uint32_t>(time_ns % 1000000000LL);
  f.sequence_number = sequence;
  f.sample_rate_hz = 16000;
  f.frame_count = frames;
  f.channel_count = channels;
  f.layout = AudioFrame::INTERLEAVED;
  for (std::uint32_t ch = 0; ch < channels; ++ch) f.channel_indices.push_back(ch);
  f.pcm.assign(static_cast<std::size_t>(frames) * channels, 0.25f);
  return f;
}
}  // namespace

TEST(OdasIngress, BoundedHopsPreserveCaptureClockAndPCM) {
  OdasIngress ingress(16000, 128, "microphone_array", 5);
  std::vector<std::int64_t> stamps;
  std::vector<std::uint64_t> seqs;
  for (int i = 0; i < 4; ++i) {
    auto result = ingress.push(frame(i, static_cast<std::int64_t>(i) * 10000000));
    EXPECT_FALSE(result.reset);
    EXPECT_EQ(result.generation, 1U);
    for (const auto & hop : result.hops) {
      EXPECT_EQ(hop.pcm.size(), 128U * 6U);
      EXPECT_EQ(hop.channels, 6U);
      EXPECT_EQ(hop.pcm.front(), 0.25f);
      stamps.push_back(hop.capture_ns);
      seqs.push_back(hop.sequence);
    }
  }
  EXPECT_EQ(stamps, (std::vector<std::int64_t>{0, 8000000, 16000000, 24000000, 32000000}));
  EXPECT_EQ(seqs, (std::vector<std::uint64_t>{1, 2, 3, 4, 5}));
  EXPECT_EQ(ingress.bufferedFrames(), 0U);
  EXPECT_EQ(ingress.stats().accepted_frames, 4U);
  EXPECT_EQ(ingress.stats().processed_hops, 5U);
}

TEST(OdasIngress, PlanarFramesBecomeInterleavedWithoutMixingChannels) {
  OdasIngress ingress(16000, 128, "microphone_array", 2);
  auto input = frame(0, 0, 128, 2);
  input.layout = AudioFrame::PLANAR;
  input.pcm.assign(128, 0.1f);
  input.pcm.insert(input.pcm.end(), 128, 0.2f);
  const auto batch = ingress.push(input);
  ASSERT_EQ(batch.hops.size(), 1U);
  EXPECT_FLOAT_EQ(batch.hops[0].pcm[0], 0.1f);
  EXPECT_FLOAT_EQ(batch.hops[0].pcm[1], 0.2f);
  EXPECT_FLOAT_EQ(batch.hops[0].pcm[254], 0.1f);
  EXPECT_FLOAT_EQ(batch.hops[0].pcm[255], 0.2f);
}

TEST(OdasIngress, RejectsMalformedFramesWithoutTouchingEpochOrPendingPCM) {
  OdasIngress ingress(16000, 128, "microphone_array", 5);
  auto initial = ingress.push(frame(0, 0));
  ASSERT_EQ(initial.hops.size(), 1U);
  EXPECT_EQ(ingress.bufferedFrames(), 32U);
  auto bad = frame(1, 10000000);
  bad.header.stamp.nanosec = 1000000000U;
  EXPECT_THROW(ingress.push(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.pcm[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(ingress.push(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.channel_indices[2] = 1;
  EXPECT_THROW(ingress.push(bad), std::invalid_argument);
  bad = frame(1, 10000000);
  bad.header.frame_id = "wrong";
  EXPECT_THROW(ingress.push(bad), std::invalid_argument);
  bad = frame(1, 10000000, 513);
  EXPECT_THROW(ingress.push(bad), std::invalid_argument);
  EXPECT_EQ(ingress.bufferedFrames(), 32U);
  EXPECT_EQ(ingress.generation(), 1U);
  EXPECT_EQ(ingress.stats().rejected_frames, 5U);
  EXPECT_FALSE(ingress.push(frame(1, 10000000)).reset);
  EXPECT_EQ(ingress.stats().accepted_frames, 2U);
}

TEST(OdasIngress, DiscontinuitiesResetPendingPCMAndSourceEpoch) {
  OdasIngress ingress(16000, 128, "microphone_array", 5);
  ingress.push(frame(0, 0));
  auto skipped = ingress.push(frame(3, 30000000));
  EXPECT_TRUE(skipped.reset);
  EXPECT_EQ(skipped.generation, 2U);
  ASSERT_EQ(skipped.hops.size(), 1U);
  EXPECT_EQ(skipped.hops[0].capture_ns, 30000000);
  EXPECT_EQ(skipped.hops[0].sequence, 1U);
  auto backwards = ingress.push(frame(4, 10000000));
  EXPECT_TRUE(backwards.reset);
  EXPECT_EQ(backwards.generation, 3U);
  auto stalled = ingress.push(frame(5, 600000000));
  EXPECT_TRUE(stalled.reset);
  EXPECT_EQ(stalled.generation, 4U);
  auto changed = ingress.push(frame(6, 610000000, 160, 7));
  EXPECT_TRUE(changed.reset);
  EXPECT_EQ(changed.generation, 5U);
  EXPECT_EQ(ingress.stats().discontinuity_resets, 4U);
}

TEST(OdasIngress, OverflowDropsWholeChunkAndFencesOldGeneration) {
  OdasIngress ingress(16000, 128, "microphone_array", 5, 2);
  ingress.push(frame(0, 0, 64));
  auto dropped = ingress.push(frame(1, 4000000, 512));
  EXPECT_TRUE(dropped.reset);
  EXPECT_TRUE(dropped.hops.empty());
  EXPECT_EQ(dropped.generation, 2U);
  EXPECT_EQ(ingress.bufferedFrames(), 0U);
  EXPECT_EQ(ingress.stats().overflow_drops, 1U);
  EXPECT_EQ(ingress.stats().accepted_frames, 1U);
  const auto recovered = ingress.push(frame(2, 36000000, 128));
  EXPECT_FALSE(recovered.reset);
  ASSERT_EQ(recovered.hops.size(), 1U);
  EXPECT_EQ(recovered.hops[0].capture_ns, 36000000);
  ingress.backendFailure();
  EXPECT_EQ(ingress.generation(), 3U);
  EXPECT_EQ(ingress.stats().backend_resets, 1U);
  EXPECT_EQ(ingress.bufferedFrames(), 0U);
}
