#include "audition_capture/alsa_pcm_reader.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

TEST(AlsaPcmReader, RejectsBadOptionsBeforeOpeningDevice) {
  using audition_capture::AlsaPcmReader;
  EXPECT_THROW(AlsaPcmReader("", 16000, 6, 160), std::invalid_argument);
  EXPECT_THROW(AlsaPcmReader("null", 0, 6, 160), std::invalid_argument);
  EXPECT_THROW(AlsaPcmReader("null", 16000, 0, 160), std::invalid_argument);
  EXPECT_THROW(AlsaPcmReader("null", 16000, 6, 0), std::invalid_argument);
  EXPECT_THROW(AlsaPcmReader("null", 16000, 6, 8193), std::invalid_argument);
  EXPECT_THROW(AlsaPcmReader("null", 16000, 6, 160, 0), std::invalid_argument);
}

TEST(AlsaPcmReader, VirtualNullCaptureProducesBoundedNormalizedFrames) {
  audition_capture::AlsaPcmReader capture("null", 16000, 6, 160, 100);
  std::vector<float> samples;
  std::uint32_t frames = 0;
  bool discontinuity = false;
  bool received = false;
  for (int i = 0; i < 100 && !received; ++i) {
    received = capture.read(samples, frames, discontinuity);
  }
  ASSERT_TRUE(received);
  EXPECT_GT(frames, 0U);
  EXPECT_LE(frames, 160U);
  EXPECT_EQ(samples.size(), static_cast<std::size_t>(frames) * 6);
  EXPECT_FALSE(discontinuity);
  for (const float sample : samples) {
    EXPECT_TRUE(std::isfinite(sample));
    EXPECT_GE(sample, -1.0f);
    EXPECT_LE(sample, 1.0f);
  }
}
