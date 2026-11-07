#include "audition_capture/wav_reader.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {
void put16(std::ostream & s, std::uint16_t v) {
  const char b[2] = {static_cast<char>(v), static_cast<char>(v >> 8)};
  s.write(b, 2);
}
void put32(std::ostream & s, std::uint32_t v) {
  const char b[4] = {static_cast<char>(v), static_cast<char>(v >> 8),
    static_cast<char>(v >> 16), static_cast<char>(v >> 24)};
  s.write(b, 4);
}
std::string fixture(bool invalid = false) {
  const auto path = std::filesystem::temp_directory_path() /
    (invalid ? "audition_bad_pcm16.wav" : "audition_pcm16_stereo.wav");
  std::ofstream out(path, std::ios::binary);
  out.write("RIFF", 4); put32(out, 36 + 8);
  out.write("WAVE", 4);
  out.write("fmt ", 4); put32(out, 16);
  put16(out, 1); put16(out, 2); put32(out, 16000);
  put32(out, 64000); put16(out, 4); put16(out, invalid ? 8 : 16);
  out.write("data", 4); put32(out, 8);
  put16(out, 0); put16(out, 16384);
  put16(out, 32768); put16(out, 32767);
  out.close();
  return path.string();
}
}  // namespace

TEST(WavReader, StereoPcm16ConvertsAndRewinds) {
  const auto path = fixture();
  audition_capture::WavReader reader(path);
  EXPECT_EQ(reader.channels(), 2);
  EXPECT_EQ(reader.sampleRate(), 16000U);
  EXPECT_EQ(reader.totalFrames(), 2U);
  auto first = reader.readFrames(1);
  ASSERT_EQ(first.size(), 2U);
  EXPECT_FLOAT_EQ(first[0], 0.0f);
  EXPECT_FLOAT_EQ(first[1], 0.5f);
  auto second = reader.readFrames(2);
  ASSERT_EQ(second.size(), 2U);
  EXPECT_FLOAT_EQ(second[0], -1.0f);
  EXPECT_NEAR(second[1], 1.0f, 0.00004f);
  EXPECT_TRUE(reader.readFrames(1).empty());
  reader.rewind();
  EXPECT_EQ(reader.readFrames(2).size(), 4U);
  std::filesystem::remove(path);
}

TEST(WavReader, RejectsInconsistentFormat) {
  const auto path = fixture(true);
  EXPECT_THROW(audition_capture::WavReader reader(path), std::runtime_error);
  std::filesystem::remove(path);
}
