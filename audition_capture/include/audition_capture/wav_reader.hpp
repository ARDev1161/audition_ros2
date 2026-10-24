#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace audition_capture {

// Intentionally narrow reader: little-endian RIFF/WAVE PCM16 or IEEE float32.
// No compressed audio, extensible WAV, network transport or implicit resampling.
class WavReader {
public:
  explicit WavReader(const std::string & path);
  WavReader(const WavReader &) = delete;
  WavReader & operator=(const WavReader &) = delete;

  [[nodiscard]] std::uint32_t sampleRate() const noexcept { return sample_rate_; }
  [[nodiscard]] std::uint16_t channels() const noexcept { return channels_; }
  [[nodiscard]] std::uint64_t totalFrames() const noexcept { return total_frames_; }

  // Returns interleaved float32 PCM. Empty only at EOF.
  [[nodiscard]] std::vector<float> readFrames(std::uint32_t requested_frames);
  void rewind();

private:
  std::ifstream stream_;
  std::uint32_t sample_rate_{0};
  std::uint16_t channels_{0};
  std::uint16_t format_{0};
  std::uint16_t bits_per_sample_{0};
  std::uint16_t block_align_{0};
  std::uint64_t data_offset_{0};
  std::uint64_t total_frames_{0};
  std::uint64_t frames_remaining_{0};
};

}  // namespace audition_capture
