#include "audition_capture/wav_reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace audition_capture {
namespace {
std::uint16_t u16(const unsigned char * p) {
  return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8);
}
std::uint32_t u32(const unsigned char * p) {
  return static_cast<std::uint32_t>(p[0]) |
    (static_cast<std::uint32_t>(p[1]) << 8) |
    (static_cast<std::uint32_t>(p[2]) << 16) |
    (static_cast<std::uint32_t>(p[3]) << 24);
}
void readExactly(std::istream & stream, unsigned char * p, std::size_t n) {
  if (!stream.read(reinterpret_cast<char *>(p), static_cast<std::streamsize>(n))) {
    throw std::runtime_error("Unexpected end of WAV file");
  }
}
}

WavReader::WavReader(const std::string & path) : stream_(path, std::ios::binary) {
  if (!stream_) throw std::runtime_error("Cannot open WAV file: " + path);
  unsigned char riff[12]{};
  readExactly(stream_, riff, sizeof(riff));
  if (std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(riff + 8, "WAVE", 4) != 0) {
    throw std::runtime_error("Expected little-endian RIFF/WAVE");
  }
  bool have_fmt = false;
  bool have_data = false;
  while (stream_) {
    unsigned char chunk[8]{};
    if (!stream_.read(reinterpret_cast<char *>(chunk), sizeof(chunk))) break;
    const std::uint32_t size = u32(chunk + 4);
    if (std::memcmp(chunk, "fmt ", 4) == 0) {
      if (size < 16) throw std::runtime_error("Truncated WAV fmt chunk");
      unsigned char fmt[16]{};
      readExactly(stream_, fmt, sizeof(fmt));
      format_ = u16(fmt);
      channels_ = u16(fmt + 2);
      sample_rate_ = u32(fmt + 4);
      const auto byte_rate = u32(fmt + 8);
      block_align_ = u16(fmt + 12);
      bits_per_sample_ = u16(fmt + 14);
      if (channels_ == 0 || channels_ > 32 || sample_rate_ < 8000 ||
          sample_rate_ > 192000 || (format_ != 1 && format_ != 3) ||
          ((format_ == 1 && bits_per_sample_ != 16) ||
           (format_ == 3 && bits_per_sample_ != 32)) ||
          block_align_ != channels_ * (bits_per_sample_ / 8) ||
          byte_rate != sample_rate_ * block_align_) {
        throw std::runtime_error("Unsupported or inconsistent WAV format");
      }
      have_fmt = true;
      stream_.seekg(static_cast<std::streamoff>(size - 16), std::ios::cur);
    } else if (std::memcmp(chunk, "data", 4) == 0) {
      if (!have_fmt) throw std::runtime_error("WAV fmt chunk must precede data");
      data_offset_ = static_cast<std::uint64_t>(stream_.tellg());
      if (size % block_align_ != 0) throw std::runtime_error("Unaligned WAV data chunk");
      total_frames_ = size / block_align_;
      frames_remaining_ = total_frames_;
      have_data = true;
      break;
    } else {
      stream_.seekg(static_cast<std::streamoff>(size), std::ios::cur);
    }
    if (size % 2 != 0) stream_.seekg(1, std::ios::cur);
  }
  if (!have_fmt || !have_data) throw std::runtime_error("Missing WAV fmt or data chunk");
}

std::vector<float> WavReader::readFrames(std::uint32_t requested_frames) {
  if (requested_frames == 0 || requested_frames > 192000) {
    throw std::invalid_argument("Invalid requested WAV frame count");
  }
  const auto count = std::min<std::uint64_t>(requested_frames, frames_remaining_);
  if (count == 0) return {};
  const auto samples = static_cast<std::size_t>(count * channels_);
  std::vector<unsigned char> bytes(samples * (bits_per_sample_ / 8));
  readExactly(stream_, bytes.data(), bytes.size());
  std::vector<float> result(samples);
  for (std::size_t i = 0; i < samples; ++i) {
    const unsigned char * p = bytes.data() + i * (bits_per_sample_ / 8);
    if (format_ == 1) {
      result[i] = static_cast<float>(static_cast<std::int16_t>(u16(p))) / 32768.0f;
    } else {
      const std::uint32_t raw = u32(p);
      std::memcpy(&result[i], &raw, sizeof(raw));
      if (!std::isfinite(result[i]) || std::fabs(result[i]) > 1.0f) {
        throw std::runtime_error("Invalid normalized IEEE float32 WAV sample");
      }
    }
  }
  frames_remaining_ -= count;
  return result;
}

void WavReader::rewind() {
  stream_.clear();
  stream_.seekg(static_cast<std::streamoff>(data_offset_), std::ios::beg);
  if (!stream_) throw std::runtime_error("WAV seek failed");
  frames_remaining_ = total_frames_;
}
}  // namespace audition_capture
