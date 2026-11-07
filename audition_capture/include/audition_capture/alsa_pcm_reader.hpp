#pragma once

#include <alsa/asoundlib.h>

#include <cstdint>
#include <string>
#include <vector>

namespace audition_capture {

// A narrowly scoped ALSA capture source. All ALSA I/O must occur on one
// worker thread; no ROS objects are required by this reader.
class AlsaPcmReader final {
public:
  AlsaPcmReader(const std::string & device, std::uint32_t sample_rate_hz,
                std::uint32_t channels, std::uint32_t chunk_frames,
                int wait_timeout_ms = 100);
  ~AlsaPcmReader();

  AlsaPcmReader(const AlsaPcmReader &) = delete;
  AlsaPcmReader & operator=(const AlsaPcmReader &) = delete;

  // true: a nonempty frame; false: timeout, EAGAIN or recovered XRUN/suspend.
  // On recovery the NEXT successful frame has discontinuity=true.
  bool read(std::vector<float> & samples, std::uint32_t & frames,
            bool & discontinuity);

  std::uint32_t sampleRate() const noexcept { return sample_rate_; }
  std::uint32_t channels() const noexcept { return channels_; }
  std::uint32_t chunkFrames() const noexcept { return chunk_frames_; }
  std::uint64_t recoveries() const noexcept { return recoveries_; }

private:
  void recoverOrThrow(int error);
  snd_pcm_t * pcm_{nullptr};
  std::vector<std::int16_t> scratch_{};
  std::uint32_t sample_rate_{0}, channels_{0}, chunk_frames_{0};
  int wait_timeout_ms_{100};
  std::uint64_t recoveries_{0};
  bool discontinuity_pending_{false};
};

}  // namespace audition_capture
