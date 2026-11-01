#include "audition_capture/alsa_pcm_reader.hpp"

#include <cerrno>
#include <limits>
#include <stdexcept>
#include <string>

namespace audition_capture {
namespace {
void check(int result, const char * action) {
  if (result < 0) {
    throw std::runtime_error(std::string(action) + ": " + snd_strerror(result));
  }
}
}

AlsaPcmReader::AlsaPcmReader(
  const std::string & device, std::uint32_t sample_rate_hz,
  std::uint32_t channels, std::uint32_t chunk_frames, int wait_timeout_ms)
: sample_rate_(sample_rate_hz), channels_(channels), chunk_frames_(chunk_frames),
  wait_timeout_ms_(wait_timeout_ms)
{
  if (device.empty() || sample_rate_hz < 8000 || sample_rate_hz > 192000 ||
      channels == 0 || channels > 32 || chunk_frames == 0 ||
      chunk_frames > 8192 || wait_timeout_ms < 1 || wait_timeout_ms > 1000) {
    throw std::invalid_argument("Invalid ALSA capture parameters");
  }

  // snd_pcm_open allocates resources before any later configuration step. Use
  // a guarded construction scope so exceptions cannot leak the PCM handle.
  check(snd_pcm_open(&pcm_, device.c_str(), SND_PCM_STREAM_CAPTURE,
                     SND_PCM_NONBLOCK), "snd_pcm_open");
  try {
    snd_pcm_hw_params_t * hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);
    check(snd_pcm_hw_params_any(pcm_, hw), "snd_pcm_hw_params_any");
    check(snd_pcm_hw_params_set_access(pcm_, hw, SND_PCM_ACCESS_RW_INTERLEAVED),
          "ALSA interleaved capture");
    check(snd_pcm_hw_params_set_format(pcm_, hw, SND_PCM_FORMAT_S16_LE),
          "ALSA S16_LE format");
    check(snd_pcm_hw_params_set_channels(pcm_, hw, channels_), "ALSA channels");
    check(snd_pcm_hw_params_set_rate(pcm_, hw, sample_rate_hz, 0),
          "ALSA sample rate");
    snd_pcm_uframes_t period = chunk_frames_;
    check(snd_pcm_hw_params_set_period_size_near(pcm_, hw, &period, nullptr),
          "ALSA period size");
    check(snd_pcm_hw_params(pcm_, hw), "snd_pcm_hw_params");

    unsigned int actual_rate = 0;
    unsigned int actual_channels = 0;
    int direction = 0;
    check(snd_pcm_hw_params_get_rate(hw, &actual_rate, &direction),
          "ALSA negotiated sample rate");
    check(snd_pcm_hw_params_get_channels(hw, &actual_channels),
          "ALSA negotiated channels");
    if (actual_rate != sample_rate_hz || actual_channels != channels_) {
      throw std::runtime_error("ALSA negotiated an unexpected rate or channel count");
    }
    scratch_.resize(static_cast<std::size_t>(chunk_frames_) * channels_);
    check(snd_pcm_prepare(pcm_), "snd_pcm_prepare");
    // Capture must be running before the first wait on physical ALSA devices.
    check(snd_pcm_start(pcm_), "snd_pcm_start");
  } catch (...) {
    snd_pcm_close(pcm_);
    pcm_ = nullptr;
    throw;
  }
}

AlsaPcmReader::~AlsaPcmReader() {
  if (pcm_ != nullptr) snd_pcm_close(pcm_);
}

void AlsaPcmReader::recoverOrThrow(int error) {
  if (error != -EPIPE && error != -ESTRPIPE) {
    check(error, "ALSA capture I/O");
    return;
  }
  const int result = snd_pcm_recover(pcm_, error, 1);
  check(result, "ALSA XRUN/suspend recovery");
  check(snd_pcm_start(pcm_), "ALSA capture restart");
  ++recoveries_;
  discontinuity_pending_ = true;
}

bool AlsaPcmReader::read(
  std::vector<float> & samples, std::uint32_t & frames, bool & discontinuity)
{
  samples.clear();
  frames = 0;
  discontinuity = false;

  const int ready = snd_pcm_wait(pcm_, wait_timeout_ms_);
  if (ready == 0) return false;
  if (ready < 0) {
    if (ready == -EAGAIN || ready == -EINTR) return false;
    recoverOrThrow(ready);
    return false;
  }

  const snd_pcm_sframes_t got = snd_pcm_readi(pcm_, scratch_.data(), chunk_frames_);
  if (got == -EAGAIN || got == -EINTR || got == 0) return false;
  if (got < 0) {
    recoverOrThrow(static_cast<int>(got));
    return false;
  }
  if (static_cast<std::uint64_t>(got) > chunk_frames_) {
    throw std::runtime_error("ALSA returned more frames than requested");
  }
  frames = static_cast<std::uint32_t>(got);
  const auto n = static_cast<std::size_t>(frames) * channels_;
  samples.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    samples[i] = static_cast<float>(scratch_[i]) / 32768.0f;
  }
  discontinuity = discontinuity_pending_;
  discontinuity_pending_ = false;
  return true;
}
}  // namespace audition_capture
