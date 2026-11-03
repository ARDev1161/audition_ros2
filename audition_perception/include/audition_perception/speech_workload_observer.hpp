#pragma once
// ASR queue statistics sampled independently of status-service traffic.
// A pending segment is not an active inference job; no deadline is inferred.
#include "audition_perception/inference_workload_stats.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>

namespace audition_perception {
class SpeechWorkloadObserver {
public:
  using Clock = std::chrono::steady_clock;
  void sample(const SpeechWorkloadSnapshot & value, Clock::time_point now = Clock::now()) {
    cached_ = value;
    peak_queued_ = std::max(peak_queued_, value.queued_segments);
    peak_buffered_ = std::max(peak_buffered_, value.buffered_results);
    last_sample_ = now;
    ++samples_;
    valid_ = true;
  }
  void failed() {
    ++failures_;
    valid_ = false;
    cached_ = {};
  }
  void reset() {
    cached_ = {};
    peak_queued_ = 0;
    peak_buffered_ = 0;
    valid_ = false;
    samples_ = 0;
    failures_ = 0;
    last_sample_ = {};
  }
  SpeechWorkloadSnapshot current() const { return valid_ ? cached_ : SpeechWorkloadSnapshot{}; }
  bool valid() const { return valid_; }
  bool everSampled() const { return samples_ > 0; }
  std::uint64_t samples() const { return samples_; }
  std::uint64_t failures() const { return failures_; }
  std::uint64_t peakQueued() const { return peak_queued_; }
  std::uint64_t peakBuffered() const { return peak_buffered_; }
  std::uint64_t ageMs(Clock::time_point now = Clock::now()) const {
    if (!everSampled() || now < last_sample_) return 0;
    return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample_).count());
  }
private:
  SpeechWorkloadSnapshot cached_{};
  Clock::time_point last_sample_{};
  std::uint64_t samples_{0}, failures_{0}, peak_queued_{0}, peak_buffered_{0};
  bool valid_{false};
};
}  // namespace audition_perception
