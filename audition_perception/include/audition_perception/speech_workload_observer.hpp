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
    // An observation gap cannot establish continuous occupancy: reset both
    // residency clocks rather than extrapolating over a stalled timer.
    if (valid_ && (now < last_sample_ ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample_).count() >
          static_cast<std::int64_t>(max_observation_gap_ms_))) {
      valid_ = false;
    }
    if (!valid_ || cached_.queued_segments == 0 || value.queued_segments == 0) {
      queue_nonempty_since_ = value.queued_segments > 0 ? now : Clock::time_point{};
      queue_contiguous_samples_ = 0;
    }
    if (!valid_ || cached_.buffered_results == 0 || value.buffered_results == 0) {
      mailbox_nonempty_since_ = value.buffered_results > 0 ? now : Clock::time_point{};
      mailbox_contiguous_samples_ = 0;
    }
    if (value.queued_segments > 0) ++queue_contiguous_samples_;
    if (value.buffered_results > 0) ++mailbox_contiguous_samples_;
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
    queue_nonempty_since_ = {};
    mailbox_nonempty_since_ = {};
    queue_contiguous_samples_ = mailbox_contiguous_samples_ = 0;
  }
  void reset() {
    cached_ = {};
    peak_queued_ = 0;
    peak_buffered_ = 0;
    valid_ = false;
    samples_ = 0;
    failures_ = 0;
    last_sample_ = {};
    queue_nonempty_since_ = {};
    mailbox_nonempty_since_ = {};
    queue_contiguous_samples_ = mailbox_contiguous_samples_ = 0;
  }
  std::uint64_t queueNonemptyAgeMs(Clock::time_point now = Clock::now()) const {
    return nonemptyAgeMs(queue_nonempty_since_, cached_.queued_segments, now);
  }
  std::uint64_t mailboxNonemptyAgeMs(Clock::time_point now = Clock::now()) const {
    return nonemptyAgeMs(mailbox_nonempty_since_, cached_.buffered_results, now);
  }
  bool fresh(std::uint64_t max_sample_age_ms, Clock::time_point now = Clock::now()) const {
    return valid_ && max_sample_age_ms > 0 && now >= last_sample_ &&
      ageMs(now) <= max_sample_age_ms;
  }
  bool queuePressure(std::uint64_t minimum_segments, std::uint64_t minimum_age_ms,
                     Clock::time_point now = Clock::now(),
                     std::uint64_t max_sample_age_ms = 3000) const {
    return fresh(max_sample_age_ms, now) && queue_contiguous_samples_ >= 2 &&
      minimum_segments > 0 && minimum_age_ms > 0 &&
      cached_.queued_segments >= minimum_segments &&
      queueNonemptyAgeMs(now) >= minimum_age_ms;
  }
  SpeechWorkloadSnapshot current() const { return valid_ ? cached_ : SpeechWorkloadSnapshot{}; }
  bool valid() const { return valid_; }
  bool everSampled() const { return samples_ > 0; }
  std::uint64_t contiguousQueueSamples() const { return queue_contiguous_samples_; }
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
  std::uint64_t nonemptyAgeMs(Clock::time_point since, std::uint64_t count,
                              Clock::time_point now) const {
    if (!valid_ || count == 0 || now < since) return 0;
    return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - since).count());
  }
  static constexpr std::uint64_t max_observation_gap_ms_{3000};
  Clock::time_point queue_nonempty_since_{}, mailbox_nonempty_since_{};
  SpeechWorkloadSnapshot cached_{};
  Clock::time_point last_sample_{};
  std::uint64_t samples_{0}, failures_{0}, peak_queued_{0}, peak_buffered_{0};
  std::uint64_t queue_contiguous_samples_{0}, mailbox_contiguous_samples_{0};
  bool valid_{false};
};
}  // namespace audition_perception
