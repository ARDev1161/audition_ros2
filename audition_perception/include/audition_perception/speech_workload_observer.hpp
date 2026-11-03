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
    // Native counters are cumulative. Record only positive deltas; a native
    // counter reset starts a new baseline instead of wrapping uint64.
    if (have_counter_baseline_) {
      dropped_delta_ = value.dropped_segments_native >= last_dropped_
        ? value.dropped_segments_native - last_dropped_ : 0;
      evicted_delta_ = value.evicted_results_native >= last_evicted_
        ? value.evicted_results_native - last_evicted_ : 0;
    } else {
      dropped_delta_ = evicted_delta_ = 0;
    }
    last_dropped_ = value.dropped_segments_native;
    last_evicted_ = value.evicted_results_native;
    have_counter_baseline_ = true;
    // Saturate rather than wrap long-lived diagnostics.
    total_dropped_delta_ = saturatingAdd(total_dropped_delta_, dropped_delta_);
    total_evicted_delta_ = saturatingAdd(total_evicted_delta_, evicted_delta_);
    cached_ = value;
    peak_queued_ = std::max(peak_queued_, value.queued_segments);
    peak_buffered_ = std::max(peak_buffered_, value.buffered_results);
    last_sample_ = now;
    ++samples_;
    valid_ = true;
  }
  void failed() {
    ++failures_;
    have_counter_baseline_ = false;
    dropped_delta_ = evicted_delta_ = 0;
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
    last_dropped_ = last_evicted_ = 0;
    dropped_delta_ = evicted_delta_ = 0;
    total_dropped_delta_ = total_evicted_delta_ = 0;
    have_counter_baseline_ = false;
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
  std::uint64_t lastDroppedDelta() const { return valid_ ? dropped_delta_ : 0; }
  std::uint64_t lastEvictedDelta() const { return valid_ ? evicted_delta_ : 0; }
  std::uint64_t observedDroppedDeltaTotal() const { return total_dropped_delta_; }
  std::uint64_t observedEvictedDeltaTotal() const { return total_evicted_delta_; }
  bool lossOnLastSample() const { return valid_ && (dropped_delta_ > 0 || evicted_delta_ > 0); }
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
  static std::uint64_t saturatingAdd(std::uint64_t a, std::uint64_t b) {
    return b > UINT64_MAX - a ? UINT64_MAX : a + b;
  }
  std::uint64_t last_dropped_{0}, last_evicted_{0};
  std::uint64_t dropped_delta_{0}, evicted_delta_{0};
  std::uint64_t total_dropped_delta_{0}, total_evicted_delta_{0};
  bool have_counter_baseline_{false};
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
