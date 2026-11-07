#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace audition_capture {

// Transport state only: never creates samples, never guesses capture timestamps.
// A discontinuity is encoded as a sequence gap on the next valid AudioFrame.
class CaptureContinuity final {
public:
  struct Snapshot {
    bool connected{true};
    bool terminal_failure{false};
    std::uint64_t published_frames{0};
    std::uint64_t last_sequence_number{0};
    std::uint64_t xrun_recoveries{0};
    std::uint64_t fatal_errors{0};
    std::uint64_t stall_reconnects{0};
    std::uint64_t reconnect_attempts{0};
    std::uint64_t reconnect_successes{0};
    std::uint64_t reconnect_failures{0};
    std::uint32_t consecutive_failures{0};
    std::uint32_t next_retry_delay_ms{0};
    std::string last_error;
  };

  explicit CaptureContinuity(std::uint32_t max_attempts = 0)
  : max_attempts_(max_attempts) {}

  void readerFailed(const std::string & reason, bool stalled = false) {
    state_.connected = false;
    state_.terminal_failure = false;
    ++state_.fatal_errors;
    if (stalled) ++state_.stall_reconnects;
    state_.last_error = reason;
    pending_gap_ = true;
  }

  bool canRetry() const noexcept {
    return max_attempts_ == 0 || state_.consecutive_failures < max_attempts_;
  }

  void reconnectAttempt() { ++state_.reconnect_attempts; }

  void reconnectFailed(const std::string & reason) {
    state_.connected = false;
    ++state_.reconnect_failures;
    ++state_.consecutive_failures;
    state_.last_error = reason;
    // 100, 200, 400, 800, 1600, 2000 ms, bounded forever.
    const auto shift = std::min(state_.consecutive_failures - 1U, 5U);
    state_.next_retry_delay_ms = std::min(2000U, 100U << shift);
    state_.terminal_failure = !canRetry();
  }

  void reconnected() {
    state_.connected = true;
    state_.terminal_failure = false;
    ++state_.reconnect_successes;
    state_.consecutive_failures = 0;
    state_.next_retry_delay_ms = 0;
    state_.last_error.clear();
    pending_gap_ = true;
  }

  std::uint64_t published(bool recovered_xrun) {
    if (recovered_xrun) {
      ++state_.xrun_recoveries;
      pending_gap_ = true;
    }
    if (pending_gap_ && state_.published_frames != 0) ++next_sequence_;
    pending_gap_ = false;
    state_.last_sequence_number = next_sequence_++;
    ++state_.published_frames;
    return state_.last_sequence_number;
  }

  Snapshot snapshot() const { return state_; }

private:
  std::uint32_t max_attempts_{0};  // zero means unlimited bounded-backoff retries
  std::uint64_t next_sequence_{0};
  bool pending_gap_{false};
  Snapshot state_;
};
}  // namespace audition_capture
