#pragma once
#include <chrono>

namespace audition_behaviors {
// Tracks the last *successfully published* bearing, not merely incoming traffic.
// A monotonic deadline revokes it once if upstream goes silent.
class AttentionTfExpiry {
public:
  using Clock = std::chrono::steady_clock;
  explicit AttentionTfExpiry(std::chrono::milliseconds timeout) : timeout_(timeout) {}
  void observe(bool valid, Clock::time_point now) {
    active_ = valid;
    if (valid) last_valid_ = now;
  }
  bool expire(Clock::time_point now) {
    if (!active_ || now < last_valid_ || now - last_valid_ < timeout_) return false;
    active_ = false;
    return true;
  }
  bool active() const { return active_; }
private:
  std::chrono::milliseconds timeout_;
  Clock::time_point last_valid_{};
  bool active_ = false;
};
