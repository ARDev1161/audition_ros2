#pragma once

#include "audition_perception/capture_session_gate.hpp"

#include <cstdint>

namespace audition_perception {

// Per-incarnation capture high-water mark. Reject delayed/duplicate packets
// before they can trigger a false discontinuity or enter a PCM buffer.
// A genuine new capture session may restart its sequence at any value.
// This is transport ordering, not publisher authentication.
class CaptureSequenceGate final {
public:
  bool isReplay(std::uint64_t sequence, CaptureSessionGate::Decision session) const noexcept {
    return session == CaptureSessionGate::Decision::Same &&
      have_cursor_ && sequence <= last_sequence_;
  }

  // Commit only after complete payload and capture-session validation.
  // An intentionally dropped overflow chunk still consumes its sequence.
  void commit(std::uint64_t sequence) noexcept {
    last_sequence_ = sequence;
    have_cursor_ = true;
  }

private:
  std::uint64_t last_sequence_{0};
  bool have_cursor_{false};
};

}  // namespace audition_perception
