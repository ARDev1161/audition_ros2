#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace audition_perception {

// Fail-closed per-process capture-incarnation gate. It is an ordering fence,
// not DDS authentication. Old sessions cannot reclaim a stream after restart.
class CaptureSessionGate final {
public:
  enum class Decision { First, Same, Changed };

  Decision inspect(const std::string & id) const {
    if (id.size() != 32 || !std::all_of(id.begin(), id.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    })) {
      throw std::invalid_argument("Invalid or missing capture_session_id");
    }
    if (id == current_) return Decision::Same;
    if (std::find(retired_.begin(), retired_.end(), id) != retired_.end()) {
      throw std::invalid_argument("Stale capture session replay");
    }
    if (current_.empty()) return Decision::First;
    if (retired_.size() >= 64) {
      throw std::invalid_argument("Capture session history exhausted; restart consumer");
    }
    return Decision::Changed;
  }

  void commit(const std::string & id) {
    if (id == current_) return;
    if (!current_.empty()) retired_.push_back(current_);
    current_ = id;
  }

  const std::string & current() const noexcept { return current_; }

private:
  std::string current_;
  std::vector<std::string> retired_;
};

}  // namespace audition_perception
