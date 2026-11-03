#pragma once
// Observed native YAMNet state residency, not a per-job runtime measurement.
#include "audition_perception/inference_workload_stats.hpp"
#include <chrono>
#include <cstdint>
#include <map>
#include <set>
#include <string>

namespace audition_perception {
struct ClassifierDeadlineSnapshot {
  std::uint32_t overdue_queued_sources{0};
  std::uint32_t overdue_inferencing_sources{0};
  std::uint64_t max_queued_age_ms{0};
  std::uint64_t max_inferencing_age_ms{0};
};
class ClassifierStateDeadlineTracker {
public:
  using Clock = std::chrono::steady_clock;
  void observe(const std::string & source, ClassifierWorkState state,
               Clock::time_point now = Clock::now()) {
    auto it = states_.find(source);
    if (it == states_.end() || it->second.state != state) {
      states_[source] = {state, now};
    }
    seen_.insert(source);
  }
  void finishSample() {
    for (auto it = states_.begin(); it != states_.end();) {
      if (!seen_.count(it->first)) it = states_.erase(it);
      else ++it;
    }
    seen_.clear();
  }
  void reset() { states_.clear(); seen_.clear(); }
  ClassifierDeadlineSnapshot snapshot(std::uint64_t queue_limit_ms,
      std::uint64_t inference_limit_ms, Clock::time_point now = Clock::now()) const {
    ClassifierDeadlineSnapshot result;
    for (const auto & entry : states_) {
      const auto & value = entry.second;
      const auto age = now >= value.since
        ? static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            now - value.since).count()) : 0U;
      if (value.state == ClassifierWorkState::Queued) {
        result.max_queued_age_ms = std::max(result.max_queued_age_ms, age);
        if (queue_limit_ms > 0 && age > queue_limit_ms) ++result.overdue_queued_sources;
      } else if (value.state == ClassifierWorkState::Inferencing) {
        result.max_inferencing_age_ms = std::max(result.max_inferencing_age_ms, age);
        if (inference_limit_ms > 0 && age > inference_limit_ms)
          ++result.overdue_inferencing_sources;
      }
    }
    return result;
  }
private:
  struct Entry { ClassifierWorkState state; Clock::time_point since; };
  std::map<std::string, Entry> states_;
  std::set<std::string> seen_;
};
}  // namespace audition_perception
