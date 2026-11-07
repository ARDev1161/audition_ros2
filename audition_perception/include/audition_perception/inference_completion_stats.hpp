#pragma once
// Process-local completed inference telemetry; no assumption about speech in silence.
#include <chrono>
#include <cstdint>
namespace audition_perception {
class InferenceCompletionStats {
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot {
    std::uint64_t classifier_completed_windows{0};
    std::uint64_t classifier_discarded_results{0};
    std::uint64_t classifier_last_completion_age_ms{0};
    std::uint64_t asr_completed_segments{0};
    std::uint64_t asr_empty_segments{0};
    std::uint64_t asr_discarded_results{0};
    std::uint64_t asr_last_completion_age_ms{0};
    std::uint64_t inference_poll_failures{0};
  };
  void classifierCompleted(Clock::time_point now = Clock::now()) {
    ++data_.classifier_completed_windows; classifier_last_=now; have_classifier_=true;
  }
  void classifierDiscarded() { ++data_.classifier_discarded_results; }
  void asrCompleted(std::uint64_t count, std::uint64_t empty,
                    std::uint64_t discarded, Clock::time_point now = Clock::now()) {
    data_.asr_completed_segments+=count;
    data_.asr_empty_segments+=empty;
    data_.asr_discarded_results+=discarded;
    if(count) { asr_last_=now; have_asr_=true; }
  }
  void pollFailed() { ++data_.inference_poll_failures; }
  Snapshot snapshot(Clock::time_point now = Clock::now()) const {
    auto out=data_;
    if(have_classifier_) out.classifier_last_completion_age_ms=age(classifier_last_,now);
    if(have_asr_) out.asr_last_completion_age_ms=age(asr_last_,now);
    return out;
  }
private:
  static std::uint64_t age(Clock::time_point last, Clock::time_point now) {
    if(now<last) return 0;
    return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now-last).count());
  }
  Snapshot data_;
  Clock::time_point classifier_last_{},asr_last_{};
  bool have_classifier_{false},have_asr_{false};
};
}  // namespace audition_perception
