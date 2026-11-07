#pragma once
// Sampled native workload, not per-job deadlines or a model-health watchdog.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
namespace audition_perception {
enum class ClassifierWorkState { Waiting, Collecting, Queued, Inferencing, Classified, Error };
struct ClassifierWorkloadSnapshot {
  std::uint32_t active_sources{0}, waiting_sources{0}, collecting_sources{0};
  std::uint32_t queued_sources{0}, inferencing_sources{0}, classified_sources{0}, error_sources{0};
  std::uint64_t dropped_windows_active{0};
  bool have_inference_timing{false}, have_queue_wait_timing{false};
  double max_last_inference_ms{0.0}, max_last_queue_wait_ms{0.0};
};
class ClassifierWorkloadAccumulator {
public:
  void observe(ClassifierWorkState state, std::uint64_t dropped,
               std::optional<double> inference_ms, std::optional<double> queue_wait_ms) {
    ++data_.active_sources;
    switch (state) {
      case ClassifierWorkState::Waiting: ++data_.waiting_sources; break;
      case ClassifierWorkState::Collecting: ++data_.collecting_sources; break;
      case ClassifierWorkState::Queued: ++data_.queued_sources; break;
      case ClassifierWorkState::Inferencing: ++data_.inferencing_sources; break;
      case ClassifierWorkState::Classified: ++data_.classified_sources; break;
      case ClassifierWorkState::Error: ++data_.error_sources; break;
    }
    data_.dropped_windows_active += dropped;
    if (inference_ms && std::isfinite(*inference_ms) && *inference_ms >= 0.0) {
      data_.have_inference_timing = true;
      data_.max_last_inference_ms = std::max(data_.max_last_inference_ms, *inference_ms);
    }
    if (queue_wait_ms && std::isfinite(*queue_wait_ms) && *queue_wait_ms >= 0.0) {
      data_.have_queue_wait_timing = true;
      data_.max_last_queue_wait_ms = std::max(data_.max_last_queue_wait_ms, *queue_wait_ms);
    }
  }
  ClassifierWorkloadSnapshot snapshot() const { return data_; }
private:
  ClassifierWorkloadSnapshot data_{};
};
struct SpeechWorkloadSnapshot {
  std::uint64_t queued_segments{0}, buffered_results{0}, retired_generations{0};
  std::uint64_t completed_segments_native{0}, dropped_segments_native{0}, evicted_results_native{0};
  bool worker_error{false};
};
}  // namespace audition_perception
