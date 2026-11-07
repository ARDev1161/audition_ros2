#pragma once
#include <cstdint>
#include <stdexcept>

namespace audition_perception {
// Startup-only diagnostics. Bounds avoid overflow, runaway polling and
// misleading sub-sample deadline precision.
struct ClassifierObservationConfig {
  std::int64_t sample_period_ms{1000};
  std::int64_t queued_threshold_ms{10000};
  std::int64_t inference_threshold_ms{30000};
  void validate() const {
    if (sample_period_ms < 200 || sample_period_ms > 10000 ||
        queued_threshold_ms < 1000 || queued_threshold_ms > 600000 ||
        inference_threshold_ms < 1000 || inference_threshold_ms > 600000 ||
        queued_threshold_ms < sample_period_ms ||
        inference_threshold_ms < sample_period_ms) {
      throw std::invalid_argument(
        "Classifier observation: sample_period_ms must be 200..10000; "
        "thresholds must be 1000..600000 and >= sample_period_ms");
    }
  }
};
} // namespace audition_perception
