#pragma once
#include <cstdint>
#include <stdexcept>

namespace audition_perception {
// Startup-only observation policy, not a native inference-job timeout.
struct AsrObservationConfig {
  std::int64_t minimum_segments{4};
  std::int64_t minimum_age_ms{10000};
  std::int64_t max_sample_age_ms{3000};

  void validate(std::int64_t sample_period_ms) const {
    if (sample_period_ms < 200 || sample_period_ms > 10000 ||
        minimum_segments < 1 || minimum_segments > 1000000 ||
        minimum_age_ms < 1000 || minimum_age_ms > 600000 ||
        max_sample_age_ms < sample_period_ms || max_sample_age_ms > 30000 ||
        minimum_age_ms < 2 * sample_period_ms) {
      throw std::invalid_argument(
        "ASR observation: segments 1..1000000, pressure age 1000..600000 "
        "and >= 2*sample_period_ms; sample freshness >= sample_period_ms "
        "and <= 30000");
    }
  }
};
}  // namespace audition_perception
