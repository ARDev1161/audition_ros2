#pragma once
// M5 admission boundary for future ROS2 evidence subscribers.
// Bearing, AV association and diagnostic intersections are NEVER metric proof.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_cell_buffer.hpp"

namespace audition_nav2 {
enum class MetricProvenance : std::uint8_t {
  BearingOnly, AvAssociation, DiagnosticIntersection2D, UnverifiedMetric,
  IndependentlyVerifiedMetric
};
struct StampedMetricEvidence {
  MetricCell cell;
  MetricProvenance provenance;
  std::int64_t transform_stamp_ns;
  std::int64_t transform_age_limit_ns;
  bool transform_lookup_succeeded;
  bool frame_matches_costmap;
  bool independent_metric_confirmation;
};
class AcousticMetricAdmission final {
 public:
  // Caller-supplied age limits cannot relax the hard TF freshness ceiling.
  static constexpr std::int64_t kMaxTransformAgeNs = 250000000LL;
  static constexpr std::int64_t kMaxStampSkewNs = 50000000LL;
  static bool admit(const StampedMetricEvidence& e, std::int64_t now_ns) {
    if (e.provenance != MetricProvenance::IndependentlyVerifiedMetric ||
        !e.independent_metric_confirmation || !e.cell.physically_validated ||
        !e.transform_lookup_succeeded || !e.frame_matches_costmap ||
        e.transform_age_limit_ns <= 0 || now_ns <= 0 ||
        e.transform_stamp_ns <= 0 || e.transform_stamp_ns > now_ns ||
        e.cell.stamp_ns <= 0 || e.cell.stamp_ns > now_ns ||
        !std::isfinite(e.cell.x_m) || !std::isfinite(e.cell.y_m))
      return false;
    // Use unsigned subtraction after ordering to avoid signed overflow.
    const auto tf_age = static_cast<std::uint64_t>(now_ns) -
                        static_cast<std::uint64_t>(e.transform_stamp_ns);
    const auto observation_tf_gap =
        e.transform_stamp_ns >= e.cell.stamp_ns
        ? static_cast<std::uint64_t>(e.transform_stamp_ns) -
          static_cast<std::uint64_t>(e.cell.stamp_ns)
        : static_cast<std::uint64_t>(e.cell.stamp_ns) -
          static_cast<std::uint64_t>(e.transform_stamp_ns);
    const auto bound = static_cast<std::uint64_t>(
        std::min(e.transform_age_limit_ns, kMaxTransformAgeNs));
    return tf_age <= bound &&
           observation_tf_gap <= std::min(bound, static_cast<std::uint64_t>(kMaxStampSkewNs));
  }
};
}  // namespace audition_nav2
