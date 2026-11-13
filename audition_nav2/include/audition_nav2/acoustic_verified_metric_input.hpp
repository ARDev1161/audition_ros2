#pragma once
// Opt-in input contract for a trusted upstream metric localization publisher.
// Ordinary M4 diagnostics, bearing/AV hypotheses and unlabeled coordinates
// are NOT accepted. The operator must bind the subscription only to a
// separately validated metric provider: a method string is not a physical
// calibration, DDS authentication or proof of sensor independence.
//
// This initial adapter only admits already-transformed positions in the
// costmap global frame. Other frames fail closed until stamped TF support is
// implemented. No sound-to-range inference is performed here.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include "audition_msgs/msg/localized_source_array.hpp"
#include "audition_nav2/acoustic_metric_admission.hpp"

namespace audition_nav2 {
class AcousticVerifiedMetricInput final {
 public:
  static constexpr std::size_t kMaxSources = 32;
  static constexpr std::int64_t kMaxAgeNs = 250000000LL;
  static constexpr std::int64_t kTtlNs = 500000000LL;
  static constexpr double kMaximumVarianceM2 = 0.25;
  static constexpr const char* kRequiredMethod =
      "independently_verified_metric_3d";

  static bool decode(
      const audition_msgs::msg::LocalizedSourceArray& msg,
      const std::string& global_frame, std::int64_t now_ns,
      std::vector<StampedMetricEvidence>& output) {
    if (global_frame.empty() || global_frame[0] == '/' ||
        msg.header.frame_id != global_frame || msg.generation == 0 ||
        msg.sources.size() > kMaxSources || now_ns <= 0)
      return false;

    std::int64_t array_stamp = 0;
    if (!stamp_ns(msg.header.stamp, array_stamp) ||
        array_stamp > now_ns ||
        static_cast<std::uint64_t>(now_ns) -
        static_cast<std::uint64_t>(array_stamp) >
        static_cast<std::uint64_t>(kMaxAgeNs))
      return false;

    std::vector<StampedMetricEvidence> pending;
    std::set<std::uint64_t> seen_ids;
    pending.reserve(msg.sources.size());
    for (const auto& source : msg.sources) {
      if (source.source_id.empty() || source.source_id.size() > 256 ||
          source.source_generation == 0 ||
          source.localization_method != kRequiredMethod ||
          source.header.frame_id != global_frame ||
          !std::isfinite(source.position.x) ||
          !std::isfinite(source.position.y) ||
          !std::isfinite(source.position.z) ||
          !valid_covariance(source.covariance))
        return false;

      std::int64_t observed = 0, header_stamp = 0;
      if (!stamp_ns(source.last_observed, observed) ||
          !stamp_ns(source.header.stamp, header_stamp) ||
          observed > array_stamp || header_stamp > array_stamp ||
          observed > now_ns || header_stamp > now_ns)
        return false;
      const auto age = static_cast<std::uint64_t>(now_ns) -
                       static_cast<std::uint64_t>(observed);
      const auto stamp_gap = observed >= header_stamp
          ? static_cast<std::uint64_t>(observed) -
            static_cast<std::uint64_t>(header_stamp)
          : static_cast<std::uint64_t>(header_stamp) -
            static_cast<std::uint64_t>(observed);
      if (age > static_cast<std::uint64_t>(kMaxAgeNs) ||
          stamp_gap > 50000000ULL ||
          static_cast<std::uint64_t>(array_stamp) -
          static_cast<std::uint64_t>(observed) >
          static_cast<std::uint64_t>(kMaxAgeNs) ||
          observed > std::numeric_limits<std::int64_t>::max() - kTtlNs)
        return false;

      const auto id = source_id(msg.generation, source.source_generation,
                                source.source_id);
      if (id == 0 || !seen_ids.insert(id).second) return false;
      StampedMetricEvidence e{};
      e.cell = {id, observed, observed + kTtlNs,
                source.position.x, source.position.y, true};
      e.provenance = MetricProvenance::IndependentlyVerifiedMetric;
      // Same-frame identity is the only transform supported by this stage.
      // Do not claim to have looked up TF for an arbitrary different frame.
      e.transform_stamp_ns = observed;
      e.transform_age_limit_ns = kMaxAgeNs;
      e.transform_lookup_succeeded = true;
      e.frame_matches_costmap = true;
      e.independent_metric_confirmation = true;
      if (!AcousticMetricAdmission::admit(e, now_ns)) return false;
      pending.push_back(e);
    }
    output.swap(pending);  // Failure never publishes a partial batch.
    return true;
  }

 private:
  template<typename Stamp>
  static bool stamp_ns(const Stamp& stamp, std::int64_t& result) {
    if (stamp.sec <= 0 || stamp.nanosec >= 1000000000u) return false;
    result = static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
             static_cast<std::int64_t>(stamp.nanosec);
    return result > 0;
  }

  template<typename Covariance>
  static bool valid_covariance(const Covariance& c) {
    for (const auto value : c)
      if (!std::isfinite(value)) return false;
    // Strong, conservative positive-definiteness check. A diagnostic ray
    // with unknown depth / degenerate covariance must not become a cell.
    if (!(c[0] > 0 && c[4] > 0 && c[8] > 0) ||
        c[0] > kMaximumVarianceM2 ||
        c[4] > kMaximumVarianceM2 ||
        c[8] > kMaximumVarianceM2 ||
        std::abs(c[1] - c[3]) > 1e-6 ||
        std::abs(c[2] - c[6]) > 1e-6 ||
        std::abs(c[5] - c[7]) > 1e-6 ||
        c[0] <= std::abs(c[1]) + std::abs(c[2]) ||
        c[4] <= std::abs(c[3]) + std::abs(c[5]) ||
        c[8] <= std::abs(c[6]) + std::abs(c[7]))
      return false;
    return true;
  }

  static std::uint64_t source_id(std::uint64_t scene_generation,
                                  std::uint64_t source_generation,
                                  const std::string& name) {
    constexpr std::uint64_t kOffset = 14695981039346656037ULL;
    constexpr std::uint64_t kPrime = 1099511628211ULL;
    std::uint64_t value = kOffset;
    const auto append_u64 = [&](std::uint64_t n, std::uint64_t& h) {
      for (unsigned shift = 0; shift < 64; shift += 8) {
        h ^= (n >> shift) & 0xffULL;
        h *= kPrime;
      }
    };
    append_u64(scene_generation, value);
    append_u64(source_generation, value);
    for (unsigned char ch : name) {
      value ^= ch;
      value *= kPrime;
    }
    return value;
  }
};
}  // namespace audition_nav2
