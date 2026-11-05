#pragma once
#include <audition_msgs/msg/attention_target.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <cstdint>
#include <string>

namespace audition_behaviors {
inline bool valid_attention_observation(
  const audition_msgs::msg::AttentionTarget & input,
  int64_t now_ns, int maximum_age_ms, std::string & reason) {
  if (!input.valid) { reason = "upstream_invalid"; return false; }
  const int64_t stamp = static_cast<int64_t>(input.header.stamp.sec) * 1000000000LL +
    input.header.stamp.nanosec;
  if (stamp <= 0 || input.header.frame_id.empty() || maximum_age_ms <= 0 ||
      now_ns < stamp || now_ns - stamp > static_cast<int64_t>(maximum_age_ms) * 1000000LL) {
    reason = "stale_or_missing_stamp";
    return false;
  }
  const auto & v = input.direction_unit;
  const double norm = std::hypot(std::hypot(v.x, v.y), v.z);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) {
    reason = "invalid_bearing";
    return false;
  }
  return true;
}
inline bool rotate_attention_direction(
  const geometry_msgs::msg::Vector3 & v,
  const geometry_msgs::msg::Quaternion & q,
  geometry_msgs::msg::Vector3 & result) {
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) return false;
  const double tx = 2.0 * (q.y*v.z - q.z*v.y);
  const double ty = 2.0 * (q.z*v.x - q.x*v.z);
  const double tz = 2.0 * (q.x*v.y - q.y*v.x);
  result.x = v.x + q.w*tx + (q.y*tz - q.z*ty);
  result.y = v.y + q.w*ty + (q.z*tx - q.x*tz);
  result.z = v.z + q.w*tz + (q.x*ty - q.y*tx);
  const double result_norm = std::hypot(std::hypot(result.x, result.y), result.z);
  return std::isfinite(result_norm) && std::abs(result_norm - 1.0) <= 0.02;
}
}  // namespace audition_behaviors
