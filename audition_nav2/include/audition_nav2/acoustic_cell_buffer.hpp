#pragma once
// M5: bounded, ROS-independent core for an optional acoustic Nav2 layer.
// Only independently validated metric observations may enter this API.
// No lethal cost, cmd_vel, or writes to the master costmap.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace audition_nav2 {
struct MetricCell {
  std::uint64_t source_id;
  std::int64_t stamp_ns;
  std::int64_t expires_ns;
  double x_m, y_m;
  bool physically_validated;
};
struct GridGeometry {
  unsigned width, height;
  double resolution, origin_x, origin_y;
};
struct LayerUpdate {
  std::map<std::pair<unsigned, unsigned>, std::uint8_t> own_cells;
  std::set<std::pair<unsigned, unsigned>> retired_cells;
  bool geometry_changed = false;
};
class AcousticCellBuffer {
 public:
  explicit AcousticCellBuffer(std::size_t max_sources = 32, std::uint8_t cost = 100,
                              std::int64_t max_ttl_ns = 5000000000LL,
                              std::int64_t max_age_ns = 1000000000LL)
      : max_sources_(max_sources), cost_(cost), max_ttl_ns_(max_ttl_ns),
        max_age_ns_(max_age_ns) {
    if (!max_sources_ || max_sources_ > 256 || !cost_ || cost_ > 252 ||
        max_ttl_ns_ <= 0 || max_age_ns_ <= 0)
      throw std::invalid_argument("invalid acoustic bounds");
  }
  // Called periodically even with no new messages. Coordinates MUST already
  // have been transformed using TF at the observation stamp.
  LayerUpdate tick(std::int64_t now_ns, const GridGeometry& grid) {
    LayerUpdate result;
    if (!valid(grid) || now_ns <= 0) {
      result.retired_cells = keys(owned_);
      owned_.clear();
      sources_.clear();
      latest_stamp_.clear();
      last_clock_ = 0;
      geometry_valid_ = false;
      return result;
    }
    const bool rewind = last_clock_ && now_ns < last_clock_;
    last_clock_ = now_ns;
    const bool changed = !geometry_valid_ || !same(grid, geometry_);
    result.geometry_changed = changed;
    if (rewind) { sources_.clear(); latest_stamp_.clear(); }
    for (auto it = sources_.begin(); it != sources_.end();) {
      if (it->second.expires_ns <= now_ns || it->second.stamp_ns > now_ns)
        it = sources_.erase(it);
      else ++it;
    }
    // Retain replay barriers through their original TTL, then prune them.
    for (auto it = latest_stamp_.begin(); it != latest_stamp_.end();) {
      if (it->second.second <= now_ns) it = latest_stamp_.erase(it);
      else ++it;
    }
    std::map<std::pair<unsigned, unsigned>, std::uint8_t> next;
    for (const auto& entry : sources_) {
      const auto& s = entry.second;
      const double col = std::floor((s.x_m - grid.origin_x) / grid.resolution);
      const double row = std::floor((s.y_m - grid.origin_y) / grid.resolution);
      if (col >= 0 && row >= 0 && col < grid.width && row < grid.height)
        next[{static_cast<unsigned>(col), static_cast<unsigned>(row)}] = cost_;
    }
    // Previous grid coordinates cannot be used to clear a shifted master map.
    if (!changed) {
      for (const auto& entry : owned_)
        if (!next.count(entry.first)) result.retired_cells.insert(entry.first);
    }
    owned_ = next;
    geometry_ = grid;
    geometry_valid_ = true;
    result.own_cells = owned_;
    return result;
  }
  bool ingest(const MetricCell& s, std::int64_t now_ns) {
    if (now_ns <= 0 || (last_clock_ && now_ns < last_clock_)) return false;
    // Subtract timestamps as unsigned integers only after ordering checks.
    // Signed subtraction can overflow on adversarial INT64_MAX timestamps.
    if (!s.physically_validated || !s.source_id || s.stamp_ns <= 0 ||
        s.stamp_ns > now_ns || s.expires_ns <= now_ns ||
        s.expires_ns <= s.stamp_ns || !std::isfinite(s.x_m) ||
        !std::isfinite(s.y_m)) return false;
    const auto ttl = static_cast<std::uint64_t>(s.expires_ns) -
                     static_cast<std::uint64_t>(s.stamp_ns);
    const auto age = static_cast<std::uint64_t>(now_ns) -
                     static_cast<std::uint64_t>(s.stamp_ns);
    if (ttl > static_cast<std::uint64_t>(max_ttl_ns_) ||
        age > static_cast<std::uint64_t>(max_age_ns_)) return false;
    auto old = sources_.find(s.source_id);
    // Preserve a bounded watermark after TTL/revocation to reject replayed
    // observations; a source may be reused only after its watermark expires.
    auto seen = latest_stamp_.find(s.source_id);
    if (seen != latest_stamp_.end() && s.stamp_ns <= seen->second.first)
      return false;
    if (old == sources_.end() && sources_.size() >= max_sources_) return false;
    if (seen == latest_stamp_.end() && latest_stamp_.size() >= max_sources_ * 4) {
      // Bounded replay history: fail closed for unseen IDs rather than evicting
      // a live replay barrier under pressure.
      return false;
    }
    sources_[s.source_id] = s;
    latest_stamp_[s.source_id] = {s.stamp_ns, s.expires_ns};
    return true;
  }
  void revoke(std::uint64_t id) { sources_.erase(id); }
  std::size_t source_count() const { return sources_.size(); }
 private:
  static bool valid(const GridGeometry& g) {
    return g.width && g.height && g.width <= 100000 && g.height <= 100000 &&
           std::isfinite(g.resolution) && g.resolution > 0 &&
           std::isfinite(g.origin_x) && std::isfinite(g.origin_y);
  }
  static bool same(const GridGeometry& a, const GridGeometry& b) {
    return a.width == b.width && a.height == b.height &&
           a.resolution == b.resolution && a.origin_x == b.origin_x &&
           a.origin_y == b.origin_y;
  }
  static std::set<std::pair<unsigned, unsigned>> keys(
      const std::map<std::pair<unsigned, unsigned>, std::uint8_t>& m) {
    std::set<std::pair<unsigned, unsigned>> result;
    for (const auto& e : m) result.insert(e.first);
    return result;
  }
  std::size_t max_sources_;
  std::uint8_t cost_;
  std::int64_t max_ttl_ns_;
  std::int64_t max_age_ns_;
  std::map<std::uint64_t, MetricCell> sources_;
  std::map<std::uint64_t, std::pair<std::int64_t, std::int64_t>> latest_stamp_;
  std::map<std::pair<unsigned, unsigned>, std::uint8_t> owned_;
  GridGeometry geometry_{};
  bool geometry_valid_ = false;
  std::int64_t last_clock_ = 0;
};
}  // namespace audition_nav2
