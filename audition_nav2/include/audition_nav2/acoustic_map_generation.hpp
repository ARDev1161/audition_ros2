#pragma once
// Map generation authority for a future trusted Nav2 baseline adapter.
// This tracker does NOT attest that a baseline is independent of acoustic
// costs. The caller must still rebuild non-acoustic layers before composing.
#include <cmath>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_cell_buffer.hpp"

namespace audition_nav2 {
class AcousticMapGeneration {
 public:
  // Returns 0 (untrusted) until the first valid geometry is observed.
  // Any geometry change revokes previous tile cursors. Changes between
  // callbacks (including ABA shifts) MUST call invalidate() explicitly.
  std::uint64_t observe(const GridGeometry& grid) {
    if (!valid(grid)) {
      invalidate();
      return 0;
    }
    if (!initialized_ || !same(grid, geometry_)) {
      if (!advance()) return 0;
      geometry_ = grid;
      initialized_ = true;
    }
    return generation_;
  }

  // Must be called on reset, rolling-origin mutation, map reallocation and
  // non-acoustic baseline replacement even if geometry appears unchanged.
  void invalidate() {
    initialized_ = false;
    (void)advance();
  }

  std::uint64_t generation() const {
    return initialized_ ? generation_ : 0;
  }

 private:
  static bool valid(const GridGeometry& g) {
    return g.width > 0 && g.height > 0 &&
           g.width <= 100000 && g.height <= 100000 &&
           static_cast<std::uint64_t>(g.width) * g.height <= 4000000ULL &&
           std::isfinite(g.resolution) && g.resolution > 0.0 &&
           std::isfinite(g.origin_x) && std::isfinite(g.origin_y);
  }
  static bool same(const GridGeometry& a, const GridGeometry& b) {
    return a.width == b.width && a.height == b.height &&
           a.resolution == b.resolution &&
           a.origin_x == b.origin_x && a.origin_y == b.origin_y;
  }
  bool advance() {
    // Never wrap a token and accidentally authorize an ancient cursor.
    if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
      initialized_ = false;
      return false;
    }
    ++generation_;
    return true;
  }
  GridGeometry geometry_{};
  std::uint64_t generation_ = 0;
  bool initialized_ = false;
};
}  // namespace audition_nav2
