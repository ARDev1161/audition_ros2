#pragma once
// Translate an already validated cell-space dirty region to Nav2 world bounds.
// This helper never writes master costs and does not authorize observations.
#include <algorithm>
#include <cmath>
#include <limits>
#include "audition_nav2/acoustic_dirty_bounds.hpp"

namespace audition_nav2 {
struct WorldBounds {
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  bool valid = false;
};
class AcousticWorldBounds {
 public:
  static WorldBounds from_cells(const CellBounds& cells,
                                const GridGeometry& grid) {
    if (cells.empty() || !grid.width || !grid.height ||
        cells.min_i >= grid.width || cells.min_j >= grid.height ||
        cells.min_i >= cells.max_i || cells.min_j >= cells.max_j ||
        cells.max_i > grid.width || cells.max_j > grid.height ||
        !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
        !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y))
      return {};
    const double x0 = grid.origin_x + static_cast<double>(cells.min_i) * grid.resolution;
    const double y0 = grid.origin_y + static_cast<double>(cells.min_j) * grid.resolution;
    const double x1 = grid.origin_x + static_cast<double>(cells.max_i) * grid.resolution;
    const double y1 = grid.origin_y + static_cast<double>(cells.max_j) * grid.resolution;
    if (!std::isfinite(x0) || !std::isfinite(y0) ||
        !std::isfinite(x1) || !std::isfinite(y1) ||
        x0 >= x1 || y0 >= y1) return {};
    // Nav2 converts world bounds back to integer cells. Round outward by
    // one representable value so floating-point conversion cannot silently
    // exclude a dirty edge cell (especially for fractional resolutions).
    const double lo_x = std::nextafter(x0, -std::numeric_limits<double>::infinity());
    const double lo_y = std::nextafter(y0, -std::numeric_limits<double>::infinity());
    const double hi_x = std::nextafter(x1, std::numeric_limits<double>::infinity());
    const double hi_y = std::nextafter(y1, std::numeric_limits<double>::infinity());
    if (!std::isfinite(lo_x) || !std::isfinite(lo_y) ||
        !std::isfinite(hi_x) || !std::isfinite(hi_y)) return {};
    return {lo_x, lo_y, hi_x, hi_y, true};
  }
  // Nav2 passes an accumulated bounding box. Never shrink bounds from other
  // layers; invalid acoustic bounds must leave the accumulator unchanged.
  static bool expand(const WorldBounds& own, double* min_x, double* min_y,
                     double* max_x, double* max_y) {
    if (!own.valid || !min_x || !min_y || !max_x || !max_y ||
        !std::isfinite(*min_x) || !std::isfinite(*min_y) ||
        !std::isfinite(*max_x) || !std::isfinite(*max_y)) return false;
    *min_x = std::min(*min_x, own.min_x);
    *min_y = std::min(*min_y, own.min_y);
    *max_x = std::max(*max_x, own.max_x);
    *max_y = std::max(*max_y, own.max_y);
    return true;
  }
};
}  // namespace audition_nav2
