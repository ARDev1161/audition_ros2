#pragma once
// Dirty-region contract for a future Nav2 CostmapLayer. Never writes to master.
// Nav2 must reset the requested region from its underlying layers before
// AcousticMasterMerge::apply overlays our current soft costs.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_cell_buffer.hpp"

namespace audition_nav2 {
struct CellBounds {
  unsigned min_i = 0, min_j = 0, max_i = 0, max_j = 0; // exclusive max
  bool empty() const { return min_i >= max_i || min_j >= max_j; }
};
class AcousticDirtyBounds {
 public:
  // Force full rebuild on geometry changes, because indices from a previous
  // rolling map/resize cannot be used to identify old world-space costs.
  static CellBounds compute(const LayerUpdate& update, const GridGeometry& grid) {
    if (!grid.width || !grid.height || grid.width > 100000 ||
        grid.height > 100000 ||
        static_cast<std::uint64_t>(grid.width) * grid.height > 4000000ULL ||
        !std::isfinite(grid.resolution) || grid.resolution <= 0 ||
        !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y))
      return {};
    if (update.geometry_changed) return {0, 0, grid.width, grid.height};
    CellBounds out;
    bool found = false;
    const auto include = [&](unsigned i, unsigned j) {
      if (i >= grid.width || j >= grid.height) return;
      if (!found) {
        out = {i, j, i + 1, j + 1};
        found = true;
      } else {
        out.min_i = std::min(out.min_i, i);
        out.min_j = std::min(out.min_j, j);
        out.max_i = std::max(out.max_i, i + 1);
        out.max_j = std::max(out.max_j, j + 1);
      }
    };
    for (const auto& cell : update.changed_cells)
      include(cell.first, cell.second);
    for (const auto& cell : update.retired_cells)
      include(cell.first, cell.second);
    return out;
  }
};
} // namespace audition_nav2
