#pragma once
// Bounded, resumable rebuild-window planner for a future trusted Nav2 adapter.
// Planning does not authorize any acoustic cost or modify a master grid.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include "audition_nav2/acoustic_dirty_bounds.hpp"
#include "audition_nav2/acoustic_master_merge.hpp"

namespace audition_nav2 {
class AcousticRebuildTiles {
 public:
  // Caller must retain the SAME geometry and dirty region until completion.
  // Any resize/rolling-origin change requires discarding the cursor and
  // starting a new reconstruction from independently rebuilt baseline data.
  static bool next(const CellBounds& dirty, const GridGeometry& grid,
                   std::uint64_t& cursor, CellBounds& tile,
                   std::uint64_t budget = AcousticMasterMerge::kMaxRebuildCells) {
    tile = {};
    if (grid.width == 0 || grid.height == 0 ||
        grid.width > 100000 || grid.height > 100000 ||
        static_cast<std::uint64_t>(grid.width) * grid.height > 4000000ULL ||
        dirty.empty() || dirty.max_i > grid.width ||
        dirty.max_j > grid.height ||
        budget == 0 || budget > AcousticMasterMerge::kMaxRebuildCells)
      return false;
    const std::uint64_t width = dirty.max_i - dirty.min_i;
    const std::uint64_t height = dirty.max_j - dirty.min_j;
    const std::uint64_t total = width * height;
    if (cursor >= total) return false;
    // Tiles never straddle row boundaries, making each update an exact
    // rectangular subregion compatible with compose_from_baseline().
    const std::uint64_t row = cursor / width;
    const std::uint64_t column = cursor % width;
    const std::uint64_t count = std::min(budget, width - column);
    tile = {static_cast<unsigned>(dirty.min_i + column),
            static_cast<unsigned>(dirty.min_j + row),
            static_cast<unsigned>(dirty.min_i + column + count),
            static_cast<unsigned>(dirty.min_j + row + 1)};
    cursor += count;
    return true;
  }
};
}  // namespace audition_nav2
