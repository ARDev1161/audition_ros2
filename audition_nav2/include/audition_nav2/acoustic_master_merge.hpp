#pragma once
// M5 master-costmap merge contract. This is NOT a Nav2 plugin.
// Nav2 must supply a freshly rebuilt master region before each merge;
// never "clear" an acoustic cell by writing FREE_SPACE into the master.
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include "audition_nav2/acoustic_cell_buffer.hpp"

namespace audition_nav2 {
class AcousticMasterMerge {
 public:
  // Nav2 constants: FREE_SPACE=0, LETHAL_OBSTACLE=254, NO_INFORMATION=255.
  // Unknown cells are never turned into evidence of free/occupied space.
  // Only a bounded acoustic soft cost may be added to a known master cell.
  static std::size_t apply(
      const LayerUpdate& update, const GridGeometry& geometry,
      unsigned min_i, unsigned min_j, unsigned max_i, unsigned max_j,
      std::uint8_t* master, std::size_t master_size) {
    if (!master || !std::isfinite(geometry.resolution) ||
        !std::isfinite(geometry.origin_x) ||
        !std::isfinite(geometry.origin_y) || geometry.resolution <= 0.0 ||
        geometry.width == 0 || geometry.height == 0 ||
        geometry.width > 100000 || geometry.height > 100000 ||
        static_cast<std::uint64_t>(geometry.width) * geometry.height > 4000000ULL ||
        static_cast<std::uint64_t>(geometry.width) * geometry.height > master_size)
      return 0;
    const unsigned end_i = std::min(max_i, geometry.width);
    const unsigned end_j = std::min(max_j, geometry.height);
    if (min_i >= end_i || min_j >= end_j) return 0;
    std::size_t changed = 0;
    // O(number of validated acoustic cells), not O(master grid area).
    for (const auto& item : update.own_cells) {
      const auto i = item.first.first, j = item.first.second;
      const auto cost = item.second;
      if (i < min_i || i >= end_i || j < min_j || j >= end_j ||
          cost == 0 || cost > 252) continue;
      const auto offset = static_cast<std::size_t>(j) * geometry.width + i;
      const auto previous = master[offset];
      if (previous == 255 || previous >= 253 || previous >= cost) continue;
      master[offset] = cost;
      ++changed;
    }
    return changed;
  }
};
}  // namespace audition_nav2
