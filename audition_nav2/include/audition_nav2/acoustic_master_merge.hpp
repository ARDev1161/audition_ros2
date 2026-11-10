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
  // Transactional alternative for adapters that can provide an independently
  // reconstructed, non-acoustic master snapshot. Rebuild the requested region
  // from that snapshot on EVERY cycle, including an empty/expired overlay.
  // Never accept aliasing: copying from the previous acoustic master would
  // retain stale soft costs and violate TTL clearing.
  //
  // Returns false on malformed input without touching destination. A true
  // result means the region was rebuilt even if no acoustic cost was added.
  static bool compose_from_baseline(
      const LayerUpdate& update, const GridGeometry& geometry,
      unsigned min_i, unsigned min_j, unsigned max_i, unsigned max_j,
      const std::uint8_t* baseline, std::size_t baseline_size,
      std::uint8_t* destination, std::size_t destination_size,
      std::size_t* acoustic_changes = nullptr) {
    if (acoustic_changes) *acoustic_changes = 0;
    if (!baseline || !destination || baseline == destination ||
        !std::isfinite(geometry.resolution) ||
        !std::isfinite(geometry.origin_x) ||
        !std::isfinite(geometry.origin_y) ||
        geometry.resolution <= 0.0 ||
        geometry.width == 0 || geometry.height == 0 ||
        geometry.width > 100000 || geometry.height > 100000)
      return false;
    const auto count = static_cast<std::uint64_t>(geometry.width) *
                       geometry.height;
    if (count > 4000000ULL || count > baseline_size ||
        count > destination_size ||
        min_i >= max_i || min_j >= max_j ||
        max_i > geometry.width || max_j > geometry.height)
      return false;
    // Reject overlapping allocations, not only identical pointers. Compare
    // integer addresses to avoid undefined relational pointer comparisons.
    const auto src = reinterpret_cast<std::uintptr_t>(baseline);
    const auto dst = reinterpret_cast<std::uintptr_t>(destination);
    if (src > std::numeric_limits<std::uintptr_t>::max() - count ||
        dst > std::numeric_limits<std::uintptr_t>::max() - count ||
        (src < dst + count && dst < src + count))
      return false;
    for (unsigned j = min_j; j < max_j; ++j) {
      const auto row = static_cast<std::size_t>(j) * geometry.width;
      std::copy(baseline + row + min_i, baseline + row + max_i,
                destination + row + min_i);
    }
    const auto changes = apply(update, geometry, min_i, min_j, max_i, max_j,
                               destination, destination_size);
    if (acoustic_changes) *acoustic_changes = changes;
    return true;
  }

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
