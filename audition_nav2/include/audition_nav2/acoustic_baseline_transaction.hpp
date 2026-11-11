#pragma once
// A bounded, one-shot transaction between a verified non-acoustic snapshot
// provider and the Nav2 master. This class does NOT certify snapshot origin:
// only a trusted adapter that rebuilt other layers may issue the ticket.
#include <cstddef>
#include <cstdint>
#include <vector>
#include <new>
#include "audition_nav2/acoustic_baseline_gate.hpp"
#include "audition_nav2/acoustic_master_merge.hpp"

namespace audition_nav2 {
class AcousticBaselineTransaction {
 public:
  // Never retain an old master as a baseline. This method consumes admission
  // before attempting the merge, including on failure.
  static bool apply(AcousticBaselineGate& gate,
                    const BaselineTicket& ticket,
                    std::uint64_t current_generation,
                    const GridGeometry& geometry,
                    const LayerUpdate& overlay,
                    unsigned min_i, unsigned min_j,
                    unsigned max_i, unsigned max_j,
                    const std::uint8_t* independent_baseline,
                    std::size_t baseline_size,
                    std::uint8_t* master, std::size_t master_size,
                    std::size_t* acoustic_changes = nullptr) {
    if (acoustic_changes) *acoustic_changes = 0;
    if (!gate.accept(ticket, current_generation, geometry)) return false;
    if (!gate.consume()) return false;
    // Compose into bounded scratch before committing. A malformed overlay,
    // window or aliased source must never partially overwrite the master.
    if (!independent_baseline || !master ||
        min_i >= max_i || min_j >= max_j ||
        max_i > geometry.width || max_j > geometry.height) return false;
    const auto area = static_cast<std::uint64_t>(max_i - min_i) *
                      (max_j - min_j);
    if (area > AcousticMasterMerge::kMaxRebuildCells) return false;
    const auto count = static_cast<std::uint64_t>(geometry.width) *
                       geometry.height;
    if (count > 4000000ULL || count > baseline_size ||
        count > master_size) return false;
    // The master is never a valid independent source, including overlapping
    // spans. A scratch buffer alone would hide this alias from the composer.
    const auto src = reinterpret_cast<std::uintptr_t>(independent_baseline);
    const auto dst = reinterpret_cast<std::uintptr_t>(master);
    if (src > UINTPTR_MAX - count || dst > UINTPTR_MAX - count ||
        (src < dst + count && dst < src + count)) return false;
    // Validate all producer cells before allocating up to 4 MiB of scratch.\n    // An invalid cell outside the requested window must also fail closed.\n    if (overlay.own_cells.size() > AcousticMasterMerge::kMaxOverlayCells)\n      return false;\n    for (const auto& cell : overlay.own_cells) {\n      if (cell.first.first >= geometry.width ||\n          cell.first.second >= geometry.height ||\n          cell.second == 0 || cell.second > 252) return false;\n    }\n    // A full-size scratch is bounded to 4M cells by the geometry contract.
    std::vector<std::uint8_t> scratch;
    // Never propagate an allocation failure out of Nav2 updateCosts().
    // The one-shot ticket has already been consumed; the master remains
    // untouched and a future cycle must obtain a fresh independent baseline.
    try {
      scratch.resize(static_cast<std::size_t>(count));
    } catch (const std::bad_alloc&) {
      return false;
    }
    std::size_t changes = 0;
    if (!AcousticMasterMerge::compose_from_baseline(
            overlay, geometry, min_i, min_j, max_i, max_j,
            independent_baseline, baseline_size, scratch.data(),
            scratch.size(), &changes)) return false;
    for (unsigned y = min_j; y < max_j; ++y) {
      const auto offset = static_cast<std::size_t>(y) * geometry.width;
      for (unsigned x = min_i; x < max_i; ++x)
        master[offset + x] = scratch[offset + x];
    }
    if (acoustic_changes) *acoustic_changes = changes;
    return true;
  }
};
} // namespace audition_nav2
