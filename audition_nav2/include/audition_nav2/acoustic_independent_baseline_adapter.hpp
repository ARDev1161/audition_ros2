#pragma once
// Explicit non-acoustic snapshot adapter. The provider MUST reconstruct its
// output from independently owned Nav2 layers, never from the prior master.
// This contract does not itself prove the provider's provenance and MUST NOT
// be wired to AcousticCostmapLayer until that Nav2 provider is implemented.
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <new>
#include <vector>
#include "audition_nav2/acoustic_baseline_transaction.hpp"

namespace audition_nav2 {
class AcousticIndependentBaselineAdapter {
 public:
  // No master pointer is exposed to the provider. The callback must fill
  // exactly count bytes from non-acoustic layers, in current map coordinates.
  using Rebuild = std::function<bool(const GridGeometry&, std::uint8_t*, std::size_t)>;

  // A failed reconstruction never changes master or consumes an admission.
  // A successful reconstruction is bound to the supplied generation/cycle;
  // AcousticBaselineTransaction consumes the ticket even on merge failure.
  static bool rebuild_and_apply(
      AcousticBaselineGate& gate, const BaselineTicket& ticket,
      std::uint64_t generation, const GridGeometry& geometry,
      const LayerUpdate& overlay, unsigned min_i, unsigned min_j,
      unsigned max_i, unsigned max_j, const Rebuild& rebuild,
      std::uint8_t* master, std::size_t master_size,
      std::size_t* changes = nullptr) {
    if (changes) *changes = 0;
    const auto count = static_cast<std::uint64_t>(geometry.width) * geometry.height;
    // Validate bounds before subtracting unsigned coordinates.
    if (min_i >= max_i || min_j >= max_j ||
        max_i > geometry.width || max_j > geometry.height) return false;
    const auto area = static_cast<std::uint64_t>(max_i - min_i) *
                      static_cast<std::uint64_t>(max_j - min_j);
    if (!rebuild || !master || ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        ticket.generation != generation || generation == 0 || ticket.cycle == 0 ||
        ticket.geometry.width != geometry.width ||
        ticket.geometry.height != geometry.height ||
        ticket.geometry.resolution != geometry.resolution ||
        ticket.geometry.origin_x != geometry.origin_x ||
        ticket.geometry.origin_y != geometry.origin_y ||
        !std::isfinite(geometry.resolution) || geometry.resolution <= 0 ||
        !std::isfinite(geometry.origin_x) || !std::isfinite(geometry.origin_y) ||
        geometry.width == 0 || geometry.height == 0 ||
        count > 4000000ULL || count > master_size ||
        area > AcousticMasterMerge::kMaxRebuildCells ||
        overlay.own_cells.size() > AcousticMasterMerge::kMaxOverlayCells)
      return false;
    // Reject malformed overlays before invoking an expensive third-party
    // reconstruction callback. Validate even cells outside the dirty window:
    // the transaction must never silently accept invalid producer data.
    for (const auto& cell : overlay.own_cells) {
      if (cell.first.first >= geometry.width ||
          cell.first.second >= geometry.height ||
          cell.second == 0 || cell.second > 252) return false;
    }
    // Reject replay and stale generations before allocating/calling the provider.
    // The transaction rechecks and consumes admission after reconstruction.
    if (!gate.can_accept(ticket, generation, geometry)) return false;
    // Private storage prevents accidental pointer aliasing with the master.
    // No costmap writes happen until reconstruction has fully succeeded.
    std::vector<std::uint8_t> baseline;
    try {
      baseline.resize(static_cast<std::size_t>(count));
      if (!rebuild(geometry, baseline.data(), baseline.size())) return false;
    } catch (const std::bad_alloc&) {
      return false;
    } catch (...) {
      // A third-party reconstruction callback may throw. Nav2 callbacks
      // must fail closed and must not leak exceptions into its update loop.
      return false;
    }
    return AcousticBaselineTransaction::apply(
        gate, ticket, generation, geometry, overlay,
        min_i, min_j, max_i, max_j,
        baseline.data(), baseline.size(), master, master_size, changes);
  }
};
}  // namespace audition_nav2
