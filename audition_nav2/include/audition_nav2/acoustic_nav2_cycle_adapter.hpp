#pragma once
// Adapter to Nav2 Humble's *normal* LayeredCostmap updateMap transaction.
//
// Nav2 resets the requested master window and invokes plugins in sequence.
// A terminal acoustic plugin therefore sees the current non-acoustic costs
// already rebuilt by other plugins. NEVER call other layers' updateCosts()
// recursively: those callbacks may consume internal state.
// This helper is only usable from our own Nav2 updateCosts callback, after
// the matching updateBounds callback, with independently admitted metric
// observations. A direct caller cannot use roster order to certify evidence.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>
#include "audition_nav2/acoustic_layer_coordinator.hpp"

namespace audition_nav2 {
class AcousticNav2CycleAdapter final {
 public:
  // Any duplicate, null, non-terminal acoustic plugin or acoustic filter is
  // ambiguous: do not assume that the normal Nav2 rebuild has completed.
  template<typename Layer>
  static bool terminal_nonacoustic_baseline(
      const std::vector<std::shared_ptr<Layer>>* plugins,
      const std::vector<std::shared_ptr<Layer>>* filters,
      const Layer* acoustic) {
    if (!plugins || !filters || !acoustic || plugins->size() < 2 ||
        plugins->back().get() != acoustic) return false;
    std::set<const Layer*> seen;
    for (const auto& plugin : *plugins) {
      if (!plugin || !seen.insert(plugin.get()).second) return false;
    }
    for (const auto& filter : *filters) {
      if (!filter || !seen.insert(filter.get()).second) return false;
    }
    return true;
  }

  // Called ONLY at the end of LayeredCostmap's regular updateCosts sequence,
  // after resetMap + each prior layer's updateCosts; never clear old acoustic
  // cells in-place. TTL removal is achieved by dirty updateBounds requesting
  // Nav2's next normal reset/rebuild (overlay may be empty by then).
  // Apply the complete overlay on EVERY updateCosts call: another layer may
  // have dirtied a window even when our source set did not change.
  static std::size_t overlay_after_nav2_reset(
      const AcousticCycle& cycle, const GridGeometry& geometry,
      int min_i, int min_j, int max_i, int max_j,
      std::uint8_t* rebuilt, std::size_t rebuilt_size,
      bool terminal_proven) {
    if (!terminal_proven || min_i < 0 || min_j < 0 ||
        max_i <= min_i || max_j <= min_j ||
        static_cast<std::uint64_t>(max_i) > geometry.width ||
        static_cast<std::uint64_t>(max_j) > geometry.height)
      return 0;
    return AcousticMasterMerge::apply(
        cycle.update, geometry, static_cast<unsigned>(min_i),
        static_cast<unsigned>(min_j), static_cast<unsigned>(max_i),
        static_cast<unsigned>(max_j), rebuilt, rebuilt_size);
  }
};
}  // namespace audition_nav2
