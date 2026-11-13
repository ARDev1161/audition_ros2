#pragma once
// Bounded composition of independently reconstructed non-acoustic Nav2 tiles.
// This helper does NOT read a master costmap and cannot certify callback
// provenance. A future Nav2 adapter must supply ONLY non-acoustic layer
// callbacks and must exclude this plugin by identity before invoking it.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>
#include "audition_nav2/acoustic_baseline_tile_pipeline.hpp"

namespace audition_nav2 {
class AcousticLayerTileCompositor {
 public:
  using Tile = AcousticBaselineTilePipeline::Tile;
  using Layer = std::function<bool(const BaselineTicket&, const Tile&,
                                   std::vector<std::uint8_t>&,
                                   std::vector<std::uint8_t>&)>;
  // All callbacks must reconstruct their own layer tile, not read the
  // acoustic-contaminated master. The caller is responsible for validating
  // layer identities and Nav2 ordering before constructing this list.
  static bool compose(const BaselineTicket& ticket, const Tile& tile,
                      const std::vector<Layer>& layers,
                      std::vector<std::uint8_t>& costs,
                      std::vector<std::uint8_t>& coverage) {
    if (ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        ticket.generation == 0 || ticket.cycle == 0 ||
        layers.empty() || layers.size() > 64 ||
        !tile.width || !tile.height ||
        tile.x >= ticket.geometry.width || tile.y >= ticket.geometry.height ||
        tile.width > ticket.geometry.width - tile.x ||
        tile.height > ticket.geometry.height - tile.y)
      return false;
    const auto n = static_cast<std::size_t>(tile.width) * tile.height;
    if (n > AcousticTiledBaseline::kMaxTileCells ||
        costs.size() != n || coverage.size() != n) return false;
    // Never leave a partially composed result on a failed callback.
    std::vector<std::uint8_t> composed(n, 0), complete(n, 0);
    try {
      for (const auto& layer : layers) {
        if (!layer) return false;
        std::vector<std::uint8_t> values(n, 0), marks(n, 0);
        if (!layer(ticket, tile, values, marks) ||
            values.size() != n || marks.size() != n) return false;
        for (std::size_t i = 0; i < n; ++i) {
          if (marks[i] != 1) return false;
          // Conservative maximum is NOT a substitute for Nav2 layer
          // combination semantics. The provider must opt into this contract
          // only when all participating layers use max-style composition.
          composed[i] = std::max(composed[i], values[i]);
          complete[i] = 1;
        }
      }
    } catch (...) {
      return false;
    }
    costs.swap(composed);
    coverage.swap(complete);
    return true;
  }
};
}  // namespace audition_nav2
