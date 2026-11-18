#pragma once
// Bounded composition of independently reconstructed non-acoustic Nav2 tiles.
// This helper does NOT read a master costmap and cannot certify callback
// provenance. A future Nav2 adapter must supply ONLY non-acoustic layer
// callbacks and must exclude this plugin by identity before invoking it.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>
#include "audition_nav2/acoustic_baseline_tile_pipeline.hpp"

namespace audition_nav2 {
class AcousticLayerTileCompositor {
 public:
  using Tile = AcousticBaselineTilePipeline::Tile;
  // Cap aggregate callback work per tile, not only tile area. A 64-layer
  // roster must not turn one bounded tile into unbounded update latency.
  static constexpr std::size_t kMaxLayerCellVisits = 262144u;
  using Layer = std::function<bool(const BaselineTicket&, const Tile&,
                                   std::vector<std::uint8_t>&,
                                   std::vector<std::uint8_t>&)>;
  // A provider must bind each callback to a distinct, verified non-acoustic
  // layer identity. These declarations are fail-closed admission checks, not
  // a way to infer provenance from arbitrary callbacks. A Nav2 adapter must
  // verify the actual plugin identity and combination method independently.
  struct VerifiedLayer {
    std::string identity;
    bool independent = false;
    bool max_combination = false;
    Layer reconstruct;
  };
  static bool compose_verified(const BaselineTicket& ticket, const Tile& tile,
                               const std::vector<VerifiedLayer>& sources,
                               const std::string& acoustic_identity,
                               std::vector<std::uint8_t>& costs,
                               std::vector<std::uint8_t>& coverage) {
    if (acoustic_identity.empty() || sources.empty() || sources.size() > 64)
      return false;
    // Preflight work limits before allocating/copying callbacks.
    if (!valid_tile(ticket, tile) ||
        sources.size() > kMaxLayerCellVisits / tile_cells(tile))
      return false;
    try {
      std::unordered_set<std::string> identities;
      std::vector<Layer> callbacks;
      callbacks.reserve(sources.size());
      for (const auto& source : sources) {
        if (source.identity.empty() || source.identity == acoustic_identity ||
            !source.independent || !source.max_combination ||
            !source.reconstruct || !identities.insert(source.identity).second)
          return false;
        callbacks.push_back(source.reconstruct);
      }
      return compose(ticket, tile, callbacks, costs, coverage);
    } catch (...) {
      // No allocation/callback exception may escape into Nav2 updateCosts.
      return false;
    }
  }
  // A roster may change while external callbacks reconstruct a tile (e.g.
  // plugin reload, reorder, or layer removal). Admit a tile only when the
  // provider's ordered roster and generation match before AND after work.
  // The provider must derive this snapshot from actual Nav2 plugin identity,
  // not from caller-controlled labels. This is a TOCTOU guard, not proof of
  // independent layer data or a replacement for Nav2 adapter validation.
  struct RosterSnapshot {
    std::uint64_t generation = 0;
    std::string acoustic_identity;
    std::vector<std::string> ordered_identities;
  };
  using SnapshotProvider = std::function<bool(RosterSnapshot&)>;
  static bool compose_verified_stable(
      const BaselineTicket& ticket, const Tile& tile,
      const std::vector<VerifiedLayer>& sources,
      const RosterSnapshot& expected, const SnapshotProvider& snapshot,
      std::vector<std::uint8_t>& costs,
      std::vector<std::uint8_t>& coverage) {
    // Preflight before invoking even the roster provider or copying caller
    // buffers. In particular, untrusted tile dimensions must never cause
    // unbounded allocation in the stable wrapper.
    if (!valid_tile(ticket, tile) || !snapshot ||
        expected.generation == 0 || expected.generation != ticket.generation ||
        expected.acoustic_identity.empty() ||
        sources.empty() || sources.size() > 64 ||
        sources.size() > kMaxLayerCellVisits / tile_cells(tile) ||
        expected.ordered_identities.size() != sources.size() ||
        costs.size() != tile_cells(tile) ||
        coverage.size() != tile_cells(tile)) return false;
    // Reject a forged or inconsistent roster before calling an external
    // snapshot provider. The snapshot check alone cannot validate declared
    // callback provenance: the verified layer contract must hold as well.
    try {
      std::unordered_set<std::string> identities;
      for (std::size_t i = 0; i < sources.size(); ++i) {
        const auto& source = sources[i];
        if (source.identity.empty() ||
            source.identity == expected.acoustic_identity ||
            !source.independent || !source.max_combination ||
            !source.reconstruct ||
            !identities.insert(source.identity).second ||
            expected.ordered_identities[i] != source.identity)
          return false;
      }
    } catch (...) {
      return false;
    }
    const auto matches = [&](const RosterSnapshot& current) {
      return current.generation == expected.generation &&
             current.acoustic_identity == expected.acoustic_identity &&
             current.ordered_identities == expected.ordered_identities;
    };
    try {
      RosterSnapshot before;
      if (!snapshot(before) || !matches(before)) return false;
      // Stage privately: a roster change or callback failure must never
      // partially replace the caller's previously committed buffers.
      std::vector<std::uint8_t> staged_costs = costs;
      std::vector<std::uint8_t> staged_coverage = coverage;
      if (!compose_verified(ticket, tile, sources, expected.acoustic_identity,
                            staged_costs, staged_coverage)) return false;
      RosterSnapshot after;
      if (!snapshot(after) || !matches(after)) return false;
      costs.swap(staged_costs);
      coverage.swap(staged_coverage);
      return true;
    } catch (...) {
      return false;
    }
  }
  // All callbacks must reconstruct their own layer tile, not read the
  // acoustic-contaminated master. The caller is responsible for validating
  // layer identities and Nav2 ordering before constructing this list.
  static bool compose(const BaselineTicket& ticket, const Tile& tile,
                      const std::vector<Layer>& layers,
                      std::vector<std::uint8_t>& costs,
                      std::vector<std::uint8_t>& coverage) {
    if (!valid_tile(ticket, tile) || layers.empty() || layers.size() > 64)
      return false;
    const auto n = tile_cells(tile);
    if (layers.size() > kMaxLayerCellVisits / n ||
        costs.size() != n || coverage.size() != n) return false;
    // Never leave a partially composed result on a failed callback.
    try {
      // Preflight the entire callback roster before invoking any provider.
      // A missing late callback must not run earlier, potentially stateful
      // Nav2 layer producers before the transaction is known to be invalid.
      for (const auto& layer : layers) {
        if (!layer) return false;
      }
      std::vector<std::uint8_t> composed(n, 0), complete(n, 0);
      for (const auto& layer : layers) {
        std::vector<std::uint8_t> values(n, 0), marks(n, 0);
        if (!layer(ticket, tile, values, marks) ||
            values.size() != n || marks.size() != n) return false;
        for (std::size_t i = 0; i < n; ++i) {
          if (marks[i] != 1 || values[i] == 255) return false;
          // Conservative maximum is NOT a substitute for Nav2 layer
          // combination semantics. Unknown (255) must be resolved by the
          // independent provider; numeric max would incorrectly turn it
          // into a permanent unknown cell, hiding known obstacles.
          // The provider must opt into this contract
          // only when all participating layers use max-style composition.
          composed[i] = std::max(composed[i], values[i]);
          complete[i] = 1;
        }
      }
      costs.swap(composed);
      coverage.swap(complete);
      return true;
    } catch (...) {
      return false;
    }
  }
 private:
  static std::size_t tile_cells(const Tile& tile) noexcept {
    return static_cast<std::size_t>(tile.width) * tile.height;
  }
  static bool valid_tile(const BaselineTicket& ticket, const Tile& tile) noexcept {
    if (ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        !ticket.generation || !ticket.cycle ||
        !ticket.geometry.width || !ticket.geometry.height ||
        !std::isfinite(ticket.geometry.resolution) ||
        ticket.geometry.resolution <= 0 ||
        !std::isfinite(ticket.geometry.origin_x) ||
        !std::isfinite(ticket.geometry.origin_y) ||
        !tile.width || !tile.height ||
        tile.x >= ticket.geometry.width || tile.y >= ticket.geometry.height ||
        tile.width > ticket.geometry.width - tile.x ||
        tile.height > ticket.geometry.height - tile.y)
      return false;
    // Division avoids overflow for untrusted dimensions.
    return static_cast<std::size_t>(tile.width) <=
           AcousticTiledBaseline::kMaxTileCells / tile.height;
  }
};
}  // namespace audition_nav2
