#pragma once
// Bounded, resumable rebuild-window planner for a future trusted Nav2 adapter.
// Planning does not authorize any acoustic cost or modify a master grid.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
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
        !std::isfinite(grid.resolution) || grid.resolution <= 0.0 ||
        !std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y) ||
        grid.width > 100000 || grid.height > 100000 ||
        static_cast<std::uint64_t>(grid.width) * grid.height > 4000000ULL ||
        dirty.empty() || dirty.min_i >= dirty.max_i ||
        dirty.min_j >= dirty.max_j ||
        dirty.min_i >= grid.width || dirty.min_j >= grid.height ||
        dirty.max_i > grid.width || dirty.max_j > grid.height ||
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

// Session binds a tile cursor to one exact map generation. A rolling map,
// resize, or dirty-window change invalidates the session instead of silently
// resuming at an unrelated cell. The caller must rebuild a fresh independent
// baseline and explicitly begin a new session after invalidation.
class AcousticRebuildSession {
 public:
  bool begin(const CellBounds& dirty, const GridGeometry& geometry) {
    invalidate();
    std::uint64_t probe = 0;
    CellBounds first;
    if (!AcousticRebuildTiles::next(dirty, geometry, probe, first)) return false;
    dirty_ = dirty;
    geometry_ = geometry;
    generation_ = 0;
    active_ = true;
    return true;
  }

  // A generation token prevents ABA reuse when a rolling map moves away and
  // later returns to exactly the same geometry between update callbacks.
  // The producer must increment generation on reset/reallocation/rolling shift.
  bool begin(const CellBounds& dirty, const GridGeometry& geometry,
             std::uint64_t generation) {
    // Zero is reserved for the legacy, unversioned test path. A trusted
    // adapter must never silently downgrade to geometry-only ABA detection.
    if (generation == 0) {
      invalidate();
      return false;
    }
    if (!begin(dirty, geometry)) return false;
    generation_ = generation;
    return true;
  }

  bool next(const CellBounds& dirty, const GridGeometry& geometry,
            CellBounds& tile,
            std::uint64_t budget = AcousticMasterMerge::kMaxRebuildCells) {
    tile = {};
    if (generation_ != 0) {
      tile = {};
      invalidate();
      return false;
    }
    return next(dirty, geometry, 0, tile, budget);
  }

  bool next(const CellBounds& dirty, const GridGeometry& geometry,
            std::uint64_t generation, CellBounds& tile,
            std::uint64_t budget = AcousticMasterMerge::kMaxRebuildCells) {
    tile = {};
    if (!active_) return false;
    // The legacy zero/zero path remains available for isolated tests;
    // trusted sessions are created only with a nonzero generation.
    if (generation != generation_ || !same(dirty, geometry)) {
      invalidate();
      return false;
    }
    // Invalid scheduling parameters revoke the cursor: a later caller must
    // explicitly revalidate its baseline and start a new transaction.
    if (budget == 0 || budget > AcousticMasterMerge::kMaxRebuildCells) {
      invalidate();
      return false;
    }
    if (!AcousticRebuildTiles::next(dirty_, geometry_, cursor_, tile, budget)) {
      // A failed scheduling attempt must not leave a reusable admission.
      // Completion is handled immediately after the last successful tile.
      invalidate();
      return false;
    }
    // Revoke the session as soon as the final tile is issued, not only when
    // a caller happens to request one more tile. This prevents an apparently
    // active rebuild cursor from being retained across idle update cycles.
    const auto total = static_cast<std::uint64_t>(dirty_.max_i - dirty_.min_i) *
                       (dirty_.max_j - dirty_.min_j);
    if (cursor_ == total) active_ = false;
    return true;
  }

  void invalidate() { active_ = false; cursor_ = 0; }
  bool active() const { return active_; }
  std::uint64_t cursor() const { return cursor_; }

 private:
  bool same(const CellBounds& dirty, const GridGeometry& geometry) const {
    return dirty.min_i == dirty_.min_i && dirty.min_j == dirty_.min_j &&
           dirty.max_i == dirty_.max_i && dirty.max_j == dirty_.max_j &&
           geometry.width == geometry_.width &&
           geometry.height == geometry_.height &&
           geometry.resolution == geometry_.resolution &&
           geometry.origin_x == geometry_.origin_x &&
           geometry.origin_y == geometry_.origin_y;
  }
  CellBounds dirty_{};
  GridGeometry geometry_{};
  std::uint64_t cursor_ = 0;
  std::uint64_t generation_ = 0;
  bool active_ = false;
};
}  // namespace audition_nav2
