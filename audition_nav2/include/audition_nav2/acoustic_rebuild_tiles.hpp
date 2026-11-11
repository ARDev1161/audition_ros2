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
    if (!begin(dirty, geometry)) return false;
    generation_ = generation;
    return true;
  }

  bool next(const CellBounds& dirty, const GridGeometry& geometry,
            CellBounds& tile,
            std::uint64_t budget = AcousticMasterMerge::kMaxRebuildCells) {
    tile = {};
    return next(dirty, geometry, generation_ == 0 ? 0 :
                std::numeric_limits<std::uint64_t>::max(), tile, budget);
  }

  bool next(const CellBounds& dirty, const GridGeometry& geometry,
            std::uint64_t generation, CellBounds& tile,
            std::uint64_t budget = AcousticMasterMerge::kMaxRebuildCells) {
    tile = {};
    if (!active_) return false;
    if (generation != generation_ || !same(dirty, geometry)) {
      invalidate();
      return false;
    }
    if (!AcousticRebuildTiles::next(dirty_, geometry_, cursor_, tile, budget)) {
      // An invalid budget must not silently complete the session.
      if (budget != 0 && budget <= AcousticMasterMerge::kMaxRebuildCells &&
          cursor_ == static_cast<std::uint64_t>(dirty_.max_i - dirty_.min_i) *
                         (dirty_.max_j - dirty_.min_j))
        active_ = false;
      return false;
    }
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
