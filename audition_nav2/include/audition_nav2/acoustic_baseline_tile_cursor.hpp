#pragma once
// Bounded, deterministic work scheduling for an independent non-acoustic
// baseline producer. Scheduling does NOT attest provenance or write to Nav2.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_tiled_baseline.hpp"

namespace audition_nav2 {
class AcousticBaselineTileCursor {
 public:
  struct Tile { unsigned x, y, width, height; std::uint64_t lease = 0; };
  static constexpr std::size_t kMaxWorkPerTick = 65536u;

  bool begin(const BaselineTicket& ticket, unsigned tile_width,
             unsigned tile_height) {
    reset();
    const auto& g = ticket.geometry;
    const std::uint64_t cells = static_cast<std::uint64_t>(g.width) * g.height;
    const std::uint64_t tile_cells =
        static_cast<std::uint64_t>(tile_width) * tile_height;
    if (ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        ticket.generation == 0 || ticket.cycle == 0 ||
        g.width == 0 || g.height == 0 || cells > AcousticTiledBaseline::kMaxCells ||
        tile_width == 0 || tile_height == 0 ||
        tile_cells > AcousticTiledBaseline::kMaxTileCells ||
        !std::isfinite(g.resolution) || g.resolution <= 0 ||
        !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y))
      return false;
    ticket_ = ticket;
    tile_width_ = tile_width;
    tile_height_ = tile_height;
    active_ = true;
    return true;
  }

  // Caller must use the returned ticket when submitting the tile. The
  // cursor advances only after the caller confirms successful reconstruction.
  // At most budget cells are scheduled per invocation; no unbounded loop.
  bool next(const BaselineTicket& current, std::size_t budget, Tile& out) {
    if (!matches(current) || awaiting_ || budget == 0 ||
        budget > kMaxWorkPerTick || finished_) return false;
    const auto& g = ticket_.geometry;
    const unsigned w = std::min(tile_width_, g.width - x_);
    const unsigned h = std::min(tile_height_, g.height - y_);
    if (static_cast<std::uint64_t>(w) * h > budget) return false;
    out = {x_, y_, w, h, lease_};
    awaiting_ = true;
    return true;
  }

  // A tile lease is tied to one cursor transaction, not merely the map
  // ticket. Reusing a ticket after abort/begin must not admit late tiles.
  bool pending(const BaselineTicket& current, const Tile& tile) const {
    return matches(current) && awaiting_ && tile.lease == lease_ &&
        tile.x == x_ && tile.y == y_ &&
        tile.width == std::min(tile_width_, ticket_.geometry.width - x_) &&
        tile.height == std::min(tile_height_, ticket_.geometry.height - y_);
  }
  bool acknowledge(const BaselineTicket& current, const Tile& tile,
                   bool success) {
    if (!success || !pending(current, tile)) {
      reset();  // Reject stale, reordered or failed asynchronous work.
      return false;
    }
    awaiting_ = false;
    x_ += tile.width;
    if (x_ == ticket_.geometry.width) {
      x_ = 0;
      y_ += tile.height;
      if (y_ == ticket_.geometry.height) finished_ = true;
    }
    return true;
  }

  bool finished(const BaselineTicket& current) const {
    return matches(current) && finished_ && !awaiting_;
  }
  bool matches(const BaselineTicket& t) const {
    return active_ && t.generation == ticket_.generation &&
        t.cycle == ticket_.cycle && t.origin == ticket_.origin &&
        t.geometry.width == ticket_.geometry.width &&
        t.geometry.height == ticket_.geometry.height &&
        t.geometry.resolution == ticket_.geometry.resolution &&
        t.geometry.origin_x == ticket_.geometry.origin_x &&
        t.geometry.origin_y == ticket_.geometry.origin_y;
  }
  void reset() {
    ++lease_;  // Invalidate every previously issued tile, including ABA.
    active_ = awaiting_ = finished_ = false;
    x_ = y_ = tile_width_ = tile_height_ = 0;
  }
 private:
  BaselineTicket ticket_{};
  unsigned x_ = 0, y_ = 0, tile_width_ = 0, tile_height_ = 0;
  bool active_ = false, awaiting_ = false, finished_ = false;
  std::uint64_t lease_ = 0;
};
}  // namespace audition_nav2
