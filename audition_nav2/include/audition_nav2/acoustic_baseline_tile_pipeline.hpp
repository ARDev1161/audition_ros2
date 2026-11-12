#pragma once
// Transactional, bounded staging of a non-acoustic baseline. The producer
// must reconstruct each tile independently; this class cannot certify that.
#include <cstddef>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_baseline_tile_cursor.hpp"

namespace audition_nav2 {
class AcousticBaselineTilePipeline {
 public:
  using Tile = AcousticBaselineTileCursor::Tile;
  bool begin(const BaselineTicket& ticket, unsigned tile_width, unsigned tile_height) {
    reset();
    if (!cursor_.begin(ticket, tile_width, tile_height) || !staging_.begin(ticket)) {
      reset();
      return false;
    }
    ticket_ = ticket;
    active_ = true;
    return true;
  }
  // Exactly one producer tile can be outstanding. A caller-supplied budget
  // limits the work admitted during a tick, not the total map size.
  bool next(const BaselineTicket& current, std::size_t budget, Tile& tile) {
    if (!active_ || !cursor_.matches(current) || !staging_.matches(current)) {
      reset();
      return false;
    }
    return cursor_.next(current, budget, tile);
  }
  bool submit(const BaselineTicket& producer, const Tile& tile,
              const std::uint8_t* costs, const std::uint8_t* coverage,
              std::size_t size) {
    if (!active_ || !cursor_.matches(producer) || !staging_.matches(producer) ||
        !staging_.add_checked(producer, tile.x, tile.y, tile.width,
                              tile.height, costs, coverage, size) ||
        !cursor_.acknowledge(producer, tile, true)) {
      reset();
      return false;
    }
    return true;
  }
  // Do not expose a partial buffer. A successful take consumes both states.
  bool take_ready(const BaselineTicket& current, std::vector<std::uint8_t>& out) {
    if (!active_ || !cursor_.finished(current) || !staging_.ready(current))
      return false;
    if (!staging_.take_ready(current, out)) {
      reset();
      return false;
    }
    reset();
    return true;
  }
  void reset() {
    cursor_.reset();
    staging_.reset();
    active_ = false;
  }
 private:
  BaselineTicket ticket_{};
  AcousticBaselineTileCursor cursor_;
  AcousticTiledBaseline staging_;
  bool active_ = false;
};
}  // namespace audition_nav2
