#pragma once
// Fail-closed preflight for a future Nav2 baseline adapter. This does not
// manufacture a non-acoustic master: the caller must independently reconstruct
// it and attest its provenance. Never use the previous acoustic master.
#include <cstdint>
#include "audition_nav2/acoustic_cell_buffer.hpp"
#include "audition_nav2/acoustic_master_merge.hpp"

namespace audition_nav2 {
enum class BaselineOrigin : std::uint8_t {
  Unknown = 0,
  IndependentNonAcousticLayers = 1,
  PreviousMaster = 2
};
struct BaselineTicket {
  std::uint64_t generation = 0;
  std::uint64_t cycle = 0;
  BaselineOrigin origin = BaselineOrigin::Unknown;
  GridGeometry geometry{};
};

// One-shot per-cycle gate: an old snapshot must never be replayed after TTL,
// resize, rolling shift, or an ABA return to the same geometric coordinates.
// The adapter must advance cycle on every independently reconstructed baseline.
class AcousticBaselineGate {
 public:
  void invalidate() noexcept {
    generation_ = 0;
    cycle_ = 0;
  }
  bool accept(const BaselineTicket& ticket,
              std::uint64_t current_generation,
              const GridGeometry& current_geometry) noexcept {
    if (ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        !ticket.generation || ticket.generation != current_generation ||
        !ticket.cycle || ticket.cycle <= cycle_ ||
        (generation_ && ticket.generation != generation_) ||
        ticket.geometry.width != current_geometry.width ||
        ticket.geometry.height != current_geometry.height ||
        ticket.geometry.resolution != current_geometry.resolution ||
        ticket.geometry.origin_x != current_geometry.origin_x ||
        ticket.geometry.origin_y != current_geometry.origin_y ||
        !valid(current_geometry)) {
      invalidate();
      return false;
    }
    generation_ = ticket.generation;
    cycle_ = ticket.cycle;
    return true;
  }
 private:
  static bool valid(const GridGeometry& g) noexcept {
    return g.width > 0 && g.height > 0 &&
           g.width <= 100000 && g.height <= 100000 &&
           static_cast<std::uint64_t>(g.width) * g.height <= 4000000ULL &&
           std::isfinite(g.resolution) && g.resolution > 0 &&
           std::isfinite(g.origin_x) && std::isfinite(g.origin_y);
  }
  std::uint64_t generation_ = 0;
  std::uint64_t cycle_ = 0;
};
} // namespace audition_nav2
