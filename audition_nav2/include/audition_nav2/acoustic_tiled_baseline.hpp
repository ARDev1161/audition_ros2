#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_baseline_gate.hpp"

namespace audition_nav2 {
// Staging only: this never certifies provenance and never writes to master.
// Each tile must be rebuilt from independently owned non-acoustic Nav2 layers.
class AcousticTiledBaseline {
 public:
  static constexpr std::size_t kMaxCells = 4000000u;
  static constexpr std::size_t kMaxTileCells = 65536u;
  bool begin(const BaselineTicket& ticket) {
    reset();
    const auto& g = ticket.geometry;
    const auto n = static_cast<std::uint64_t>(g.width) * g.height;
    if (ticket.origin != BaselineOrigin::IndependentNonAcousticLayers ||
        ticket.generation == 0 || ticket.cycle == 0 ||
        g.width == 0 || g.height == 0 || n > kMaxCells ||
        !std::isfinite(g.resolution) || g.resolution <= 0 ||
        !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y)) return false;
    try {
      cells_.resize(static_cast<std::size_t>(n));
      covered_.assign(static_cast<std::size_t>(n), 0);
    } catch (...) { reset(); return false; }
    ticket_ = ticket;
    active_ = true;
    return true;
  }
  // Reject overlaps, malformed coverage and oversized tiles atomically.
  bool add(unsigned x, unsigned y, unsigned width, unsigned height,
           const std::uint8_t* costs, const std::uint8_t* coverage,
           std::size_t size) {
    if (!active_ || !costs || !coverage || !width || !height ||
        x >= ticket_.geometry.width || y >= ticket_.geometry.height ||
        width > ticket_.geometry.width - x ||
        height > ticket_.geometry.height - y) return false;
    const auto n = static_cast<std::uint64_t>(width) * height;
    if (n > kMaxTileCells || n != size) return false;
    for (std::size_t k = 0; k < size; ++k) {
      const auto idx = static_cast<std::size_t>(y + k / width) *
                       ticket_.geometry.width + x + k % width;
      if (coverage[k] != 1 || covered_[idx]) return false;
    }
    for (std::size_t k = 0; k < size; ++k) {
      const auto idx = static_cast<std::size_t>(y + k / width) *
                       ticket_.geometry.width + x + k % width;
      cells_[idx] = costs[k];
      covered_[idx] = 1;
    }
    count_ += size;
    return true;
  }
  bool ready(const BaselineTicket& current) const {
    return active_ && count_ == cells_.size() &&
        current.generation == ticket_.generation &&
        current.cycle == ticket_.cycle &&
        current.origin == ticket_.origin &&
        current.geometry.width == ticket_.geometry.width &&
        current.geometry.height == ticket_.geometry.height &&
        current.geometry.resolution == ticket_.geometry.resolution &&
        current.geometry.origin_x == ticket_.geometry.origin_x &&
        current.geometry.origin_y == ticket_.geometry.origin_y;
  }
  const std::vector<std::uint8_t>& cells() const { return cells_; }
  void reset() { active_ = false; count_ = 0; cells_.clear(); covered_.clear(); }
 private:
  BaselineTicket ticket_{};
  bool active_ = false;
  std::size_t count_ = 0;
  std::vector<std::uint8_t> cells_;
  std::vector<std::uint8_t> covered_;
};
}  // namespace audition_nav2
