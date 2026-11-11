#include <cassert>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_baseline_transaction.hpp"
using namespace audition_nav2;

int main() {
  const GridGeometry g{4, 3, 0.5, -1.0, 2.0};
  std::vector<std::uint8_t> baseline(12, 0), master(12, 42);
  baseline[1] = 254;
  baseline[2] = 255;
  baseline[3] = 175;
  LayerUpdate overlay;
  overlay.own_cells[{0, 0}] = 100;
  overlay.own_cells[{1, 0}] = 100;
  overlay.own_cells[{2, 0}] = 100;
  overlay.own_cells[{3, 0}] = 100;
  AcousticBaselineGate gate;
  BaselineTicket ticket{1, 1, BaselineOrigin::IndependentNonAcousticLayers, g};
  std::size_t changes = 0;
  assert(AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                            0, 0, 4, 3, baseline.data(),
                                            baseline.size(), master.data(),
                                            master.size(), &changes));
  assert(changes == 1);
  assert(master[0] == 100 && master[1] == 254 &&
         master[2] == 255 && master[3] == 175);
  // Replay must not overwrite the current master.
  const auto snapshot = master;
  assert(!AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                              0, 0, 4, 3, baseline.data(),
                                              baseline.size(), master.data(),
                                              master.size()));
  assert(master == snapshot);
  // TTL expiry: a fresh independent snapshot with an empty overlay clears
  // only our cost and preserves independent obstacles and unknowns.
  overlay.own_cells.clear();
  ticket.cycle = 2;
  assert(AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                            0, 0, 4, 3, baseline.data(),
                                            baseline.size(), master.data(),
                                            master.size()));
  assert(master == baseline);
  // Never certify a snapshot aliased to the previous acoustic master.
  ticket.cycle = 3;
  assert(!AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                              0, 0, 4, 3, master.data(),
                                              master.size(), master.data(),
                                              master.size()));
  assert(master == baseline);
  // Reject invalid overlay atomically, and consume the ticket.
  ticket.cycle = 4;
  overlay.own_cells[{1, 1}] = 253;
  assert(!AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                              0, 0, 4, 3, baseline.data(),
                                              baseline.size(), master.data(),
                                              master.size()));
  assert(master == baseline);
  overlay.own_cells.clear();
  assert(!AcousticBaselineTransaction::apply(gate, ticket, 1, g, overlay,
                                              0, 0, 4, 3, baseline.data(),
                                              baseline.size(), master.data(),
                                              master.size()));
  // Reject geometry drift even when ticket generation matches.
  ticket.cycle = 5;
  auto moved = g;
  moved.origin_x += 0.5;
  assert(!AcousticBaselineTransaction::apply(gate, ticket, 1, moved, overlay,
                                              0, 0, 4, 3, baseline.data(),
                                              baseline.size(), master.data(),
                                              master.size()));
  assert(master == baseline);
  // Large map, tiny rolling window: only the requested cells are touched.
  // Exercise offsets near the far corner and preserve every outside cell.
  const GridGeometry large{2000, 2000, 0.05, -20.0, 10.0};
  std::vector<std::uint8_t> other(4000000, 7);
  std::vector<std::uint8_t> combined(4000000, 99);
  const auto idx = [](unsigned x, unsigned y) {
    return static_cast<std::size_t>(y) * 2000 + x;
  };
  other[idx(1998, 1999)] = 254;
  other[idx(1999, 1999)] = 255;
  LayerUpdate corner;
  corner.own_cells[{1997, 1999}] = 80;
  corner.own_cells[{1998, 1999}] = 80;
  corner.own_cells[{1999, 1999}] = 80;
  AcousticBaselineGate corner_gate;
  BaselineTicket corner_ticket{1, 1,
      BaselineOrigin::IndependentNonAcousticLayers, large};
  std::size_t corner_changes = 0;
  assert(AcousticBaselineTransaction::apply(
      corner_gate, corner_ticket, 1, large, corner,
      1997, 1999, 2000, 2000, other.data(), other.size(),
      combined.data(), combined.size(), &corner_changes));
  assert(corner_changes == 1);
  assert(combined[idx(1997, 1999)] == 80);
  assert(combined[idx(1998, 1999)] == 254);
  assert(combined[idx(1999, 1999)] == 255);
  assert(combined[idx(1996, 1999)] == 99);
  assert(combined[idx(1997, 1998)] == 99);
  // No new observation: independent baseline restores only the expired
  // acoustic window while keeping independent lethal/unknown values.
  corner.own_cells.clear();
  corner_ticket.cycle = 2;
  assert(AcousticBaselineTransaction::apply(
      corner_gate, corner_ticket, 1, large, corner,
      1997, 1999, 2000, 2000, other.data(), other.size(),
      combined.data(), combined.size()));
  assert(combined[idx(1997, 1999)] == 7);
  assert(combined[idx(1998, 1999)] == 254);
  assert(combined[idx(1999, 1999)] == 255);
  assert(combined[idx(1996, 1999)] == 99);
  // A window above the synchronous budget is rejected atomically.
  corner_ticket.cycle = 3;
  const auto before = combined;
  assert(!AcousticBaselineTransaction::apply(
      corner_gate, corner_ticket, 1, large, corner,
      0, 0, 2000, 40, other.data(), other.size(),
      combined.data(), combined.size()));
  assert(combined == before);
  return 0;
}
