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
  // Multi-source expiry must restore a fresh independent baseline.
  // Physical obstacles, unknown cells and stronger independent costs survive.
  const GridGeometry multi_geometry{6, 5, 0.1, -2.0, 3.0};
  std::vector<std::uint8_t> multi_base(30, 10);
  std::vector<std::uint8_t> multi_master(30, 90);
  multi_base[7] = 254;
  multi_base[8] = 255;
  multi_base[9] = 160;
  LayerUpdate multiple;
  multiple.own_cells[{0, 1}] = 80;
  multiple.own_cells[{1, 1}] = 120;
  multiple.own_cells[{2, 1}] = 120;
  multiple.own_cells[{3, 1}] = 100;
  multiple.own_cells[{4, 1}] = 110;
  AcousticBaselineGate multi_gate;
  BaselineTicket multi_ticket{1, 1,
      BaselineOrigin::IndependentNonAcousticLayers, multi_geometry};
  assert(AcousticBaselineTransaction::apply(
      multi_gate, multi_ticket, 1, multi_geometry, multiple,
      0, 1, 5, 2, multi_base.data(), multi_base.size(),
      multi_master.data(), multi_master.size()));
  assert(multi_master[6] == 80);
  assert(multi_master[7] == 254);
  assert(multi_master[8] == 255);
  assert(multi_master[9] == 160);
  assert(multi_master[10] == 110);
  assert(multi_master[5] == 90);  // Outside the dirty window.
  // Source 1 expires while source 2 remains.
  multiple.own_cells.erase({0, 1});
  multiple.own_cells.erase({1, 1});
  multi_ticket.cycle = 2;
  assert(AcousticBaselineTransaction::apply(
      multi_gate, multi_ticket, 1, multi_geometry, multiple,
      0, 1, 5, 2, multi_base.data(), multi_base.size(),
      multi_master.data(), multi_master.size()));
  assert(multi_master[6] == 10);
  assert(multi_master[7] == 254);
  assert(multi_master[8] == 255);
  assert(multi_master[9] == 160);
  assert(multi_master[10] == 110);
  // Expire all sources without a new observation: no ghost costs.
  multiple.own_cells.clear();
  multi_ticket.cycle = 3;
  assert(AcousticBaselineTransaction::apply(
      multi_gate, multi_ticket, 1, multi_geometry, multiple,
      0, 1, 5, 2, multi_base.data(), multi_base.size(),
      multi_master.data(), multi_master.size()));
  for (unsigned x = 0; x < 5; ++x) {
    assert(multi_master[6 + x] == multi_base[6 + x]);
  }
  // A replay of cycle 2 cannot resurrect expired source cells.
  const auto expired_snapshot = multi_master;
  multi_ticket.cycle = 2;
  multiple.own_cells[{0, 1}] = 80;
  assert(!AcousticBaselineTransaction::apply(
      multi_gate, multi_ticket, 1, multi_geometry, multiple,
      0, 1, 5, 2, multi_base.data(), multi_base.size(),
      multi_master.data(), multi_master.size()));
  assert(multi_master == expired_snapshot);
  return 0;
}
