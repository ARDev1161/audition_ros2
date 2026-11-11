// Negative and bounded-window contracts for the M5 independent-baseline path.
// Standalone: no Nav2 dependency; integration with LayeredCostmap is separate.
#include <cassert>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_baseline_transaction.hpp"

using namespace audition_nav2;

int main() {
  const GridGeometry geometry{8, 8, 0.1, 0.0, 0.0};
  std::vector<std::uint8_t> baseline(64, 7);
  std::vector<std::uint8_t> master(64, 23);
  LayerUpdate overlay;
  overlay.own_cells[{2, 3}] = 100;
  overlay.own_cells[{3, 3}] = 120;
  baseline[3 * 8 + 3] = 254;
  baseline[3 * 8 + 4] = 255;
  AcousticBaselineGate gate;
  BaselineTicket ticket{9, 1, BaselineOrigin::IndependentNonAcousticLayers,
                        geometry};
  std::size_t changes = 99;
  // Rebuild a strict subwindow. All other master cells must remain intact.
  assert(AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 5, 4,
      baseline.data(), baseline.size(), master.data(), master.size(), &changes));
  assert(changes == 1);
  for (unsigned y = 0; y < 8; ++y) {
    for (unsigned x = 0; x < 8; ++x) {
      const auto index = y * 8 + x;
      const bool in_window = y == 3 && x >= 2 && x < 5;
      const auto expected = !in_window ? 23 :
          (x == 2 ? 100 : (x == 3 ? 254 : 255));
      assert(master[index] == expected);
    }
  }

  const auto snapshot = master;
  // Invalid exclusive windows are atomic and consume their tickets.
  ticket.cycle = 2;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 9, 4,
      baseline.data(), baseline.size(), master.data(), master.size(), &changes));
  assert(changes == 0 && master == snapshot);
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 5, 4,
      baseline.data(), baseline.size(), master.data(), master.size()));

  // Overlapping but non-identical baseline/master pointers are forbidden.
  std::vector<std::uint8_t> shared(65, 5);
  ticket.cycle = 3;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 0, 0, 8, 8,
      shared.data(), 64, shared.data() + 1, 64));
  for (auto value : shared) assert(value == 5);

  // A short baseline or destination is never partially copied.
  ticket.cycle = 4;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 0, 0, 8, 8,
      baseline.data(), 63, master.data(), master.size()));
  assert(master == snapshot);
  ticket.cycle = 5;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 0, 0, 8, 8,
      baseline.data(), baseline.size(), master.data(), 63));
  assert(master == snapshot);

  // Unknown and previous-master origins must never authorize a write.
  ticket.cycle = 6;
  ticket.origin = BaselineOrigin::PreviousMaster;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 0, 0, 8, 8,
      baseline.data(), baseline.size(), master.data(), master.size()));
  ticket.origin = BaselineOrigin::Unknown;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 0, 0, 8, 8,
      baseline.data(), baseline.size(), master.data(), master.size()));
  assert(master == snapshot);

  // Malformed producer data outside the window is rejected atomically.
  ticket.origin = BaselineOrigin::IndependentNonAcousticLayers;
  ticket.cycle = 7;
  overlay.own_cells[{7, 7}] = 253;
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 5, 4,
      baseline.data(), baseline.size(), master.data(), master.size(), &changes));
  assert(changes == 0 && master == snapshot);
  overlay.own_cells.erase({7, 7});
  // Invalid producer data consumes admission; the same cycle cannot replay.
  assert(!AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 5, 4,
      baseline.data(), baseline.size(), master.data(), master.size()));

  // Empty overlay is a full restoration of this window, not FREE_SPACE.
  overlay.own_cells.clear();
  ticket.origin = BaselineOrigin::IndependentNonAcousticLayers;
  ticket.cycle = 8;
  assert(AcousticBaselineTransaction::apply(
      gate, ticket, 9, geometry, overlay, 2, 3, 5, 4,
      baseline.data(), baseline.size(), master.data(), master.size(), &changes));
  assert(changes == 0);
  for (unsigned x = 2; x < 5; ++x)
    assert(master[3 * 8 + x] == baseline[3 * 8 + x]);
  assert(master[0] == 23);
  return 0;
}
