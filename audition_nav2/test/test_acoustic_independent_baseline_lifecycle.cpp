#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_independent_baseline_adapter.hpp"

using namespace audition_nav2;

// Deterministic integration contract for map geometry churn and multi-source
// expiration. No Nav2 runtime or audio DSP is required.
int main() {
  AcousticBaselineGate gate;
  const GridGeometry initial{8, 8, 0.1, 0.0, 0.0};
  const GridGeometry shifted{8, 8, 0.1, 0.4, 0.0};
  const GridGeometry resized{10, 8, 0.1, 0.4, 0.0};
  std::vector<std::uint8_t> master(80, 0);
  std::size_t calls = 0;
  const auto provider = [&](const GridGeometry& g, std::uint8_t* out,
                            std::uint8_t* coverage, std::size_t n) {
    ++calls;
    assert(n == static_cast<std::size_t>(g.width) * g.height);
    std::fill(out, out + n, static_cast<std::uint8_t>(12));
    std::fill(coverage, coverage + n, static_cast<std::uint8_t>(1));
    // Independent physical obstacle and unknown area must survive audio.
    out[2 * g.width + 2] = 254;
    out[2 * g.width + 3] = 255;
    return true;
  };
  const auto ticket = [](std::uint64_t generation, std::uint64_t cycle,
                         const GridGeometry& geometry) {
    return BaselineTicket{generation, cycle,
                          BaselineOrigin::IndependentNonAcousticLayers, geometry};
  };
  LayerUpdate sources;
  sources.own_cells[{1, 1}] = 80;
  sources.own_cells[{2, 2}] = 150;
  sources.own_cells[{3, 2}] = 150;
  sources.own_cells[{6, 6}] = 210;
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1, 1, initial), 1, initial, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  assert(master[1 * 8 + 1] == 80);
  assert(master[6 * 8 + 6] == 210);
  assert(master[2 * 8 + 2] == 254);
  assert(master[2 * 8 + 3] == 255);

  // Expire one source, preserve another. No packet is required: an empty
  // source set on the next independent rebuild clears all old soft costs.
  sources.own_cells.erase({1, 1});
  sources.own_cells.erase({2, 2});
  sources.own_cells.erase({3, 2});
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1, 2, initial), 1, initial, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  assert(master[1 * 8 + 1] == 12);
  assert(master[6 * 8 + 6] == 210);
  sources.own_cells.clear();
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1, 3, initial), 1, initial, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  assert(master[6 * 8 + 6] == 12);

  // Rolling-map origin changes require a new generation. A stale ticket
  // must be rejected BEFORE invoking the provider or changing master.
  const auto before_shift = master;
  const auto previous_calls = calls;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1, 4, initial), 2, shifted, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  assert(calls == previous_calls && master == before_shift);
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2, 1, shifted), 2, shifted, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  // ABA: returning to exactly the original origin cannot revive generation 1.
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1, 100, initial), 1, initial, sources, 0, 0, 8, 8,
      provider, master.data(), master.size()));
  assert(calls == previous_calls + 1);

  // Resize must reject old-generation tickets and reconstruct every cell
  // from the independent provider, including the newly added columns.
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2, 2, shifted), 3, resized, sources, 0, 0, 10, 8,
      provider, master.data(), master.size()));
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3, 1, resized), 3, resized, sources, 0, 0, 10, 8,
      provider, master.data(), master.size()));
  assert(master[9] == 12 && master[7 * 10 + 9] == 12);
  assert(master[2 * 10 + 2] == 254);
  assert(master[2 * 10 + 3] == 255);

  // A failed provider cannot consume admission or erase the last good map.
  const auto before_failure = master;
  const auto fail = [](const GridGeometry&, std::uint8_t* out,
                       std::uint8_t* coverage, std::size_t n) {
    std::fill(out, out + n, static_cast<std::uint8_t>(0));
    std::fill(coverage, coverage + n, static_cast<std::uint8_t>(1));
    return false;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3, 2, resized), 3, resized, sources, 0, 0, 10, 8,
      fail, master.data(), master.size()));
  assert(master == before_failure);
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3, 2, resized), 3, resized, sources, 0, 0, 10, 8,
      provider, master.data(), master.size()));
  assert(master == before_failure);
  return 0;
}
