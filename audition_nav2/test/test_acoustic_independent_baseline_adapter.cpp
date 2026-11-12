#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "audition_nav2/acoustic_independent_baseline_adapter.hpp"
using namespace audition_nav2;

int main() {
  const GridGeometry geometry{20, 20, 0.1, 0, 0};
  std::vector<std::uint8_t> master(400, 7);
  std::vector<std::uint8_t> independent(400, 11);
  independent[2 * 20 + 3] = 254;
  independent[2 * 20 + 4] = 255;
  LayerUpdate overlay;
  overlay.own_cells[{2, 2}] = 120;
  overlay.own_cells[{3, 2}] = 120;
  overlay.own_cells[{4, 2}] = 120;
  AcousticBaselineGate gate;
  auto ticket = [&](std::uint64_t cycle) {
    return BaselineTicket{17, cycle,
        BaselineOrigin::IndependentNonAcousticLayers, geometry};
  };
  unsigned calls = 0;
  const auto rebuild = [&](const GridGeometry&, std::uint8_t* dst, std::size_t n) {
    ++calls;
    assert(n == independent.size());
    std::copy(independent.begin(), independent.end(), dst);
    return true;
  };
  std::size_t changes = 999;
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size(), &changes));
  assert(calls == 1 && changes == 1);
  assert(master[2 * 20 + 2] == 120);
  assert(master[2 * 20 + 3] == 254);
  assert(master[2 * 20 + 4] == 255);
  // No incoming observations: reconstructing from independent layers
  // clears old audio costs without touching non-acoustic obstacles.
  overlay.own_cells.clear();
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == independent);
  // Replay is rejected, including when the callback is otherwise valid.
  const auto before = master;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == before);
  // Stale/replayed tickets must not invoke expensive reconstruction.
  assert(calls == 2);
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(calls == 2 && master == before);
  auto old_generation = ticket(100);
  old_generation.generation = 16;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, old_generation, 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(calls == 2 && master == before);
  // Callback failure and exceptions never mutate the master.
  const auto failed = [](const GridGeometry&, std::uint8_t*, std::size_t) {
    return false;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3), 17, geometry, overlay, 0, 0, 20, 20,
      failed, master.data(), master.size()));
  const auto throwing = [](const GridGeometry&, std::uint8_t*, std::size_t) -> bool {
    throw std::runtime_error("provider unavailable");
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3), 17, geometry, overlay, 0, 0, 20, 20,
      throwing, master.data(), master.size()));
  assert(master == before);
  // Failed provider does not consume cycle 3; fresh independent data can retry.
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, {20, 20, 0.1, 1, 0}, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == before);
  // A ticket claiming the previous master is never admitted.
  auto bad = ticket(4);
  bad.origin = BaselineOrigin::PreviousMaster;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, bad, 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == before);
  return 0;
}
