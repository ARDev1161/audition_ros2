// Regression: replay barriers and bounded ownership survive Nav2 update pauses.
// Standalone C++ contract; no ROS or audio backend required.
#include <cassert>
#include <cstdint>
#include "audition_nav2/acoustic_layer_coordinator.hpp"

int main() {
  using namespace audition_nav2;
  const GridGeometry grid{20, 20, 0.5, 0.0, 0.0};
  AcousticLayerCoordinator layer(2, 100);
  layer.enable();
  constexpr std::int64_t t = 10000000000LL;
  assert(layer.ingest_validated({1, t, t + 500000000LL, 1.0, 1.0, true}, t));
  assert(layer.ingest_validated({2, t, t + 900000000LL, 1.0, 1.0, true}, t));
  // Capacity is strict: a third source cannot evict a live one.
  assert(!layer.ingest_validated({3, t, t + 900000000LL, 2.0, 2.0, true}, t));
  auto cycle = layer.tick(t + 100000000LL, grid);
  assert(cycle.update.own_cells.size() == 1);
  assert(cycle.update.own_cells.at({2, 2}) == 100);
  // Wall timer expires sources without consuming the dirty master region.
  layer.expire_without_tick(t + 600000000LL);
  assert(layer.source_count() == 1);
  // Nav2 can stop calling updateMap for longer than one source's TTL.
  cycle = layer.tick(t + 600000000LL, grid);
  assert(layer.source_count() == 1);
  assert(cycle.update.own_cells.size() == 1);
  assert(!cycle.update.retired_cells.count({2, 2}));
  // Replayed source #1 must not consume capacity or recreate an old cell.
  assert(!layer.ingest_validated({1, t, t + 500000000LL, 1.0, 1.0, true},
                                 t + 600000000LL));
  layer.expire_without_tick(t + 901000000LL);
  assert(layer.source_count() == 0);
  // The old cell remains owned until the next Nav2 cycle requests clearing.
  cycle = layer.tick(t + 901000000LL, grid);
  assert(cycle.update.own_cells.empty());
  assert(cycle.update.retired_cells.count({2, 2}));
  assert(cycle.rebuild_required);
  // Explicit disable is a rollback barrier; only fresh observations can
  // repopulate the map after reauthorization.
  layer.disable();
  assert(!layer.ingest_validated({2, t + 902000000LL, t + 1200000000LL,
                                  1.0, 1.0, true}, t + 902000000LL));
  layer.enable();
  assert(!layer.ingest_validated({2, t, t + 900000000LL, 1.0, 1.0, true},
                                 t + 903000000LL));
  assert(layer.ingest_validated({2, t + 904000000LL, t + 1200000000LL,
                                 1.0, 1.0, true}, t + 904000000LL));
  cycle = layer.tick(t + 905000000LL, grid);
  assert(cycle.update.own_cells.count({2, 2}));
  // A backwards clock invalidates all old evidence, even if geometry matches.
  cycle = layer.tick(t - 1000000000LL, grid);
  assert(cycle.update.own_cells.empty());
  assert(layer.source_count() == 0);
  assert(cycle.update.retired_cells.count({2, 2}));
  // No delayed packet from the old time domain may re-enter immediately.
  assert(!layer.ingest_validated({2, t + 904000000LL, t + 1200000000LL,
                                  1.0, 1.0, true}, t - 900000000LL));
  // Deterministic high-churn contract: the maximum admitted set is fixed at
  // 32, independent of DDS replay volume and rolling/resize cycles. This
  // verifies bounded state/dirty ownership, not a CPU timing benchmark.
  AcousticLayerCoordinator stress(32, 100);
  stress.enable();
  constexpr std::int64_t base = 20000000000LL;
  const GridGeometry wide{64, 64, 0.25, -2.0, -2.0};
  const GridGeometry shifted{64, 64, 0.25, -1.0, -2.0};
  const GridGeometry resized{128, 128, 0.125, -2.0, -2.0};
  for (std::uint64_t id = 1; id <= 32; ++id) {
    const double x = 0.25 * static_cast<double>((id - 1) % 8);
    const double y = 0.25 * static_cast<double>((id - 1) / 8);
    assert(stress.ingest_validated(
        {id, base, base + 900000000LL, x, y, true}, base));
  }
  assert(!stress.ingest_validated(
      {33, base, base + 900000000LL, 3.0, 3.0, true}, base));
  for (int iteration = 0; iteration < 240; ++iteration) {
    const auto now = base + 1000000LL * (iteration + 1);
    const auto& geometry = iteration % 3 == 0 ? wide :
                           iteration % 3 == 1 ? shifted : resized;
    // A replay of the same observation must never add a 33rd source.
    assert(!stress.ingest_validated(
        {1, base, base + 900000000LL, 0.0, 0.0, true}, now));
    const auto current = stress.tick(now, geometry);
    assert(stress.source_count() == 32);
    assert(current.update.own_cells.size() <= 32);
    assert(current.update.changed_cells.size() <= 32);
    assert(current.update.retired_cells.size() <= 32);
    assert(current.rebuild_required);
    for (const auto& cell : current.update.own_cells)
      assert(cell.second == 100);
  }
  // Expiration without a Nav2 tick must retain the retirement region.
  stress.expire_without_tick(base + 901000000LL);
  assert(stress.source_count() == 0);
  const auto retired = stress.tick(base + 902000000LL, resized);
  assert(retired.update.own_cells.empty());
  assert(retired.rebuild_required);
  assert(retired.update.retired_cells.size() == 32);
  assert(!stress.tick(base + 903000000LL, resized).rebuild_required);
  // Rollback must not allow a stale packet to recreate a cell.
  stress.disable();
  assert(!stress.ingest_validated(
      {1, base, base + 900000000LL, 0.0, 0.0, true},
      base + 904000000LL));
  stress.enable();
  assert(!stress.ingest_validated(
      {1, base, base + 900000000LL, 0.0, 0.0, true},
      base + 904000000LL));
  return 0;
}
