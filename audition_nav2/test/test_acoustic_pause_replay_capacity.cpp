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
  // Nav2 can stop calling updateMap for longer than one source's TTL.
  cycle = layer.tick(t + 600000000LL, grid);
  assert(layer.source_count() == 1);
  assert(cycle.update.own_cells.size() == 1);
  assert(!cycle.update.retired_cells.count({2, 2}));
  // Replayed source #1 must not consume capacity or recreate an old cell.
  assert(!layer.ingest_validated({1, t, t + 500000000LL, 1.0, 1.0, true},
                                 t + 600000000LL));
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
  return 0;
}
