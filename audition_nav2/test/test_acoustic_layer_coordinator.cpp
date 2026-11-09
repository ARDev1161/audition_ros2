#include "audition_nav2/acoustic_layer_coordinator.hpp"
#include <cassert>
#include <cstdint>
#include <vector>
using namespace audition_nav2;
int main() {
  const GridGeometry g{8, 8, 1.0, 0.0, 0.0};
  AcousticLayerCoordinator layer(4, 80);
  assert(!layer.ingest_validated({1, 100, 200, 2, 3, false}, 110));
  assert(layer.ingest_validated({1, 100, 200, 2, 3, true}, 110));
  auto a = layer.tick(110, g);
  assert(a.rebuild_required && a.update.own_cells.size() == 1);
  std::vector<std::uint8_t> master(64, 0);
  master[3 * 8 + 2] = 254;
  assert(AcousticLayerCoordinator::overlay_after_rebuild(a, g, 0, 0, 8, 8,
          master.data(), master.size()) == 0);
  assert(master[3 * 8 + 2] == 254);
  master[3 * 8 + 2] = 0; // simulated reset from underlying layers
  assert(AcousticLayerCoordinator::overlay_after_rebuild(a, g, 0, 0, 8, 8,
          master.data(), master.size()) == 1);
  assert(master[3 * 8 + 2] == 80);
  layer.disable();
  assert(!layer.ingest_validated({2, 120, 220, 4, 4, true}, 120));
  auto cleared = layer.tick(120, g);
  assert(cleared.rebuild_required && cleared.update.own_cells.empty());
  assert(cleared.update.retired_cells.count({2, 3}) == 1);
  master[3 * 8 + 2] = 0; // Nav2 reset, not acoustic direct clear
  assert(AcousticLayerCoordinator::overlay_after_rebuild(cleared, g, 0, 0, 8, 8,
          master.data(), master.size()) == 0);
  layer.enable();
  assert(!layer.ingest_validated({1, 100, 200, 2, 3, true}, 130));
  assert(layer.ingest_validated({1, 131, 220, 2, 3, true}, 131));
  auto restored = layer.tick(131, g);
  assert(restored.rebuild_required);
  auto shifted = layer.tick(132, {8, 8, 1.0, 1.0, 0.0});
  assert(shifted.update.geometry_changed && shifted.dirty.max_i == 8);
  auto expired = layer.tick(221, {8, 8, 1.0, 1.0, 0.0});
  assert(expired.rebuild_required && expired.update.own_cells.empty());
  assert(!expired.update.retired_cells.empty());
  auto quiet = layer.tick(222, {8, 8, 1.0, 1.0, 0.0});
  assert(!quiet.rebuild_required);
}
