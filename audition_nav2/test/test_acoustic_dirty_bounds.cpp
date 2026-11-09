#include "audition_nav2/acoustic_dirty_bounds.hpp"
#include <cassert>
#include <cstdint>
using namespace audition_nav2;
int main() {
  const GridGeometry grid{10, 8, 0.1, 0., 0.};
  LayerUpdate update;
  assert(AcousticDirtyBounds::compute(update, grid).empty());
  update.own_cells[{3, 2}] = 100;
  auto b = AcousticDirtyBounds::compute(update, grid);
  assert(b.min_i == 3 && b.min_j == 2 && b.max_i == 4 && b.max_j == 3);
  // Expiration with no new observation still dirties the old cell.
  update.own_cells.clear();
  update.retired_cells.insert({3, 2});
  b = AcousticDirtyBounds::compute(update, grid);
  assert(b.min_i == 3 && b.max_i == 4 && b.min_j == 2 && b.max_j == 3);
  update.own_cells[{8, 7}] = 90;
  b = AcousticDirtyBounds::compute(update, grid);
  assert(b.min_i == 3 && b.min_j == 2 && b.max_i == 9 && b.max_j == 8);
  // Out-of-map indices never expand the dirty rectangle.
  update.retired_cells.insert({999, 999});
  b = AcousticDirtyBounds::compute(update, grid);
  assert(b.max_i == 9 && b.max_j == 8);
  // Rolling map or resize needs a full master rebuild; stale indices unsafe.
  update.geometry_changed = true;
  b = AcousticDirtyBounds::compute(update, grid);
  assert(b.min_i == 0 && b.min_j == 0 && b.max_i == 10 && b.max_j == 8);
  assert(AcousticDirtyBounds::compute(update, GridGeometry{0,8,1,0,0}).empty());
  assert(AcousticDirtyBounds::compute(update, GridGeometry{5000,5000,1,0,0}).empty());
  return 0;
}
