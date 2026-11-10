#include "audition_nav2/acoustic_master_merge.hpp"
#include <cassert>
#include <cstdint>
#include <limits>
#include <cmath>
#include <vector>
using namespace audition_nav2;
int main() {
  GridGeometry grid{4, 3, 1., 0., 0.};
  LayerUpdate u;
  u.own_cells[{0,0}] = 100;
  u.own_cells[{1,0}] = 100;
  u.own_cells[{2,0}] = 100;
  u.own_cells[{3,0}] = 100;
  u.own_cells[{1,1}] = 254; // malformed producer must not create lethal
  u.own_cells[{99,99}] = 100;
  std::vector<std::uint8_t> master(12, 0);
  master[0] = 255; // unknown remains unknown
  master[1] = 254; // obstacle remains lethal
  master[2] = 120; // higher pre-existing cost preserved
  assert(AcousticMasterMerge::apply(u,grid,0,0,4,3,master.data(),master.size()) == 1);
  assert(master[0] == 255 && master[1] == 254 && master[2] == 120);
  assert(master[3] == 100 && master[5] == 0);
  assert(AcousticMasterMerge::apply(u,grid,0,0,4,3,master.data(),master.size()) == 0);
  assert(AcousticMasterMerge::apply(u,grid,0,0,4,3,nullptr,master.size()) == 0);
  assert(AcousticMasterMerge::apply(u,grid,0,0,4,3,master.data(),1) == 0);
  // Nav2 recomputes the master region; the acoustic layer never clears
  // a previous cell itself. Expired costs vanish from the next merge.
  master[3] = 0;
  u.own_cells.clear();
  u.retired_cells.insert({3,0});
  assert(AcousticMasterMerge::apply(u,grid,0,0,4,3,master.data(),master.size()) == 0);
  assert(master[3] == 0);
  // Rolling geometry cannot reuse previous indices: only current own_cells.
  u.geometry_changed = true;
  u.own_cells[{2,2}] = 90;
  assert(AcousticMasterMerge::apply(u,grid,2,2,3,3,master.data(),master.size()) == 1);
  assert(master[10] == 90);
  assert(AcousticMasterMerge::apply(u,grid,0,0,2,2,master.data(),master.size()) == 0);
  // Invalid Nav2 geometry must never modify another layer's master costs.
  const auto before_invalid = master;
  const double bad[] = {std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()};
  for (const double value : bad) {
    for (int field = 0; field < 3; ++field) {
      GridGeometry malformed = grid;
      if (field == 0) malformed.resolution = value;
      if (field == 1) malformed.origin_x = value;
      if (field == 2) malformed.origin_y = value;
      assert(AcousticMasterMerge::apply(u, malformed, 0, 0, 4, 3,
                                        master.data(), master.size()) == 0);
      assert(master == before_invalid);
    }
  }
  for (const double resolution : {0.0, -0.5}) {
    GridGeometry malformed = grid;
    malformed.resolution = resolution;
    assert(AcousticMasterMerge::apply(u, malformed, 0, 0, 4, 3,
                                      master.data(), master.size()) == 0);
    assert(master == before_invalid);
  }
  return 0;
}
