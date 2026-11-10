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

  // A partial Nav2 update must not touch any cell outside its exclusive
  // [min_i,max_i) x [min_j,max_j) window, even when the overlay contains it.
  LayerUpdate window;
  window.own_cells[{0,0}] = 80;
  window.own_cells[{1,0}] = 81;
  window.own_cells[{2,1}] = 82;
  window.own_cells[{3,2}] = 83;
  std::vector<std::uint8_t> clipped(12, 7);
  assert(AcousticMasterMerge::apply(window, grid, 1, 0, 3, 2,
                                    clipped.data(), clipped.size()) == 2);
  assert(clipped[0] == 7 && clipped[1] == 81 && clipped[6] == 82);
  assert(clipped[11] == 7);
  // Empty, inverted and out-of-range update windows are strictly inert.
  const auto before_window = clipped;
  assert(AcousticMasterMerge::apply(window, grid, 3, 2, 1, 0,
                                    clipped.data(), clipped.size()) == 0);
  assert(AcousticMasterMerge::apply(window, grid, 4, 0, 5, 3,
                                    clipped.data(), clipped.size()) == 0);
  assert(AcousticMasterMerge::apply(window, grid, 0, 3, 4, 4,
                                    clipped.data(), clipped.size()) == 0);
  assert(clipped == before_window);
  // Soft acoustic evidence cannot overwrite unknown, inscribed/inflated
  // collision costs or a stronger independent layer.
  LayerUpdate competing;
  for (unsigned i = 0; i < 4; ++i) competing.own_cells[{i,0}] = 252;
  std::vector<std::uint8_t> protected_master{255,254,253,252,
                                               0,0,0,0,0,0,0,0};
  assert(AcousticMasterMerge::apply(competing, grid, 0, 0, 4, 3,
                                    protected_master.data(),
                                    protected_master.size()) == 0);
  assert(protected_master[0] == 255 && protected_master[1] == 254 &&
         protected_master[2] == 253 && protected_master[3] == 252);
  // A malformed producer must never synthesize lethal/inscribed values.
  competing.own_cells.clear();
  competing.own_cells[{0,1}] = 253;
  competing.own_cells[{1,1}] = 254;
  competing.own_cells[{2,1}] = 255;
  assert(AcousticMasterMerge::apply(competing, grid, 0, 0, 4, 3,
                                    protected_master.data(),
                                    protected_master.size()) == 0);
  assert(protected_master[4] == 0 && protected_master[5] == 0 &&
         protected_master[6] == 0);
  // Simulate two sources competing for a cell: the higher existing
  // soft cost wins, and a later weaker observation cannot reduce it.
  LayerUpdate first, second;
  first.own_cells[{3,1}] = 180;
  second.own_cells[{3,1}] = 60;
  assert(AcousticMasterMerge::apply(first, grid, 0, 0, 4, 3,
                                    protected_master.data(),
                                    protected_master.size()) == 1);
  assert(AcousticMasterMerge::apply(second, grid, 0, 0, 4, 3,
                                    protected_master.data(),
                                    protected_master.size()) == 0);
  assert(protected_master[7] == 180);
  // The caller must rebuild the master from non-acoustic layers on TTL:
  // no stale acoustic value is removed by writing FREE_SPACE.
  protected_master[7] = 43; // independently rebuilt master value
  second.own_cells.clear();
  second.retired_cells.insert({3,1});
  assert(AcousticMasterMerge::apply(second, grid, 0, 0, 4, 3,
                                    protected_master.data(),
                                    protected_master.size()) == 0);
  assert(protected_master[7] == 43);
  // A fresh non-acoustic baseline is mandatory for TTL rollback.
  // A previously merged master is never a valid baseline.
  LayerUpdate acoustic;
  acoustic.own_cells[{1, 1}] = 110;
  acoustic.own_cells[{2, 1}] = 150;
  std::vector<std::uint8_t> baseline(12, 23);
  baseline[5] = 254;  // Independent obstacle wins.
  baseline[6] = 40;
  std::vector<std::uint8_t> composed(12, 77);
  std::size_t additions = 999;
  assert(AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 4, 3, baseline.data(), baseline.size(),
      composed.data(), composed.size(), &additions));
  assert(additions == 1 && composed[5] == 254 && composed[6] == 150);
  // No packets arrive: TTL expiry produces an empty overlay. Recomposition
  // must restore exactly the non-acoustic baseline, not write FREE_SPACE.
  acoustic.own_cells.clear();
  acoustic.retired_cells.insert({2, 1});
  assert(AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 4, 3, baseline.data(), baseline.size(),
      composed.data(), composed.size(), &additions));
  assert(additions == 0 && composed == baseline);
  // Partial Nav2 windows must not alter cells outside the dirty rectangle.
  composed.assign(12, 77);
  acoustic.own_cells[{2, 1}] = 80;
  assert(AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 2, 1, 3, 2, baseline.data(), baseline.size(),
      composed.data(), composed.size(), &additions));
  assert(additions == 1 && composed[6] == 80);
  for (std::size_t k = 0; k < composed.size(); ++k)
    if (k != 6) assert(composed[k] == 77);
  // Alias, overlap, truncated buffers and invalid geometry fail atomically.
  const auto before_reject = composed;
  assert(!AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 4, 3, composed.data(), composed.size(),
      composed.data(), composed.size(), &additions));
  assert(additions == 0 && composed == before_reject);
  std::vector<std::uint8_t> overlapping(24, 88);
  const auto before_overlap = overlapping;
  assert(!AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 4, 3, overlapping.data(), 12,
      overlapping.data() + 1, 12, &additions));
  assert(overlapping == before_overlap);
  assert(!AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 4, 3, baseline.data(), 11,
      composed.data(), composed.size(), &additions));
  assert(!AcousticMasterMerge::compose_from_baseline(
      acoustic, grid, 0, 0, 5, 3, baseline.data(), baseline.size(),
      composed.data(), composed.size(), &additions));
  assert(composed == before_reject);
  // A malformed geometry must never cause a partial baseline copy.
  GridGeometry invalid_grid = grid;
  invalid_grid.resolution = std::numeric_limits<double>::quiet_NaN();
  assert(!AcousticMasterMerge::compose_from_baseline(
      acoustic, invalid_grid, 0, 0, 4, 3, baseline.data(), baseline.size(),
      composed.data(), composed.size(), &additions));
  assert(composed == before_reject);
  return 0;
}
