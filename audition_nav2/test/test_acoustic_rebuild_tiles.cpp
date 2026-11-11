#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>
#include "audition_nav2/acoustic_rebuild_tiles.hpp"

using namespace audition_nav2;
int main() {
  const GridGeometry grid{1024, 1024, 0.05, -10.0, -10.0};
  const CellBounds dirty{17, 29, 900, 330};
  std::vector<unsigned char> seen(1024 * 1024, 0);
  std::uint64_t cursor = 0;
  CellBounds tile;
  std::uint64_t visited = 0;
  std::uint64_t iterations = 0;
  while (AcousticRebuildTiles::next(dirty, grid, cursor, tile, 4096)) {
    assert(++iterations < 1000);
    assert(tile.min_i >= dirty.min_i && tile.max_i <= dirty.max_i);
    assert(tile.min_j >= dirty.min_j && tile.max_j <= dirty.max_j);
    assert(tile.max_j == tile.min_j + 1);
    assert(static_cast<std::uint64_t>(tile.max_i - tile.min_i) <= 4096);
    for (unsigned x = tile.min_i; x < tile.max_i; ++x) {
      auto& cell = seen[tile.min_j * grid.width + x];
      assert(cell == 0);
      cell = 1;
      ++visited;
    }
  }
  assert(visited == static_cast<std::uint64_t>(dirty.max_i - dirty.min_i) *
                        (dirty.max_j - dirty.min_j));
  assert(cursor == visited);
  assert(!AcousticRebuildTiles::next(dirty, grid, cursor, tile));
  assert(tile.empty());
  for (unsigned y = 0; y < grid.height; ++y)
    for (unsigned x = 0; x < grid.width; ++x)
      assert(seen[y * grid.width + x] ==
             static_cast<unsigned char>(x >= dirty.min_i && x < dirty.max_i &&
                                        y >= dirty.min_j && y < dirty.max_j));

  // Maximum supported map and budget, including a partial last row.
  const GridGeometry large{2000, 2000, 0.1, 0, 0};
  const CellBounds all{0, 0, 2000, 2000};
  cursor = 0;
  iterations = 0;
  while (AcousticRebuildTiles::next(all, large, cursor, tile)) {
    assert(++iterations <= 2000);
    assert(tile.max_i - tile.min_i <=
           AcousticMasterMerge::kMaxRebuildCells);
  }
  assert(cursor == 4000000ULL);
  // Reject invalid geometry, bounds, and budgets without cursor advancement.
  cursor = 0;
  const auto check_reject = [&](CellBounds region, GridGeometry geometry,
                                std::uint64_t budget) {
    CellBounds out{1, 1, 2, 2};
    assert(!AcousticRebuildTiles::next(region, geometry, cursor, out, budget));
    assert(cursor == 0);
    assert(out.empty());
  };
  check_reject({0, 0, 2001, 1}, large, 10);
  check_reject({0, 0, 1, 2001}, large, 10);
  check_reject({0, 0, 1, 1}, large, 0);
  check_reject({0, 0, 1, 1}, large,
               AcousticMasterMerge::kMaxRebuildCells + 1);
  check_reject({0, 0, 1, 1}, {2001, 2000, 0.1, 0, 0}, 10);
  check_reject({0, 0, 0, 1}, large, 10);
  // Geometry preflight rejects non-metric or non-finite map metadata.
  check_reject({0, 0, 1, 1}, {2000, 2000, 0, 0, 0}, 10);
  check_reject({0, 0, 1, 1}, {2000, 2000, -1, 0, 0}, 10);
  check_reject({0, 0, 1, 1},
               {2000, 2000, std::numeric_limits<double>::quiet_NaN(), 0, 0}, 10);
  check_reject({0, 0, 1, 1},
               {2000, 2000, 0.1, std::numeric_limits<double>::infinity(), 0}, 10);

  // A resumable session must never continue across a rolling origin, resize,
  // resolution change, or changed dirty window.
  const CellBounds small{2, 3, 12, 5};
  const GridGeometry stable{20, 20, 0.1, 0, 0};
  AcousticRebuildSession session;
  assert(session.begin(small, stable));
  assert(session.next(small, stable, tile, 4));
  assert(session.cursor() == 4);
  assert(tile.min_i == 2 && tile.max_i == 6 && tile.min_j == 3);
  assert(!session.next(small, {20, 20, 0.1, 0.1, 0}, tile, 4));
  assert(!session.active() && session.cursor() == 0 && tile.empty());
  assert(!session.next(small, stable, tile, 4));
  assert(session.begin(small, stable));
  assert(session.next(small, stable, tile, 4));
  assert(!session.next({2, 3, 13, 5}, stable, tile, 4));
  assert(!session.active());
  assert(session.begin(small, stable));
  assert(session.next(small, stable, tile, 4));
  assert(!session.next(small, {21, 20, 0.1, 0, 0}, tile, 4));
  assert(session.begin(small, stable));
  assert(session.next(small, stable, tile, 4));
  assert(!session.next(small, {20, 20, 0.2, 0, 0}, tile, 4));
  assert(session.begin(small, stable));
  assert(!session.next(small, stable, tile, 0));
  assert(!session.active() && session.cursor() == 0);
  assert(session.begin(small, stable));
  std::uint64_t completed = 0;
  while (session.next(small, stable, tile, 3)) {
    completed += tile.max_i - tile.min_i;
  }
  assert(completed == 20);
  assert(!session.active() && session.cursor() == 20);
  assert(!session.next(small, stable, tile, 3));
  assert(!session.begin(small, {20, 20, 0.0, 0, 0}));
  assert(!session.active());
  // Geometry alone cannot detect ABA: a rolling map may return to its
  // original origin between two callbacks. Epoch changes must revoke cursor.
  // A trusted (epoch-bound) session may never use the legacy zero token.
  assert(!session.begin(small, stable, 0));
  assert(!session.active());
  assert(session.begin(small, stable, 41));
  assert(session.next(small, stable, 41, tile, 4));
  assert(session.cursor() == 4);
  assert(!session.next(small, stable, 42, tile, 4));
  assert(!session.active() && session.cursor() == 0 && tile.empty());
  assert(session.begin(small, stable, 43));
  assert(!session.next(small, stable, 0, tile, 4));
  assert(!session.active() && session.cursor() == 0);
  assert(session.begin(small, stable, 43));
  assert(!session.next(small, stable, tile, 4));
  assert(!session.active() && tile.empty());
  assert(session.begin(small, stable, 44));
  std::uint64_t epoch_visited = 0;
  while (session.next(small, stable, 44, tile, 3))
    epoch_visited += tile.max_i - tile.min_i;
  assert(epoch_visited == 20);
  assert(!session.active());
  // Invalid budgets must revoke a partially processed transaction, rather
  // than allow resumption against a potentially stale baseline.
  assert(session.begin(small, stable, 45));
  assert(session.next(small, stable, 45, tile, 2));
  assert(!session.next(small, stable, 45, tile, 0));
  assert(!session.active() && session.cursor() == 0 && tile.empty());
  assert(session.begin(small, stable, 45));
  assert(session.next(small, stable, 45, tile, 2));
  assert(!session.next(small, stable, 45, tile,
                       AcousticMasterMerge::kMaxRebuildCells + 1));
  assert(!session.active() && session.cursor() == 0);
  assert(session.begin(small, stable, 45));
  assert(session.next(small, stable, 45, tile, 2));
  assert(!session.next(small, stable, 46, tile, 2));
  assert(session.begin(small, stable, 46));
  assert(session.next(small, stable, 46, tile, 2));
  assert(tile.min_i == small.min_i);
  // End-to-end bounded baseline reconstruction: an expired acoustic cell
  // must disappear from the master, while a non-acoustic obstacle survives.
  // This is a ROS-independent contract test, not permission to enable Nav2.
  const GridGeometry rebuild_grid{20, 20, 0.1, 0, 0};
  const CellBounds rebuild_region{0, 0, 20, 20};
  std::vector<std::uint8_t> baseline(400, 10);
  std::vector<std::uint8_t> master(400, 10);
  baseline[3 * 20 + 2] = 254;  // Independently owned lethal obstacle.
  LayerUpdate acoustic;
  acoustic.own_cells[{4, 3}] = 100;
  acoustic.own_cells[{2, 3}] = 100;
  AcousticRebuildSession rebuild;
  assert(rebuild.begin(rebuild_region, rebuild_grid, 101));
  std::size_t painted = 0;
  while (rebuild.next(rebuild_region, rebuild_grid, 101, tile, 7)) {
    std::size_t changes = 999;
    assert(AcousticMasterMerge::compose_from_baseline(
        acoustic, rebuild_grid, tile.min_i, tile.min_j, tile.max_i,
        tile.max_j, baseline.data(), baseline.size(), master.data(),
        master.size(), &changes));
    painted += changes;
  }
  assert(painted == 1);
  assert(master[3 * 20 + 4] == 100);
  assert(master[3 * 20 + 2] == 254);
  // No new acoustic messages: a subsequent baseline-only rebuild clears
  // the old soft cost without overwriting any independent obstacle.
  acoustic.own_cells.clear();
  assert(rebuild.begin(rebuild_region, rebuild_grid, 102));
  while (rebuild.next(rebuild_region, rebuild_grid, 102, tile, 7)) {
    std::size_t changes = 999;
    assert(AcousticMasterMerge::compose_from_baseline(
        acoustic, rebuild_grid, tile.min_i, tile.min_j, tile.max_i,
        tile.max_j, baseline.data(), baseline.size(), master.data(),
        master.size(), &changes));
    assert(changes == 0);
  }
  assert(master == baseline);
  // A geometry epoch change mid-rebuild must revoke the cursor. A caller
  // must not continue writing tiles against an unrelated baseline.
  assert(rebuild.begin(rebuild_region, rebuild_grid, 103));
  assert(rebuild.next(rebuild_region, rebuild_grid, 103, tile, 7));
  assert(!rebuild.next(rebuild_region, rebuild_grid, 104, tile, 7));
  assert(!rebuild.active() && rebuild.cursor() == 0 && tile.empty());
  // A multi-source overlay must retain independent lethal/unknown baseline
  // cells and clear *all* former soft costs when every source expires.
  std::vector<std::uint8_t> multi_baseline(400, 12);
  multi_baseline[1 * 20 + 1] = 254;
  multi_baseline[2 * 20 + 2] = 255;
  std::vector<std::uint8_t> multi_master = multi_baseline;
  LayerUpdate multi;
  multi.own_cells[{1, 1}] = 120;  // Must not replace a lethal obstacle.
  multi.own_cells[{2, 2}] = 90;   // Must not replace unknown space.
  multi.own_cells[{7, 7}] = 110;
  multi.own_cells[{18, 18}] = 145;
  assert(rebuild.begin(rebuild_region, rebuild_grid, 105));
  std::size_t multi_painted = 0;
  while (rebuild.next(rebuild_region, rebuild_grid, 105, tile, 5)) {
    std::size_t changes = 0;
    assert(AcousticMasterMerge::compose_from_baseline(
        multi, rebuild_grid, tile.min_i, tile.min_j, tile.max_i,
        tile.max_j, multi_baseline.data(), multi_baseline.size(),
        multi_master.data(), multi_master.size(), &changes));
    multi_painted += changes;
  }
  assert(multi_painted == 2);
  assert(multi_master[1 * 20 + 1] == 254);
  assert(multi_master[2 * 20 + 2] == 255);
  assert(multi_master[7 * 20 + 7] == 110);
  assert(multi_master[18 * 20 + 18] == 145);

  // An invalid cell outside the current tile must reject the entire
  // operation before any destination bytes are changed.
  LayerUpdate invalid = multi;
  invalid.own_cells[{19, 19}] = 254;
  const auto untouched = multi_master;
  std::size_t rejected_changes = 999;
  assert(!AcousticMasterMerge::compose_from_baseline(
      invalid, rebuild_grid, 0, 0, 5, 1,
      multi_baseline.data(), multi_baseline.size(),
      multi_master.data(), multi_master.size(), &rejected_changes));
  assert(multi_master == untouched);
  assert(rejected_changes == 0);

  multi.own_cells.clear();
  assert(rebuild.begin(rebuild_region, rebuild_grid, 106));
  while (rebuild.next(rebuild_region, rebuild_grid, 106, tile, 5)) {
    assert(AcousticMasterMerge::compose_from_baseline(
        multi, rebuild_grid, tile.min_i, tile.min_j, tile.max_i,
        tile.max_j, multi_baseline.data(), multi_baseline.size(),
        multi_master.data(), multi_master.size()));
  }
  assert(multi_master == multi_baseline);
  return 0;
}
