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
  assert(session.active() && session.cursor() == 0);
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
  assert(session.begin(small, stable, 41));
  assert(session.next(small, stable, 41, tile, 4));
  assert(session.cursor() == 4);
  assert(!session.next(small, stable, 42, tile, 4));
  assert(!session.active() && session.cursor() == 0 && tile.empty());
  assert(session.begin(small, stable, 43));
  assert(!session.next(small, stable, tile, 4));
  assert(!session.active() && tile.empty());
  assert(session.begin(small, stable, 44));
  std::uint64_t epoch_visited = 0;
  while (session.next(small, stable, 44, tile, 3))
    epoch_visited += tile.max_i - tile.min_i;
  assert(epoch_visited == 20);
  assert(!session.active());
  assert(session.begin(small, stable, 45));
  assert(session.next(small, stable, 45, tile, 2));
  assert(!session.next(small, stable, 46, tile, 2));
  assert(session.begin(small, stable, 46));
  assert(session.next(small, stable, 46, tile, 2));
  assert(tile.min_i == small.min_i);
  return 0;
}
