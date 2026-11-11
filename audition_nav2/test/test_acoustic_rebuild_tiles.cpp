#include <cassert>
#include <cstdint>
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
  cursor = 3;
  const auto check_reject = [&](CellBounds region, GridGeometry geometry,
                                std::uint64_t budget) {
    CellBounds out{1, 1, 2, 2};
    assert(!AcousticRebuildTiles::next(region, geometry, cursor, out, budget));
    assert(cursor == 3);
    assert(out.empty());
  };
  check_reject({0, 0, 2001, 1}, large, 10);
  check_reject({0, 0, 1, 2001}, large, 10);
  check_reject({0, 0, 1, 1}, large, 0);
  check_reject({0, 0, 1, 1}, large,
               AcousticMasterMerge::kMaxRebuildCells + 1);
  check_reject({0, 0, 1, 1}, {2001, 2000, 0.1, 0, 0}, 10);
  check_reject({0, 0, 0, 1}, large, 10);
  return 0;
}
