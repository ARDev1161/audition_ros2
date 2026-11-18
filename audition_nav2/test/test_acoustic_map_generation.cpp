#include "audition_nav2/acoustic_map_generation.hpp"
#include "audition_nav2/acoustic_rebuild_tiles.hpp"
#include <cassert>
#include <limits>
using namespace audition_nav2;

int main() {
  AcousticMapGeneration authority;
  GridGeometry grid{12, 9, 0.1, -1.0, -2.0};
  CellBounds dirty{0, 0, 12, 9};
  assert(authority.generation() == 0);
  const auto first = authority.observe(grid);
  assert(first != 0);
  assert(authority.observe(grid) == first);
  AcousticRebuildSession session;
  assert(session.begin(dirty, grid, first));
  CellBounds tile;
  assert(session.next(dirty, grid, first, tile, 7));
  assert(session.cursor() == 7);

  // Rolling shift invalidates a previously started rebuild.
  grid.origin_x += 0.1;
  const auto shifted = authority.observe(grid);
  assert(shifted != 0 && shifted != first);
  assert(!session.next(dirty, grid, shifted, tile, 7));
  assert(!session.active() && session.cursor() == 0);
  assert(session.begin(dirty, grid, shifted));

  // ABA: a map moves back to the old geometry but not the old generation.
  grid.origin_x -= 0.1;
  const auto restored = authority.observe(grid);
  assert(restored != first && restored != shifted);
  assert(!session.next(dirty, grid, restored, tile, 7));

  // A same-geometry baseline replacement must explicitly revoke tokens.
  assert(session.begin(dirty, grid, restored));
  authority.invalidate();
  assert(authority.generation() == 0);
  const auto replacement = authority.observe(grid);
  assert(replacement != restored);
  assert(!session.next(dirty, grid, replacement, tile, 7));

  // Resize and resolution changes cannot reuse the previous generation.
  const auto before_resize = replacement;
  grid.width = 13;
  dirty.max_i = 13;
  assert(authority.observe(grid) != before_resize);
  const auto before_resolution = authority.generation();
  grid.resolution *= 2;
  assert(authority.observe(grid) != before_resolution);

  // Invalid geometry revokes the authority, never authorizes a cursor.
  grid.resolution = std::numeric_limits<double>::quiet_NaN();
  assert(authority.observe(grid) == 0);
  assert(authority.generation() == 0);
  grid.resolution = 0.1;
  const auto recovered = authority.observe(grid);
  assert(recovered != 0 && recovered != before_resolution);
  grid.width = 100001;
  assert(authority.observe(grid) == 0);
  grid.width = 13;
  assert(authority.observe(grid) != 0);
  return 0;
}
