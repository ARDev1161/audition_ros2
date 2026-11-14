#include <cassert>
#include <cstdint>
#include "audition_nav2/acoustic_baseline_tile_cursor.hpp"
using namespace audition_nav2;
int main() {
  const GridGeometry g{513, 257, 0.05, 0, 0};
  const BaselineTicket t{2, 7, BaselineOrigin::IndependentNonAcousticLayers, g};
  AcousticBaselineTileCursor cursor;
  AcousticBaselineTileCursor::Tile tile{};
  assert(!cursor.begin(t, 257, 256)); // tile exceeds 65536
  assert(cursor.begin(t, 256, 256));
  assert(!cursor.next(t, 1, tile)); // cannot exceed caller budget
  assert(!cursor.next(t, 65537, tile)); // budget itself is bounded
  assert(cursor.next(t, 65536, tile));
  assert(tile.x == 0 && tile.y == 0 && tile.width == 256 && tile.height == 256);
  assert(!cursor.next(t, 65536, tile)); // pending work cannot duplicate
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.next(t, 65536, tile));
  assert(tile.x == 256 && tile.y == 0 && tile.width == 256);
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.next(t, 65536, tile));
  assert(tile.x == 512 && tile.y == 0 && tile.width == 1);
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.next(t, 65536, tile));
  assert(tile.x == 0 && tile.y == 256 && tile.height == 1);
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.next(t, 65536, tile));
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.next(t, 65536, tile));
  assert(cursor.acknowledge(t, tile, true));
  assert(cursor.finished(t));
  assert(!cursor.next(t, 65536, tile));
  auto stale = t; stale.cycle++;
  assert(!cursor.finished(stale));
  assert(cursor.begin(t, 256, 256));
  assert(cursor.next(t, 65536, tile));
  assert(!cursor.acknowledge(stale, tile, true));
  assert(!cursor.matches(t)); // stale producer revokes transaction
  assert(cursor.begin(t, 256, 256));
  assert(cursor.next(t, 65536, tile));
  assert(!cursor.acknowledge(t, tile, false));
  assert(!cursor.matches(t));
  assert(cursor.begin(t, 256, 256));
  assert(cursor.next(t, 65536, tile));
  auto corrupted = tile; corrupted.x++;
  assert(!cursor.acknowledge(t, corrupted, true));
  assert(!cursor.matches(t));
  auto shifted = t; shifted.geometry.origin_x += 0.05;
  assert(cursor.begin(t, 256, 256));
  assert(!cursor.next(shifted, 65536, tile));
  assert(!cursor.finished(shifted));
  assert(!cursor.begin(t, 0, 256));
  assert(!cursor.begin(t, 256, 0));
  assert(!cursor.begin(t, 65536, 65536));
  auto bad = t; bad.origin = BaselineOrigin::PreviousMaster;
  assert(!cursor.begin(bad, 256, 256));
}
