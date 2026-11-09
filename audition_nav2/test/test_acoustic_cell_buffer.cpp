#include "audition_nav2/acoustic_cell_buffer.hpp"
#include <cassert>
using namespace audition_nav2;
int main() {
  AcousticCellBuffer b(2, 100);
  const GridGeometry g{10, 10, 1., 0., 0.};
  const MetricCell a{1, 90, 200, 2., 3., true};
  const MetricCell other{2, 91, 200, 4., 5., true};
  assert(!b.ingest({3, 90, 200, 1., 1., false}, 100));
  assert(b.ingest(a, 100));
  assert(!b.ingest(a, 100)); // replay
  assert(b.ingest(other, 100));
  assert(!b.ingest({3, 90, 200, 6., 6., true}, 100)); // capacity
  auto r = b.tick(100, g);
  assert(r.own_cells.size() == 2 && r.own_cells.at({2,3}) == 100);
  b.revoke(1);
  r = b.tick(101, g);
  assert(r.own_cells.size() == 1 && r.retired_cells.count({2,3}));
  r = b.tick(102, {10,10,1.,1.,0.}); // rolling origin
  assert(r.geometry_changed && r.retired_cells.empty());
  assert(r.own_cells.count({3,5}));
  r = b.tick(201, {10,10,1.,1.,0.}); // TTL without new messages
  assert(r.own_cells.empty() && r.retired_cells.count({3,5}));
  assert(b.ingest({4, 205, 300, 2., 3., true}, 210));
  r = b.tick(210, g);
  assert(r.own_cells.count({2,3}));
  r = b.tick(100, g); // rewind clears old epoch
  assert(r.own_cells.empty() && r.retired_cells.count({2,3}));
  assert(b.source_count() == 0);
  assert(!b.ingest({5, 100, 150, 0., 0., true}, 0));
  return 0;
}
