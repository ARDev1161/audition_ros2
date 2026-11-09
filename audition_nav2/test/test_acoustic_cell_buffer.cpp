#include "audition_nav2/acoustic_cell_buffer.hpp"
#include <cassert>
#include <limits>
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
  assert(r.own_cells.size() == 2 && r.own_cells.at(std::make_pair(2u,3u)) == 100);
  b.revoke(1);
  r = b.tick(101, g);
  assert(r.own_cells.size() == 1 && r.retired_cells.count(std::make_pair(2u,3u)));
  r = b.tick(102, {10,10,1.,1.,0.}); // rolling origin
  assert(r.geometry_changed && r.retired_cells.empty());
  assert(r.own_cells.count(std::make_pair(3u,5u)));
  r = b.tick(201, {10,10,1.,1.,0.}); // TTL without new messages
  assert(r.own_cells.empty() && r.retired_cells.count(std::make_pair(3u,5u)));
  assert(b.ingest({4, 205, 300, 2., 3., true}, 210));
  r = b.tick(210, g);
  assert(r.own_cells.count(std::make_pair(2u,3u)));
  r = b.tick(100, g); // rewind clears old epoch
  assert(r.own_cells.empty() && r.retired_cells.count(std::make_pair(2u,3u)));
  assert(b.source_count() == 0);
  assert(!b.ingest({5, 100, 150, 0., 0., true}, 0));
  // Reordered observations cannot resurrect a revoked source before TTL.
  AcousticCellBuffer replay(2, 80);
  assert(replay.ingest({11, 100, 300, 1., 1., true}, 110));
  replay.revoke(11);
  assert(!replay.ingest({11, 99, 301, 1., 1., true}, 111));
  assert(!replay.ingest({11, 100, 301, 1., 1., true}, 111));
  assert(replay.ingest({11, 111, 310, 2., 2., true}, 111));
  replay.tick(311, g);
  assert(replay.source_count() == 0);
  // Replay history is bounded even when distinct IDs are revoked repeatedly.
  AcousticCellBuffer bounded(1, 80);
  for (std::uint64_t id = 1; id <= 4; ++id) {
    assert(bounded.ingest({id, 100, 300, 1., 1., true}, 110));
    bounded.revoke(id);
  }
  assert(!bounded.ingest({5, 100, 300, 1., 1., true}, 110));
  bounded.tick(301, g);
  assert(bounded.ingest({5, 301, 400, 1., 1., true}, 301));
  // A malicious producer cannot pin soft costs indefinitely or replay ancient
  // observations. Bounded TTL/age are enforced independently of timer cadence.
  AcousticCellBuffer horizons(2, 100, 1000, 100);
  assert(!horizons.ingest({21, 100, 1101, 1., 1., true}, 110)); // TTL too long
  assert(!horizons.ingest({21, 100, 300, 1., 1., true}, 201)); // too old
  assert(horizons.ingest({21, 110, 210, 1., 1., true}, 110));
  auto h = horizons.tick(110, g);
  assert(h.own_cells.count(std::make_pair(1u,1u)));
  h = horizons.tick(210, g);
  assert(h.own_cells.empty() && h.retired_cells.count(std::make_pair(1u,1u)));
  assert(!horizons.ingest({21, 110, 210, 1., 1., true}, 211));
  try { AcousticCellBuffer invalid(2, 100, 0, 100); (void)invalid; assert(false); }
  catch (const std::invalid_argument&) {}
  // Exercise extreme but representable signed timestamps under UBSan.
  // The TTL difference must be computed without signed overflow.
  AcousticCellBuffer extremes(2, 100, 1000, 100);
  const auto max_time = std::numeric_limits<std::int64_t>::max();
  assert(!extremes.ingest({31, 1, max_time, 1., 1., true}, max_time - 1));
  assert(!extremes.ingest({31, 1, max_time, 1., 1., true}, max_time));
  assert(extremes.ingest({31, max_time - 50, max_time, 1., 1., true}, max_time - 25));
  auto extreme_cells = extremes.tick(max_time - 25, g);
  assert(extreme_cells.own_cells.size() == 1);
  extreme_cells = extremes.tick(max_time, g);
  assert(extreme_cells.own_cells.empty());
  return 0;
}
