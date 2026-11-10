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
  // A newer invalidation creates a tombstone through the original bounded
  // expiry. A delayed valid observation between the accepted sample and the
  // invalidation must not restore the source.
  AcousticCellBuffer invalidation(2, 80);
  assert(invalidation.ingest({12, 100, 300, 1., 1., true}, 110));
  assert(!invalidation.revoke_if_not_older(12, 99));
  assert(invalidation.source_count() == 1);
  assert(invalidation.revoke_if_not_older(12, 150));
  assert(invalidation.source_count() == 0);
  assert(!invalidation.ingest({12, 120, 300, 1., 1., true}, 160));
  assert(!invalidation.ingest({12, 150, 300, 1., 1., true}, 160));
  assert(invalidation.ingest({12, 161, 300, 2., 2., true}, 161));
  invalidation.revoke_all();
  assert(!invalidation.ingest({12, 150, 300, 1., 1., true}, 170));
  invalidation.tick(301, g);
  assert(invalidation.ingest({12, 301, 400, 3., 3., true}, 301));
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
  // Huge but finite metric coordinates must not produce integer casts from
  // nonfinite projected indices (UB under some compiler configurations).
  AcousticCellBuffer projection(2, 100);
  assert(projection.ingest({41, 100, 200, 1e308, 1e308, true}, 100));
  auto projected = projection.tick(100, {10, 10, 1e-300, 0., 0.});
  assert(projected.own_cells.empty());
  // Reject an enormous map geometry rather than exposing it downstream.
  assert(projection.ingest({42, 110, 200, 1., 1., true}, 110));
  projected = projection.tick(110, {100000, 100000, 1., 0., 0.});
  assert(projected.own_cells.empty() && projection.source_count() == 0);
  // Invalid geometry must clear previously owned indices on the old grid.
  AcousticCellBuffer invalid_grid(2, 100);
  assert(invalid_grid.ingest({43, 100, 200, 1., 1., true}, 100));
  assert(invalid_grid.tick(100, g).own_cells.size() == 1);
  auto cleared = invalid_grid.tick(110, {100000, 100000, 1., 0., 0.});
  assert(cleared.retired_cells.count(std::make_pair(1u, 1u)));
  // Invalid/oversized geometry must not erase replay watermarks. After
  // recovery, delayed packets from the retired epoch must remain rejected.
  AcousticCellBuffer grid_recovery(2, 90);
  assert(grid_recovery.ingest({61, 100, 300, 2., 2., true}, 110));
  assert(grid_recovery.tick(110, g).own_cells.size() == 1);
  auto lost_grid = grid_recovery.tick(120, {100000, 100000, 1., 0., 0.});
  assert(lost_grid.retired_cells.count(std::make_pair(2u, 2u)));
  assert(grid_recovery.source_count() == 0);
  assert(!grid_recovery.ingest({61, 100, 300, 2., 2., true}, 121));
  auto recovered_grid = grid_recovery.tick(121, g);
  assert(recovered_grid.geometry_changed);
  assert(recovered_grid.own_cells.empty());
  assert(grid_recovery.ingest({61, 122, 300, 3., 3., true}, 122));
  assert(grid_recovery.tick(122, g).own_cells.count(std::make_pair(3u, 3u)));
  grid_recovery.tick(301, {100000, 100000, 1., 0., 0.});
  assert(grid_recovery.ingest({62, 301, 400, 4., 4., true}, 301));
  assert(grid_recovery.tick(301, g).own_cells.count(std::make_pair(4u, 4u)));
  // Upstream TF loss / operator rollback must revoke all active costs even
  // without an Ended message. Old packets cannot resurrect revoked cells.
  AcousticCellBuffer rollback(2, 90);
  assert(rollback.ingest({51, 100, 300, 1., 2., true}, 110));
  assert(rollback.ingest({52, 101, 300, 3., 4., true}, 110));
  assert(rollback.tick(110, g).own_cells.size() == 2);
  rollback.revoke_all();
  assert(rollback.source_count() == 0);
  auto rollback_update = rollback.tick(111, g);
  assert(rollback_update.own_cells.empty());
  assert(rollback_update.retired_cells.size() == 2);
  assert(!rollback.ingest({51, 100, 300, 1., 2., true}, 112));
  assert(!rollback.ingest({52, 101, 300, 3., 4., true}, 112));
  // A genuinely newer, independently validated observation may be admitted
  // only after the external metric/TF authorization gate is restored.
  assert(rollback.ingest({51, 113, 300, 1., 2., true}, 113));
  assert(rollback.tick(113, g).own_cells.size() == 1);
  // Two independent sources may project to the same cell. Revoking or
  // expiring only one must never erase the other source's soft cost.
  AcousticCellBuffer overlap(3, 90);
  assert(overlap.ingest({71, 100, 150, 2., 2., true}, 110));
  assert(overlap.ingest({72, 101, 250, 2., 2., true}, 110));
  auto shared = overlap.tick(110, g);
  assert(shared.own_cells.size() == 1);
  // First tick establishes geometry and requests a full-grid rebuild;
  // changed_cells is intentionally reserved for same-geometry deltas.
  assert(shared.geometry_changed);
  assert(shared.changed_cells.empty());
  assert(shared.own_cells.count(std::make_pair(2u, 2u)) == 1);
  overlap.revoke(71);
  shared = overlap.tick(111, g);
  assert(shared.own_cells.size() == 1);
  assert(shared.changed_cells.empty() && shared.retired_cells.empty());
  shared = overlap.tick(151, g);
  assert(shared.own_cells.size() == 1);
  assert(shared.changed_cells.empty() && shared.retired_cells.empty());
  overlap.revoke(72);
  shared = overlap.tick(152, g);
  assert(shared.own_cells.empty());
  assert(shared.retired_cells.count(std::make_pair(2u, 2u)) == 1);
  assert(overlap.ingest({73, 153, 200, 2., 2., true}, 153));
  shared = overlap.tick(153, g);
  assert(shared.changed_cells.count(std::make_pair(2u, 2u)) == 1);
  return 0;
}
