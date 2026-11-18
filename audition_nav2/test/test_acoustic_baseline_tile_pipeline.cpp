#include <cassert>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include "audition_nav2/acoustic_baseline_tile_pipeline.hpp"
using namespace audition_nav2;
int main() {
  const GridGeometry geometry{513, 257, 0.05, 0, 0};
  const BaselineTicket ticket{4, 9, BaselineOrigin::IndependentNonAcousticLayers, geometry};
  AcousticBaselineTilePipeline pipeline;
  AcousticBaselineTilePipeline::Tile tile{};
  std::vector<std::uint8_t> result{42};
  assert(pipeline.begin(ticket, 256, 256));
  assert(!pipeline.next(ticket, 100, tile));
  assert(!pipeline.take_ready(ticket, result));
  assert(result == std::vector<std::uint8_t>{42});
  unsigned submitted = 0;
  while (pipeline.next(ticket, 65536, tile)) {
    const auto n = static_cast<std::size_t>(tile.width) * tile.height;
    std::vector<std::uint8_t> costs(n, static_cast<std::uint8_t>(10 + submitted));
    std::vector<std::uint8_t> coverage(n, 1);
    assert(pipeline.submit(ticket, tile, costs.data(), coverage.data(), n));
    ++submitted;
  }
  assert(submitted == 6);
  assert(pipeline.take_ready(ticket, result));
  assert(result.size() == 513u * 257u);
  assert(result.front() == 10);
  assert(result[256] == 11);
  assert(result[512] == 12);
  assert(result[256u * 513u] == 13);
  assert(result.back() == 15);
  assert(!pipeline.take_ready(ticket, result));
  // A stale completion request must revoke staged work, not merely return
  // false and permit a later completion with the old ticket.
  assert(pipeline.begin(ticket, 256, 256));
  assert(pipeline.next(ticket, 65536, tile));
  std::vector<std::uint8_t> first_costs(65536, 25), first_coverage(65536, 1);
  assert(pipeline.submit(ticket, tile, first_costs.data(), first_coverage.data(), 65536));
  auto newer = ticket;
  ++newer.generation;
  const auto unchanged = result;
  assert(!pipeline.take_ready(newer, result));
  assert(result == unchanged);
  assert(!pipeline.next(ticket, 65536, tile));
  assert(!pipeline.take_ready(ticket, result));
  // The same invalidation applies to a shifted rolling window.
  assert(pipeline.begin(ticket, 256, 256));
  auto rolled = ticket;
  rolled.geometry.origin_y += 0.05;
  assert(!pipeline.take_ready(rolled, result));
  assert(!pipeline.next(ticket, 65536, tile));
  // A malformed tile revokes the entire transaction, never exposes partial data.
  assert(pipeline.begin(ticket, 256, 256));
  assert(pipeline.next(ticket, 65536, tile));
  std::vector<std::uint8_t> costs(65536, 30), coverage(65536, 1);
  coverage[5] = 0;
  assert(!pipeline.submit(ticket, tile, costs.data(), coverage.data(), costs.size()));
  assert(!pipeline.next(ticket, 65536, tile));
  assert(!pipeline.take_ready(ticket, result));
  // Rolling map / stale asynchronous work invalidates the pending transaction.
  assert(pipeline.begin(ticket, 256, 256));
  assert(pipeline.next(ticket, 65536, tile));
  auto shifted = ticket;
  shifted.geometry.origin_x += 0.05;
  assert(!pipeline.submit(shifted, tile, costs.data(), coverage.data(), costs.size()));
  assert(!pipeline.next(ticket, 65536, tile));
  assert(pipeline.begin(ticket, 256, 256));
  assert(pipeline.next(ticket, 65536, tile));
  assert(!pipeline.submit(ticket, tile, costs.data(), coverage.data(), 1));
  assert(!pipeline.take_ready(ticket, result));
  // Reordered tile cannot advance the cursor.
  assert(pipeline.begin(ticket, 256, 256));
  assert(pipeline.next(ticket, 65536, tile));
  coverage[5] = 1;
  auto wrong = tile;
  wrong.x = 256;
  assert(!pipeline.submit(ticket, wrong, costs.data(), coverage.data(), costs.size()));
  assert(!pipeline.next(ticket, 65536, tile));
  // Bounded producer: one tile per tick, no partial output.
  const GridGeometry small{5, 2, 0.1, 0, 0};
  const BaselineTicket fresh{8, 11, BaselineOrigin::IndependentNonAcousticLayers, small};
  auto producer = [](const BaselineTicket& seen, const AcousticBaselineTilePipeline::Tile& t,
                     std::vector<std::uint8_t>& c, std::vector<std::uint8_t>& cover) {
    assert(seen.generation == 8);
    assert(t.width * t.height <= 2);
    for (auto& v : c) v = 30;
    for (auto& v : cover) v = 1;
    return true;
  };
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 1, producer)); // Budget too small: no callback.
  for (int i = 0; i < 4; ++i) {
    assert(pipeline.produce_one(fresh, 2, producer));
    assert(!pipeline.take_ready(fresh, result));
  }
  assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));
  assert(result == std::vector<std::uint8_t>(10, 30));

  // Callback failure, exception, and malformed output all revoke the ticket.
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [](const auto&, const auto&, auto&, auto&) {
    return false;
  }));
  assert(!pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [](const auto&, const auto&, auto&, auto&) -> bool {
    throw std::runtime_error("producer failed");
  }));
  assert(!pipeline.take_ready(fresh, result));
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [](const auto&, const auto&, auto& c, auto&) {
    c.clear();
    return true;
  }));
  assert(!pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.begin(fresh, 2, 1));
  assert(pipeline.produce_one(fresh, 2, producer));
  pipeline.abort();
  assert(!pipeline.take_ready(fresh, result));

  // A callback can synchronously cancel or restart the transaction. Its
  // completion must never be submitted into the new transaction (same-ticket ABA).
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [&](const auto&, const auto&, auto& c, auto& cover) {
    pipeline.abort();
    for (auto& v : c) v = 99;
    for (auto& v : cover) v = 1;
    return true;
  }));
  assert(!pipeline.take_ready(fresh, result));
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [&](const auto&, const auto&, auto& c, auto& cover) {
    assert(pipeline.begin(fresh, 2, 1));
    for (auto& v : c) v = 99;
    for (auto& v : cover) v = 1;
    return true;
  }));
  // The replacement transaction is still intact and can be completed normally.
  for (int i = 0; i < 6; ++i) assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));
  assert(result == std::vector<std::uint8_t>(10, 30));
  // An exception after a callback restarts the pipeline must not abort the
  // replacement transaction either.
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [&](const auto&, const auto&, auto&, auto&) -> bool {
    assert(pipeline.begin(fresh, 2, 1));
    throw std::runtime_error("stale callback");
  }));
  for (int i = 0; i < 6; ++i) assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));

  // Same-ticket ABA for manually submitted asynchronous tiles: a delayed
  // completion from a cancelled transaction must never poison the new one.
  assert(pipeline.begin(fresh, 2, 1));
  AcousticBaselineTilePipeline::Tile stale{};
  assert(pipeline.next(fresh, 2, stale));
  pipeline.abort();
  assert(pipeline.begin(fresh, 2, 1));
  AcousticBaselineTilePipeline::Tile live{};
  assert(pipeline.next(fresh, 2, live));
  assert(stale.x == live.x && stale.y == live.y);
  assert(stale.lease != live.lease);
  std::vector<std::uint8_t> stale_costs(2, 99), valid_costs(2, 30), valid_cover(2, 1);
  assert(!pipeline.submit(fresh, stale, stale_costs.data(), valid_cover.data(), 2));
  // Fail closed after stale submission; no partially staged data may escape.
  assert(!pipeline.take_ready(fresh, result));
  assert(pipeline.begin(fresh, 2, 1));
  assert(pipeline.next(fresh, 2, live));
  assert(pipeline.submit(fresh, live, valid_costs.data(), valid_cover.data(), 2));
  for (int i = 0; i < 5; ++i) assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));
  assert(result == std::vector<std::uint8_t>(10, 30));

  // Stalled async tiles must expire without requiring another producer
  // completion or ROS message. Use one monotonic clock domain throughout.
  assert(pipeline.begin(fresh, 2, 1));
  assert(pipeline.active());
  assert(!pipeline.expire_if_stalled()); // Immediate wall-timer tick must preserve work.
  assert(pipeline.next(fresh, 2, live));
  assert(!pipeline.expire_if_stalled(fresh, 1000, 1099, 100));
  assert(pipeline.expire_if_stalled(fresh, 1000, 1100, 100));
  assert(!pipeline.active());
  assert(!pipeline.expire_if_stalled());
  assert(!pipeline.submit(fresh, live, valid_costs.data(), valid_cover.data(), 2));
  assert(!pipeline.take_ready(fresh, result));
  // Clock rollback and zero deadline are invalid, not unlimited retention.
  assert(pipeline.begin(fresh, 2, 1));
  assert(pipeline.expire_if_stalled(fresh, 1000, 999, 100));
  assert(pipeline.begin(fresh, 2, 1));
  assert(pipeline.expire_if_stalled(fresh, 1000, 1000, 0));
  // A stale ticket also revokes partial reconstruction immediately.
  assert(pipeline.begin(fresh, 2, 1));
  auto stale_ticket = fresh;
  ++stale_ticket.cycle;
  assert(pipeline.expire_if_stalled(stale_ticket, 1000, 1001, 100));
  assert(!pipeline.produce_one(fresh, 2, producer));
  // A producer that finishes after its transaction was timed out must not
  // publish stale data, even when the periodic timer has not run again.
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.produce_one(fresh, 2, [&](const auto&, const auto&, auto& c, auto& cover) {
    for (auto& v : c) v = 99;
    for (auto& v : cover) v = 1;
    assert(pipeline.expire_if_stalled(fresh, 1000, 1100, 100));
    return true;
  }));
  assert(!pipeline.active());
  assert(!pipeline.take_ready(fresh, result));
  // A fresh transaction still works after a timed-out callback.
  assert(pipeline.begin(fresh, 2, 1));
  for (int i = 0; i < 6; ++i) assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));
  assert(result == std::vector<std::uint8_t>(10, 30));

  // Healthy reconstruction stays usable before the deadline.
  assert(pipeline.begin(fresh, 2, 1));
  assert(!pipeline.expire_if_stalled(fresh, 1000, 1099, 100));
  for (int i = 0; i < 6; ++i) assert(pipeline.produce_one(fresh, 2, producer));
  assert(pipeline.take_ready(fresh, result));
  assert(result == std::vector<std::uint8_t>(10, 30));
}
