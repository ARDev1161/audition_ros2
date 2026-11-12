#include <cassert>
#include <cstdint>
#include <vector>
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
}
