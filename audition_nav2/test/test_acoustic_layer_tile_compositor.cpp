#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "audition_nav2/acoustic_layer_tile_compositor.hpp"
using namespace audition_nav2;
int main() {
  const GridGeometry g{4, 2, 0.1, 0, 0};
  const BaselineTicket t{2, 3, BaselineOrigin::IndependentNonAcousticLayers, g};
  const AcousticLayerTileCompositor::Tile tile{0, 0, 4, 2, 7};
  std::vector<std::uint8_t> costs(8, 99), marks(8, 0);
  const auto obstacle = [](const auto&, const auto&, auto& c, auto& m) {
    c[2] = 254;
    std::fill(m.begin(), m.end(), 1);
    return true;
  };
  const auto inflation = [](const auto&, const auto&, auto& c, auto& m) {
    c[1] = 100; c[2] = 80;
    std::fill(m.begin(), m.end(), 1);
    return true;
  };
  using C = AcousticLayerTileCompositor;
  assert(C::compose(t, tile, {obstacle, inflation}, costs, marks));
  assert(costs[1] == 100 && costs[2] == 254 && costs[0] == 0);
  assert(marks == std::vector<std::uint8_t>(8, 1));
  const auto original = costs;
  const auto original_marks = marks;
  auto incomplete = [](const auto&, const auto&, auto&, auto& m) {
    std::fill(m.begin(), m.end(), 1); m[3] = 0; return true;
  };
  assert(!C::compose(t, tile, {obstacle, incomplete}, costs, marks));
  assert(costs == original && marks == original_marks);
  auto bad_size = [](const auto&, const auto&, auto& c, auto& m) {
    c.clear(); std::fill(m.begin(), m.end(), 1); return true;
  };
  assert(!C::compose(t, tile, {bad_size}, costs, marks));
  assert(costs == original && marks == original_marks);
  auto throwing = [](const auto&, const auto&, auto&, auto&) -> bool {
    throw std::runtime_error("layer failed");
  };
  assert(!C::compose(t, tile, {throwing}, costs, marks));
  assert(costs == original && marks == original_marks);
  auto bad_ticket = t;
  bad_ticket.origin = BaselineOrigin::PreviousMaster;
  assert(!C::compose(bad_ticket, tile, {obstacle}, costs, marks));
  auto bad_tile = tile; bad_tile.x = 3;
  assert(!C::compose(t, bad_tile, {obstacle}, costs, marks));
  assert(!C::compose(t, tile, {}, costs, marks));
  // Integrate the compositor with bounded pipeline without Nav2 writes.
  AcousticBaselineTilePipeline pipeline;
  assert(pipeline.begin(t, 2, 1));
  const auto producer = [&](const BaselineTicket& ticket,
                            const AcousticBaselineTilePipeline::Tile& next,
                            auto& c, auto& m) {
    return C::compose(ticket, next, {obstacle, inflation}, c, m);
  };
  for (int i = 0; i < 4; ++i) assert(pipeline.produce_one(t, 2, producer));
  std::vector<std::uint8_t> result;
  assert(pipeline.take_ready(t, result));
  assert(result.size() == 8 && result[1] == 100 && result[2] == 254);
  assert(!pipeline.take_ready(t, result));
}
