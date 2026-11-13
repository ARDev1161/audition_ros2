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
    if (c.size() > 2) c[2] = 254; else c[0] = 254;
    std::fill(m.begin(), m.end(), 1);
    return true;
  };
  const auto inflation = [](const auto&, const auto&, auto& c, auto& m) {
    if (c.size() > 1) c[1] = 100; if (c.size() > 2) c[2] = 80;
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
  // Nav2 NO_INFORMATION (255) cannot be combined by numeric max: a
  // provider must resolve unknowns before claiming complete coverage.
  auto unknown = [](const auto&, const auto&, auto& c, auto& m) {
    std::fill(c.begin(), c.end(), 0);
    std::fill(m.begin(), m.end(), 1);
    c[2] = 255;
    return true;
  };
  assert(!C::compose(t, tile, {obstacle, unknown}, costs, marks));
  assert(costs == original && marks == original_marks);
  // Reject malformed coverage values, not just missing cells.
  auto invalid_coverage = [](const auto&, const auto&, auto&, auto& m) {
    std::fill(m.begin(), m.end(), 1);
    m[0] = 2;
    return true;
  };
  assert(!C::compose(t, tile, {invalid_coverage}, costs, marks));
  assert(costs == original && marks == original_marks);
  // A verified provider must enumerate unique, independent non-acoustic
  // identities and certify max-style combination before any callbacks run.
  using V = C::VerifiedLayer;
  std::vector<V> verified{{"obstacles", true, true, obstacle},
                          {"inflation", true, true, inflation}};
  assert(C::compose_verified(t, tile, verified, "acoustic", costs, marks));
  assert(costs[2] == 254 && costs[1] == 100);
  const auto verified_costs = costs, verified_marks = marks;
  int invoked = 0;
  auto counted = [&](const auto&, const auto&, auto& c, auto& m) {
    ++invoked;
    std::fill(c.begin(), c.end(), 1);
    std::fill(m.begin(), m.end(), 1);
    return true;
  };
  const auto rejected = [&](std::vector<V> sources, const std::string& self) {
    assert(!C::compose_verified(t, tile, sources, self, costs, marks));
    assert(costs == verified_costs && marks == verified_marks);
    assert(invoked == 0);
  };
  rejected({{"acoustic", true, true, counted}}, "acoustic");
  rejected({{"", true, true, counted}}, "acoustic");
  rejected({{"obstacles", false, true, counted}}, "acoustic");
  rejected({{"obstacles", true, false, counted}}, "acoustic");
  rejected({{"obstacles", true, true, counted},
            {"obstacles", true, true, counted}}, "acoustic");
  rejected({{"obstacles", true, true, counted}}, "");
  rejected({}, "acoustic");
  // Invalid provenance must be rejected before invoking even the first
  // producer. An accepted callback failure still rolls back atomically.
  assert(!C::compose_verified(t, tile,
      {{"obstacles", true, true, counted}, {"bad", true, true, throwing}},
      "acoustic", costs, marks));
  assert(invoked == 1 && costs == verified_costs && marks == verified_marks);
  invoked = 0;
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
