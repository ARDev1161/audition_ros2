#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "audition_nav2/acoustic_independent_baseline_adapter.hpp"
using namespace audition_nav2;

int main() {
  const GridGeometry geometry{20, 20, 0.1, 0, 0};
  std::vector<std::uint8_t> master(400, 7);
  std::vector<std::uint8_t> independent(400, 11);
  independent[2 * 20 + 3] = 254;
  independent[2 * 20 + 4] = 255;
  LayerUpdate overlay;
  overlay.own_cells[{2, 2}] = 120;
  overlay.own_cells[{3, 2}] = 120;
  overlay.own_cells[{4, 2}] = 120;
  AcousticBaselineGate gate;
  auto ticket = [&](std::uint64_t cycle) {
    return BaselineTicket{17, cycle,
        BaselineOrigin::IndependentNonAcousticLayers, geometry};
  };
  unsigned calls = 0;
  const auto rebuild = [&](const GridGeometry&, std::uint8_t* dst,
                           std::uint8_t* coverage, std::size_t n) {
    ++calls;
    assert(n == independent.size());
    std::copy(independent.begin(), independent.end(), dst);
    std::fill(coverage, coverage + n, static_cast<std::uint8_t>(1));
    return true;
  };
  std::size_t changes = 999;
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size(), &changes));
  assert(calls == 1 && changes == 1);
  assert(master[2 * 20 + 2] == 120);
  assert(master[2 * 20 + 3] == 254);
  assert(master[2 * 20 + 4] == 255);
  // No incoming observations: reconstructing from independent layers
  // clears old audio costs without touching non-acoustic obstacles.
  overlay.own_cells.clear();
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == independent);
  // Replay is rejected, including when the callback is otherwise valid.
  const auto before = master;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(2), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == before);
  // Stale/replayed tickets must not invoke expensive reconstruction.
  assert(calls == 2);
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(1), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(calls == 2 && master == before);
  auto old_generation = ticket(100);
  old_generation.generation = 16;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, old_generation, 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(calls == 2 && master == before);
  // Invalid and inverted windows must be rejected before provider invocation.
  // This also exercises the unsigned subtraction boundary in area accounting.
  for (const auto& window : std::vector<std::vector<unsigned>>{
           {10, 0, 9, 20}, {0, 12, 20, 11},
           {0, 0, 21, 20}, {0, 0, 20, 21},
           {0, 0, 0, 20}, {0, 0, 20, 0}}) {
    assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
        gate, ticket(3), 17, geometry, overlay,
        window[0], window[1], window[2], window[3],
        rebuild, master.data(), master.size()));
    assert(calls == 2 && master == before);
  }
  // Invalid producer data anywhere on the map (including outside the
  // requested window) must not invoke the independent baseline provider.
  // These failures also must not consume the next valid cycle.
  for (const auto& invalid : std::vector<std::pair<
           std::pair<unsigned, unsigned>, std::uint8_t>>{
           {{20, 0}, 120}, {{0, 20}, 120},
           {{19, 19}, 0}, {{19, 19}, 253},
           {{19, 19}, 254}, {{19, 19}, 255}}) {
    LayerUpdate malformed;
    malformed.own_cells[invalid.first] = invalid.second;
    assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
        gate, ticket(3), 17, geometry, malformed, 0, 0, 5, 5,
        rebuild, master.data(), master.size()));
    assert(calls == 2 && master == before);
  }
  LayerUpdate valid_outside_window;
  valid_outside_window.own_cells[{19, 19}] = 120;
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(3), 17, geometry, valid_outside_window, 0, 0, 5, 5,
      rebuild, master.data(), master.size()));
  assert(calls == 3 && master == before);
  // A tiny dirty window on a large map must not trigger an unbounded
  // synchronous full-map provider rebuild in the Nav2 update thread.
  // This is independent of the 65,536-cell dirty-window admission limit.
  {
    const GridGeometry large{257, 256, 0.1, 0, 0}; // 65,792 cells
    std::vector<std::uint8_t> large_master(257u * 256u, 33);
    AcousticBaselineGate large_gate;
    std::size_t large_calls = 0;
    const auto expensive = [&](const GridGeometry&, std::uint8_t*,
                               std::uint8_t*, std::size_t) {
      ++large_calls;
      return true;
    };
    const BaselineTicket large_ticket{
        1, 1, BaselineOrigin::IndependentNonAcousticLayers, large};
    LayerUpdate empty;
    assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
        large_gate, large_ticket, 1, large, empty, 0, 0, 1, 1,
        expensive, large_master.data(), large_master.size()));
    assert(large_calls == 0);
    assert(std::all_of(large_master.begin(), large_master.end(),
                       [](std::uint8_t v) { return v == 33; }));
  }
  // Boundary admission: exactly 65,536 cells are allowed even for a tiny
  // dirty window, while larger maps are rejected above. Confirm the full
  // coverage contract and non-acoustic lethal/unknown preservation at limit.
  {
    const GridGeometry boundary{256, 256, 0.05, -1.0, 2.0};
    std::vector<std::uint8_t> boundary_master(256u * 256u, 77);
    AcousticBaselineGate boundary_gate;
    unsigned boundary_calls = 0;
    const auto boundary_provider = [&](const GridGeometry&, std::uint8_t* out,
                                       std::uint8_t* coverage, std::size_t n) {
      ++boundary_calls;
      assert(n == boundary_master.size());
      std::fill(out, out + n, static_cast<std::uint8_t>(9));
      std::fill(coverage, coverage + n, static_cast<std::uint8_t>(1));
      out[0] = 254;
      out[n - 1] = 255;
      return true;
    };
    LayerUpdate boundary_overlay;
    boundary_overlay.own_cells[{1, 0}] = 252;
    const BaselineTicket boundary_ticket{
        1, 1, BaselineOrigin::IndependentNonAcousticLayers, boundary};
    assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
        boundary_gate, boundary_ticket, 1, boundary, boundary_overlay,
        0, 0, 2, 1, boundary_provider,
        boundary_master.data(), boundary_master.size()));
    assert(boundary_calls == 1);
    assert(boundary_master[0] == 254);
    assert(boundary_master[1] == 252);
    assert(boundary_master.back() == 77);  // Outside dirty window untouched.
    // Replaying a ticket must not invoke the provider.
    assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
        boundary_gate, boundary_ticket, 1, boundary, boundary_overlay,
        0, 0, 2, 1, boundary_provider,
        boundary_master.data(), boundary_master.size()));
    assert(boundary_calls == 1);
  }
  // Invalid floating-point geometry must not enter the provider or mutate
  // the master, even if its ticket carries the same invalid geometry.
  {
    auto invalid = geometry;
    invalid.resolution = std::numeric_limits<double>::quiet_NaN();
    const BaselineTicket invalid_ticket{
        18, 1, BaselineOrigin::IndependentNonAcousticLayers, invalid};
    assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
        gate, invalid_ticket, 18, invalid, overlay, 0, 0, 20, 20,
        rebuild, master.data(), master.size()));
    assert(calls == 3 && master == before);
  }
  // Subsequent provider-failure checks use a fresh admission cycle.
  // Callback failure and exceptions never mutate the master.
  const auto failed = [](const GridGeometry&, std::uint8_t*,
                         std::uint8_t*, std::size_t) {
    return false;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      failed, master.data(), master.size()));
  const auto throwing = [](const GridGeometry&, std::uint8_t*,
                           std::uint8_t*, std::size_t) -> bool {
    throw std::runtime_error("provider unavailable");
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      throwing, master.data(), master.size()));
  assert(master == before);
  // A provider may partially populate its scratch buffer before failing.
  // Neither that data nor a thrown exception may leak into the master.
  unsigned partial_calls = 0;
  const auto partial_failure = [&](const GridGeometry&, std::uint8_t* dst,
                                   std::uint8_t* coverage, std::size_t n) {
    ++partial_calls;
    assert(n == independent.size());
    std::fill(dst, dst + n, static_cast<std::uint8_t>(252));
    std::fill(coverage, coverage + n / 2, static_cast<std::uint8_t>(1));
    return false;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      partial_failure, master.data(), master.size()));
  assert(partial_calls == 1 && master == before);
  // Even a provider that returns success must prove that every cell was
  // reconstructed. Unwritten zero-initialized cells must not clear obstacles.
  unsigned incomplete_calls = 0;
  const auto incomplete_success = [&](const GridGeometry&, std::uint8_t* dst,
                                      std::uint8_t* coverage, std::size_t n) {
    ++incomplete_calls;
    std::copy(independent.begin(), independent.end(), dst);
    std::fill(coverage, coverage + n - 1, static_cast<std::uint8_t>(1));
    return true;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      incomplete_success, master.data(), master.size()));
  assert(incomplete_calls == 1 && master == before);
  const auto invalid_coverage = [&](const GridGeometry&, std::uint8_t* dst,
                                    std::uint8_t* coverage, std::size_t n) {
    std::copy(independent.begin(), independent.end(), dst);
    std::fill(coverage, coverage + n, static_cast<std::uint8_t>(1));
    coverage[n - 1] = 2;
    return true;
  };
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      invalid_coverage, master.data(), master.size()));
  assert(master == before);
  // Failed provider does not consume cycle 4; fresh independent data can retry.
  // Restore the acoustic overlay explicitly: it was cleared for the TTL test
  // above, and an empty overlay must never synthesize a soft cost.
  overlay.own_cells[{2, 2}] = 120;
  overlay.own_cells[{3, 2}] = 120;
  overlay.own_cells[{4, 2}] = 120;
  assert(AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(4), 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  const auto after_reapply = master;
  assert(calls == 4);
  assert(master[2 * 20 + 2] == 120);
  assert(master[2 * 20 + 3] == 254);
  assert(master[2 * 20 + 4] == 255);
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, ticket(5), 17, {20, 20, 0.1, 1, 0}, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == after_reapply);
  // A ticket claiming the previous master is never admitted.
  auto bad = ticket(5);
  bad.origin = BaselineOrigin::PreviousMaster;
  assert(!AcousticIndependentBaselineAdapter::rebuild_and_apply(
      gate, bad, 17, geometry, overlay, 0, 0, 20, 20,
      rebuild, master.data(), master.size()));
  assert(master == after_reapply);
  return 0;
}
