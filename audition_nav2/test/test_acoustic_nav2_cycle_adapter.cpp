#include "audition_nav2/acoustic_nav2_cycle_adapter.hpp"
#include "audition_nav2/acoustic_world_bounds.hpp"
#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

using namespace audition_nav2;

static std::size_t index(unsigned x, unsigned y, unsigned w) {
  return static_cast<std::size_t>(y) * w + x;
}

int main() {
  // The Nav2 plugin must be unique and last, after at least one
  // non-acoustic plugin. Filters run separately after all ordinary plugins.
  auto static_layer = std::make_shared<int>(1);
  auto obstacle_layer = std::make_shared<int>(2);
  auto acoustic = std::make_shared<int>(3);
  auto filter = std::make_shared<int>(4);
  const std::vector<std::shared_ptr<int>> empty;
  std::vector<std::shared_ptr<int>> plugins{static_layer, obstacle_layer, acoustic};
  std::vector<std::shared_ptr<int>> filters;
  const auto valid = [&]() {
    return AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
        &plugins, &filters, acoustic.get());
  };
  assert(valid());
  filters.push_back(filter);
  assert(valid());
  filters.push_back(acoustic);
  assert(!valid());  // The acoustic layer cannot double as a filter.
  filters.pop_back();
  plugins.push_back(obstacle_layer);
  assert(!valid());  // Not last.
  plugins.pop_back();
  plugins.insert(plugins.begin(), acoustic);
  assert(!valid());  // Duplicate acoustic plugin.
  plugins.erase(plugins.begin());
  plugins[0] = nullptr;
  assert(!valid());
  plugins[0] = static_layer;
  filters.clear();
  assert(valid());
  plugins = {acoustic};
  assert(!valid());  // No non-acoustic baseline.
  plugins = {obstacle_layer, acoustic};
  assert(valid());
  assert(!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
      &plugins, &filters, static_layer.get()));
  assert(!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline<int>(
      nullptr, &filters, acoustic.get()));

  const GridGeometry map{7, 5, 1., 0., 0.};
  AcousticLayerCoordinator coordinator(8, 100);
  coordinator.enable();
  constexpr std::int64_t t0 = 1000000000LL;
  assert(coordinator.ingest_validated(
      {1, t0, t0 + 400000000LL, 1.5, 1.5, true}, t0 + 10000000LL));
  assert(coordinator.ingest_validated(
      {2, t0, t0 + 400000000LL, 2.5, 1.5, true}, t0 + 10000000LL));
  const auto initial = coordinator.tick(t0 + 10000000LL, map);
  assert(initial.rebuild_required);
  assert(initial.update.own_cells.size() == 2);
  std::vector<std::uint8_t> master(map.width * map.height, 0);
  master[index(0, 0, map.width)] = 254;
  master[index(3, 1, map.width)] = 255;
  master[index(4, 1, map.width)] = 175;
  auto apply = [&](const AcousticCycle& cycle, bool permitted,
                   int i0 = 0, int j0 = 0, int i1 = 7, int j1 = 5) {
    return AcousticNav2CycleAdapter::overlay_after_nav2_reset(
        cycle, map, i0, j0, i1, j1, master.data(), master.size(), permitted);
  };
  assert(apply(initial, false) == 0);
  assert(master[index(1, 1, map.width)] == 0);
  assert(apply(initial, true) == 2);
  assert(master[index(1, 1, map.width)] == 100);
  assert(master[index(2, 1, map.width)] == 100);
  assert(master[index(0, 0, map.width)] == 254);
  assert(master[index(3, 1, map.width)] == 255);
  assert(master[index(4, 1, map.width)] == 175);
  // Repeated overlay with no changed cells still restores active sources
  // after Nav2 reset caused by a different plugin.
  master[index(2, 1, map.width)] = 0;  // Simulated Nav2 rebuild window.
  const auto unchanged = coordinator.tick(t0 + 20000000LL, map);
  assert(!unchanged.rebuild_required);
  assert(apply(unchanged, true, 2, 1, 3, 2) == 1);
  assert(master[index(2, 1, map.width)] == 100);

  // No in-place FREE_SPACE writes: Nav2 does the clearing by resetting its
  // dirty window and replaying the *real* non-acoustic layers first.
  coordinator.revoke(1);
  const auto retired = coordinator.tick(t0 + 30000000LL, map);
  assert(retired.rebuild_required);
  assert(retired.update.retired_cells.count({1u, 1u}) == 1);
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  assert(AcousticWorldBounds::expand(
      AcousticWorldBounds::from_cells(retired.dirty, map),
      &min_x, &min_y, &max_x, &max_y));
  assert(min_x < 1. && min_y < 1. && max_x > 2. && max_y > 2.);
  master[index(1, 1, map.width)] = 175;  // Restored other-layer cost.
  assert(apply(retired, true, 1, 1, 2, 2) == 0);
  assert(master[index(1, 1, map.width)] == 175);
  assert(master[index(2, 1, map.width)] == 100);
  // Malformed update bounds must never modify any master cell.
  const auto before_invalid = master;
  assert(apply(retired, true, -1, 0, 7, 5) == 0);
  assert(apply(retired, true, 0, 0, 8, 5) == 0);
  assert(apply(retired, true, 2, 1, 2, 3) == 0);
  assert(master == before_invalid);

  // TTL retirement happens even when no additional source message arrives.
  const auto timed_out = coordinator.tick(t0 + 450000000LL, map);
  assert(timed_out.rebuild_required);
  assert(timed_out.update.own_cells.empty());
  assert(timed_out.update.retired_cells.count({2u, 1u}) == 1);
  master[index(2, 1, map.width)] = 0;  // Nav2 re-built the retired window.
  assert(apply(timed_out, true, 2, 1, 3, 2) == 0);
  assert(master[index(2, 1, map.width)] == 0);
  assert(master[index(1, 1, map.width)] == 175);

  // Rolling maps reproject world positions, not previous cell indices.
  assert(coordinator.ingest_validated(
      {3, t0 + 500000000LL, t0 + 900000000LL, 3.5, 2.5, true},
      t0 + 510000000LL));
  const auto back = coordinator.tick(t0 + 510000000LL, map);
  assert(back.update.own_cells.count({3u, 2u}) == 1);
  const GridGeometry rolling{7, 5, 1., 1., 0.};
  const auto shifted = coordinator.tick(t0 + 520000000LL, rolling);
  assert(shifted.rebuild_required && shifted.update.geometry_changed);
  assert(shifted.update.own_cells.count({2u, 2u}) == 1);
  std::vector<std::uint8_t> rolling_master(rolling.width * rolling.height, 0);
  const auto changes = AcousticNav2CycleAdapter::overlay_after_nav2_reset(
      shifted, rolling, 0, 0, 7, 5,
      rolling_master.data(), rolling_master.size(), true);
  assert(changes == 1 && rolling_master[index(2, 2, rolling.width)] == 100);

  return 0;
}
