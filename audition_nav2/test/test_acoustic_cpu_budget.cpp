// M5 reproducible CPU-budget evidence for the bounded, ROS-independent
// acoustic overlay. This is NOT an end-to-end Nav2 or hardware benchmark.
// Prints median, p95, p99 and worst tick latency, plus replay admission time.
// No fragile wall-clock PASS threshold: CI hosts have variable contention.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>
#include "audition_nav2/acoustic_cell_buffer.hpp"

int main() {
  using namespace audition_nav2;
  using Clock = std::chrono::steady_clock;
  constexpr std::int64_t base = 100000000000LL;
  constexpr int cycles = 20000;
  AcousticCellBuffer buffer(32, 100, 5000000000LL, 1000000000LL);
  const GridGeometry grids[] = {
      {64, 64, 0.25, -2.0, -2.0},
      {64, 64, 0.25, -1.0, -2.0},
      {128, 128, 0.125, -2.0, -2.0}};
  for (std::uint64_t id = 1; id <= 32; ++id) {
    const double x = 0.25 * static_cast<double>((id - 1) % 8);
    const double y = 0.25 * static_cast<double>((id - 1) / 8);
    assert(buffer.ingest({id, base, base + 5000000000LL, x, y, true}, base));
  }
  std::vector<std::int64_t> durations;
  durations.reserve(cycles);
  std::int64_t ingest_total_ns = 0;
  std::size_t checksum = 0;
  for (int i = 0; i < cycles; ++i) {
    // Remain within TTL/age throughout: 20,000 * 20 us = 400 ms.
    const auto now = base + (i + 1) * 20000LL;
    const auto start_ingest = Clock::now();
    assert(!buffer.ingest({1, base, base + 5000000000LL,
                           0.0, 0.0, true}, now));
    ingest_total_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now() - start_ingest).count();
    const auto start_tick = Clock::now();
    const auto result = buffer.tick(now, grids[i % 3]);
    durations.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now() - start_tick).count());
    checksum += result.own_cells.size();
    assert(buffer.source_count() == 32);
    assert(result.own_cells.size() <= 32);
    assert(result.changed_cells.size() <= 32);
    assert(result.retired_cells.size() <= 32);
  }
  std::sort(durations.begin(), durations.end());
  const auto percentile = [&](std::size_t p) {
    return durations[(durations.size() - 1) * p / 100];
  };
  std::cout << "acoustic_cell_buffer cycles=" << cycles
            << " sources=32 rolling_resize=on"
            << " tick_median_ns=" << percentile(50)
            << " tick_p95_ns=" << percentile(95)
            << " tick_p99_ns=" << percentile(99)
            << " tick_max_ns=" << durations.back()
            << " replay_mean_ns=" << ingest_total_ns / cycles
            << " checksum=" << checksum << '\n';
  assert(checksum == static_cast<std::size_t>(cycles) * 32);
  // Timer-only expiration must retire ownership on the SAME geometry as
  // the last tick. Grid indices are not valid across rolling/resize changes.
  // The 20,000th tick uses grids[(cycles - 1) % 3], not grids[2].
  buffer.expire_without_tick(base + 5000000001LL);
  const auto cleared = buffer.tick(base + 5000000002LL,
                                   grids[(cycles - 1) % 3]);
  assert(cleared.own_cells.empty());
  assert(!cleared.geometry_changed);
  assert(cleared.retired_cells.size() == 32);
  // A later geometry change must not re-emit obsolete grid indices.
  const auto resized_after_clear = buffer.tick(base + 5000000003LL, grids[2]);
  assert(resized_after_clear.geometry_changed);
  assert(resized_after_clear.own_cells.empty());
  assert(resized_after_clear.retired_cells.empty());
  return 0;
}
