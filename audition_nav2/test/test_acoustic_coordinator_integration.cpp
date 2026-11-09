#include "audition_nav2/acoustic_layer_coordinator.hpp"
#include <cassert>
#include <cstdint>
#include <vector>

using namespace audition_nav2;

// Simulates the Nav2 contract: underlying layers rebuild the dirty region
// before the acoustic layer overlays its *current* nonlethal costs.
static void rebuild_and_overlay(const AcousticCycle& cycle,
                                const GridGeometry& grid,
                                std::vector<std::uint8_t>& master,
                                const std::vector<std::uint8_t>& baseline) {
  if (!cycle.rebuild_required) return;
  for (unsigned j = cycle.dirty.min_j; j < cycle.dirty.max_j; ++j)
    for (unsigned i = cycle.dirty.min_i; i < cycle.dirty.max_i; ++i)
      master[static_cast<std::size_t>(j) * grid.width + i] =
          baseline[static_cast<std::size_t>(j) * grid.width + i];
  AcousticLayerCoordinator::overlay_after_rebuild(
      cycle, grid, cycle.dirty.min_i, cycle.dirty.min_j,
      cycle.dirty.max_i, cycle.dirty.max_j, master.data(), master.size());
}

int main() {
  const GridGeometry grid{6, 6, 1.0, 0.0, 0.0};
  AcousticLayerCoordinator coordinator(3, 80);
  std::vector<std::uint8_t> baseline(36, 0), master(36, 0);
  baseline[2 * 6 + 2] = 254;  // Obstacle belongs to another layer.
  baseline[4 * 6 + 4] = 255;  // Unknown must remain unknown.
  baseline[3 * 6 + 3] = 110;  // Higher non-acoustic cost.
  master = baseline;

  // Disabled by default. No implicit acoustic authorization.
  assert(!coordinator.ingest_validated({1, 100, 200, 1, 1, true}, 110));
  assert(!coordinator.tick(110, grid).rebuild_required);
  coordinator.enable();
  assert(!coordinator.ingest_validated({1, 100, 200, 1, 1, false}, 110));
  assert(coordinator.ingest_validated({1, 100, 200, 1, 1, true}, 110));
  assert(coordinator.ingest_validated({2, 100, 160, 2, 2, true}, 110));
  assert(coordinator.ingest_validated({3, 100, 200, 3, 3, true}, 110));
  assert(!coordinator.ingest_validated({4, 100, 200, 4, 4, true}, 110));
  auto first = coordinator.tick(110, grid);
  assert(first.rebuild_required);
  rebuild_and_overlay(first, grid, master, baseline);
  assert(master[1 * 6 + 1] == 80);
  assert(master[2 * 6 + 2] == 254);
  assert(master[3 * 6 + 3] == 110);

  // Expiry without any new messages must retire one source, not all sources.
  auto expiry = coordinator.tick(161, grid);
  assert(expiry.rebuild_required);
  assert(expiry.update.retired_cells.count({2, 2}) == 1);
  rebuild_and_overlay(expiry, grid, master, baseline);
  assert(master[2 * 6 + 2] == 254);
  assert(master[1 * 6 + 1] == 80);
  assert(!coordinator.ingest_validated({2, 100, 160, 2, 2, true}, 162));

  // Disable is a rollback: rebuild removes acoustic cost, preserves obstacles.
  coordinator.disable();
  auto rollback = coordinator.tick(162, grid);
  assert(rollback.rebuild_required);
  rebuild_and_overlay(rollback, grid, master, baseline);
  assert(master == baseline);
  assert(!coordinator.ingest_validated({1, 163, 220, 1, 1, true}, 163));
  coordinator.enable();
  assert(!coordinator.ingest_validated({1, 100, 200, 1, 1, true}, 163));
  assert(coordinator.ingest_validated({1, 164, 220, 1, 1, true}, 164));
  auto renewed = coordinator.tick(164, grid);
  rebuild_and_overlay(renewed, grid, master, baseline);
  assert(master[1 * 6 + 1] == 80);

  // A shifted rolling map requires full-grid rebuild; no old cell indices
  // may be used as an in-place clearing instruction.
  const GridGeometry shifted{6, 6, 1.0, 1.0, 0.0};
  auto rolling = coordinator.tick(165, shifted);
  assert(rolling.update.geometry_changed);
  assert(rolling.dirty.min_i == 0 && rolling.dirty.max_i == 6);
  assert(rolling.dirty.min_j == 0 && rolling.dirty.max_j == 6);
  coordinator.disable();
  auto final = coordinator.tick(166, shifted);
  assert(final.rebuild_required);
  assert(final.update.own_cells.empty());
  assert(!coordinator.tick(167, shifted).rebuild_required);
}
