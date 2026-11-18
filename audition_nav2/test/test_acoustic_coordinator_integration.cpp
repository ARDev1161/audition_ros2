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
  // First grid establishment may request a full reset even with no sources.
  assert(coordinator.tick(110, grid).update.own_cells.empty());
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
  // Periodic timer ticks with unchanged evidence must not request a
  // master-map rebuild. The complete overlay remains available separately.
  for (std::int64_t t = 111; t < 160; ++t) {
    auto idle = coordinator.tick(t, grid);
    assert(!idle.rebuild_required);
    assert(idle.dirty.empty());
    assert(idle.update.own_cells.size() == 3);
    assert(idle.update.changed_cells.empty());
  }

  // Moving a single source dirties only the old and new cells; other
  // sources remain in the full overlay without expanding the rebuild bounds.
  assert(coordinator.ingest_validated({1, 159, 200, 1, 2, true}, 159));
  auto moved = coordinator.tick(159, grid);
  assert(moved.rebuild_required);
  assert(moved.update.retired_cells.count({1, 1}) == 1);
  assert(moved.update.changed_cells.count({1, 2}) == 1);
  assert(moved.update.changed_cells.size() == 1);
  assert(moved.dirty.min_i == 1 && moved.dirty.max_i == 2);
  assert(moved.dirty.min_j == 1 && moved.dirty.max_j == 3);
  rebuild_and_overlay(moved, grid, master, baseline);
  assert(master[1 * 6 + 1] == baseline[1 * 6 + 1]);
  assert(master[2 * 6 + 1] == 80);
  assert(master[3 * 6 + 3] == 110);

  // Expiry without any new messages must retire one source, not all sources.
  auto expiry = coordinator.tick(161, grid);
  assert(expiry.rebuild_required);
  assert(expiry.update.retired_cells.count({2, 2}) == 1);
  rebuild_and_overlay(expiry, grid, master, baseline);
  assert(master[2 * 6 + 2] == 254);
  assert(master[2 * 6 + 1] == 80);
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

  // Deterministic multi-source stress: repeated movement, overlapping cells,
  // revocation, TTL, rolling origin, and idle cycles. Compare the incremental
  // dirty-region update with a complete baseline rebuild at every step.
  AcousticLayerCoordinator stress(8, 90);
  stress.enable();
  GridGeometry sg{12, 12, 1.0, 0.0, 0.0};
  std::vector<std::uint8_t> base(144, 0), incremental(144, 0);
  for (unsigned j = 0; j < 12; ++j)
    for (unsigned i = 0; i < 12; ++i) {
      const auto k = static_cast<std::size_t>(j) * 12 + i;
      base[k] = (i + 3 * j) % 17 == 0 ? 254 :
                (i + j) % 19 == 0 ? 255 :
                (i + j) % 11 == 0 ? 130 : 0;
    }
  incremental = base;
  for (std::int64_t step = 1; step <= 480; ++step) {
    const auto now = 100000 + step;
    if (step % 80 == 0)
      sg.origin_x = (step / 80) % 3;
    if (step % 37 == 0) stress.revoke(static_cast<std::uint64_t>(step % 8 + 1));
    if (step % 113 == 0) stress.disable();
    if (step % 113 == 1) stress.enable();
    if (step % 3 == 0) {
      const auto id = static_cast<std::uint64_t>(step % 8 + 1);
      const double x = static_cast<double>((step / 3 + id) % 9 + 1);
      const double y = static_cast<double>((step / 9 + id) % 9 + 1);
      (void)stress.ingest_validated({id, now, now + 19, x, y, true}, now);
    }
    const auto cycle = stress.tick(now, sg);
    rebuild_and_overlay(cycle, sg, incremental, base);
    auto expected = base;
    // A complete rebuild always applies the full current overlay, even when
    // no dirty bounds were produced this cycle.
    AcousticMasterMerge::apply(cycle.update, sg, 0, 0, 12, 12,
                               expected.data(), expected.size());
    assert(incremental == expected);
    for (const auto cost : incremental)
      assert(cost == 0 || cost == 90 || cost == 130 ||
             cost == 254 || cost == 255);
  }

  // Two independently confirmed sources may occupy one cell. Retiring the
  // first must not clear the second; only the final retirement may restore
  // the non-acoustic baseline. Repeat with a lethal/unknown neighbor to
  // guard against accidental cross-layer clearing.
  AcousticLayerCoordinator overlap(4, 65);
  overlap.enable();
  const GridGeometry og{5, 5, 1.0, 0.0, 0.0};
  std::vector<std::uint8_t> ob(25, 0), om(25, 0);
  ob[1 * 5 + 1] = 254;
  ob[3 * 5 + 3] = 255;
  om = ob;
  assert(overlap.ingest_validated({21, 2000, 2050, 2, 2, true}, 2000));
  assert(overlap.ingest_validated({22, 2000, 2200, 2, 2, true}, 2000));
  auto both = overlap.tick(2000, og);
  rebuild_and_overlay(both, og, om, ob);
  assert(om[2 * 5 + 2] == 65);
  auto one_expired = overlap.tick(2051, og);
  rebuild_and_overlay(one_expired, og, om, ob);
  assert(om[2 * 5 + 2] == 65);
  assert(om[1 * 5 + 1] == 254 && om[3 * 5 + 3] == 255);
  // No incoming messages or Nav2 cycles until both sources have expired.
  auto all_expired = overlap.tick(2201, og);
  assert(all_expired.rebuild_required);
  rebuild_and_overlay(all_expired, og, om, ob);
  assert(om == ob);
  assert(!overlap.ingest_validated({21, 2000, 2050, 2, 2, true}, 2202));
  assert(!overlap.ingest_validated({22, 2000, 2200, 2, 2, true}, 2202));

  // A resize is a new coordinate system, not an opportunity to replay
  // stale grid indices. Keep only still-live world-frame sources, and
  // recompute the overlay against the newly rebuilt non-acoustic baseline.
  AcousticLayerCoordinator resize(4, 72);
  resize.enable();
  const GridGeometry small{4, 4, 1.0, 0.0, 0.0};
  const GridGeometry large{7, 5, 1.0, -1.0, -1.0};
  std::vector<std::uint8_t> small_base(16, 0), small_master(16, 0);
  small_base[1 * 4 + 1] = 254;
  small_master = small_base;
  assert(resize.ingest_validated({31, 3000, 3200, 2.0, 2.0, true}, 3000));
  assert(resize.ingest_validated({32, 3000, 3020, 3.0, 3.0, true}, 3000));
  auto small_cycle = resize.tick(3000, small);
  rebuild_and_overlay(small_cycle, small, small_master, small_base);
  assert(small_master[2 * 4 + 2] == 72);
  assert(small_master[1 * 4 + 1] == 254);

  // No Nav2 updates during resize; the short-lived source expires while
  // the persistent source moves to a different cell in the larger grid.
  std::vector<std::uint8_t> large_base(35, 0), large_master(35, 0);
  large_base[2 * 7 + 2] = 255;
  large_base[3 * 7 + 3] = 130;
  large_master = large_base;
  auto resized = resize.tick(3030, large);
  assert(resized.update.geometry_changed);
  assert(resized.rebuild_required);
  assert(resized.dirty.min_i == 0 && resized.dirty.max_i == 7);
  assert(resized.dirty.min_j == 0 && resized.dirty.max_j == 5);
  assert(resized.update.own_cells.size() == 1);
  rebuild_and_overlay(resized, large, large_master, large_base);
  assert(large_master[3 * 7 + 3] == 130);
  assert(large_master[2 * 7 + 2] == 255);
  assert(!resize.ingest_validated({32, 3000, 3020, 3.0, 3.0, true}, 3030));
  assert(!resize.tick(3031, large).rebuild_required);

  // TTL retirement after resize restores the new baseline, not the old
  // small-grid master, and a second idle update stays clean.
  auto resized_expiry = resize.tick(3201, large);
  assert(resized_expiry.rebuild_required);
  rebuild_and_overlay(resized_expiry, large, large_master, large_base);
  assert(large_master == large_base);
  assert(!resize.tick(3202, large).rebuild_required);

  // Nav2 may suspend updateMap() while the ROS timer continues. The timer
  // must NOT consume dirty cells: the resumed bounds pass retires expired
  // acoustic costs from the master even without incoming messages.
  AcousticLayerCoordinator paused(4, 75);
  paused.enable();
  const GridGeometry pg{8, 8, 1.0, 0.0, 0.0};
  std::vector<std::uint8_t> pb(64, 0), pm(64, 0);
  pb[1 * 8 + 1] = 130;
  pm = pb;
  assert(paused.ingest_validated({11, 1000, 1050, 2.0, 2.0, true}, 1000));
  assert(paused.ingest_validated({12, 1000, 1200, 1.0, 1.0, true}, 1000));
  auto before_pause = paused.tick(1000, pg);
  rebuild_and_overlay(before_pause, pg, pm, pb);
  assert(pm[2 * 8 + 2] == 75);
  assert(pm[1 * 8 + 1] == 130);
  // No coordinator tick from t=1000 to t=1100: updateMap is suspended.
  auto resumed = paused.tick(1100, pg);
  assert(resumed.rebuild_required);
  assert(resumed.update.retired_cells.count({2, 2}) == 1);
  assert(resumed.update.own_cells.count({2, 2}) == 0);
  rebuild_and_overlay(resumed, pg, pm, pb);
  assert(pm[2 * 8 + 2] == 0);
  assert(pm[1 * 8 + 1] == 130);
  assert(!paused.tick(1101, pg).rebuild_required);
  assert(!paused.ingest_validated({11, 1000, 1050, 2.0, 2.0, true}, 1101));

  // Rolling map while suspended: old grid indices cannot clear the new
  // master; resumed update must request a full-grid baseline rebuild.
  assert(paused.ingest_validated({13, 1102, 1150, 4.0, 4.0, true}, 1102));
  auto pre_roll = paused.tick(1102, pg);
  rebuild_and_overlay(pre_roll, pg, pm, pb);
  assert(pm[4 * 8 + 4] == 75);
  const GridGeometry rolled{8, 8, 1.0, 1.0, 0.0};
  auto after_roll = paused.tick(1201, rolled);
  assert(after_roll.update.geometry_changed);
  assert(after_roll.rebuild_required);
  assert(after_roll.dirty.min_i == 0 && after_roll.dirty.max_i == 8);
  assert(after_roll.dirty.min_j == 0 && after_roll.dirty.max_j == 8);
  assert(after_roll.update.own_cells.empty());
  // Nav2 supplies the new baseline, never the previous acoustic master.
  pm = pb;
  rebuild_and_overlay(after_roll, rolled, pm, pb);
  assert(pm == pb);
  assert(!paused.tick(1202, rolled).rebuild_required);

  // Operator rollback while Nav2 is paused: a disabled layer must clear
  // ownership at the first resumed cycle, without writing over a newer
  // non-acoustic obstacle or accepting a replay from the revoked session.
  AcousticLayerCoordinator emergency(3, 60);
  emergency.enable();
  const GridGeometry eg{6, 4, 0.5, -1.0, -0.5};
  std::vector<std::uint8_t> eb(24, 0), em(24, 0);
  eb[1 * 6 + 3] = 254;
  eb[2 * 6 + 4] = 255;
  em = eb;
  assert(emergency.ingest_validated({41, 4000, 4500, 0.0, 0.0, true}, 4000));
  auto authorized = emergency.tick(4000, eg);
  rebuild_and_overlay(authorized, eg, em, eb);
  assert(em[1 * 6 + 2] == 60);
  emergency.disable();  // No updateMap() call while disabled.
  assert(!emergency.ingest_validated({42, 4001, 4500, 0.5, 0.0, true}, 4001));
  auto disabled_resume = emergency.tick(4100, eg);
  assert(disabled_resume.rebuild_required);
  assert(disabled_resume.update.own_cells.empty());
  rebuild_and_overlay(disabled_resume, eg, em, eb);
  assert(em == eb);
  assert(!emergency.tick(4101, eg).rebuild_required);

  // Reauthorization is not a new replay epoch: only newer metric evidence
  // can create a cost, and the previous baseline must remain intact.
  emergency.enable();
  assert(!emergency.ingest_validated({41, 4000, 4500, 0.0, 0.0, true}, 4102));
  assert(emergency.ingest_validated({41, 4103, 4300, -0.5, 0.0, true}, 4103));
  auto reauthorized = emergency.tick(4103, eg);
  rebuild_and_overlay(reauthorized, eg, em, eb);
  assert(em[1 * 6 + 3] == 254);
  assert(em[1 * 6 + 1] == 60);
  assert(em[1 * 6 + 2] == 0);
  emergency.disable();
  auto second_rollback = emergency.tick(4104, eg);
  rebuild_and_overlay(second_rollback, eg, em, eb);
  assert(em == eb);
}
