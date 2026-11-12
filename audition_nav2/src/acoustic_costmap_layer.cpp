#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <functional>
#include "audition_nav2/acoustic_world_bounds.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace audition_nav2 {
void AcousticCostmapLayer::onInitialize() {
  // No subscription is installed: there is no trusted metric position source.
  // This plugin is intentionally inert until an independently verified
  // evidence/TF adapter and master-map clearing integration are implemented.
  enabled_ = false;
  current_ = true;
  coordinator_.disable();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  // The layer remains opt-in and has no evidence subscriber. A timer is
  // nevertheless required: future evidence must expire without a new packet.
  // Capture a weak lifecycle node to avoid extending its lifetime.
  auto node = node_.lock();
  if (node) {
    expiry_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AcousticCostmapLayer::expireOnTimer, this));
  }
}
void AcousticCostmapLayer::expireOnTimer() {
  std::lock_guard<std::mutex> lock(mutex_);
  // No trusted evidence is connected. Revoke, never generate map costs.
  coordinator_.revoke_all();
  baseline_gate_.invalidate();
  // Only a transaction older than its steady-clock deadline is aborted.
  // A periodic timer must not discard a healthy multi-tick rebuild.
  baseline_pipeline_.expire_if_stalled();
}
void AcousticCostmapLayer::updateBounds(double, double, double,
                                       double* min_x, double* min_y,
                                       double* max_x, double* max_y) {
  // Reject incomplete callbacks without modifying another layer's bounds.
  if (!min_x || !min_y || !max_x || !max_y) return;
  std::lock_guard<std::mutex> lock(mutex_);
  // No trusted metric subscriber or master reconstruction contract exists.
  // Keep the coordinator disabled, but exercise its real geometry/TTL path
  // from the Nav2 update loop. No acoustic costs can be generated here.
  if (!layered_costmap_) return;
  const auto* map = layered_costmap_->getCostmap();
  if (!map) return;
  const GridGeometry geometry{
      map->getSizeInCellsX(), map->getSizeInCellsY(),
      map->getResolution(), map->getOriginX(), map->getOriginY()};
  // Observe geometry on every bounds callback, even while inert. A change
  // revokes any previously issued rebuild cursor before the next cycle.
  // This token alone is NOT proof of an independent non-acoustic baseline.
  const auto previous_generation = map_generation_.generation();
  const auto observed_generation = map_generation_.observe(geometry);
  if (!observed_generation || observed_generation != previous_generation) {
    baseline_gate_.invalidate();
    baseline_pipeline_.abort();
    coordinator_.revoke_all();
    if (!observed_generation) return;
  }
  const auto node = node_.lock();
  if (!node) return;
  const auto cycle = coordinator_.tick(node->now().nanoseconds(), geometry);
  if (!cycle.rebuild_required) return;
  const auto bounds = AcousticWorldBounds::from_cells(cycle.dirty, geometry);
  AcousticWorldBounds::expand(bounds, min_x, min_y, max_x, max_y);
}
void AcousticCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D& master,
                                      int, int, int, int) {
  std::lock_guard<std::mutex> lock(mutex_);
  // A resize or rolling shift may occur between bounds and costs callbacks.
  // Detect it here as well; the layer still cannot write until a trusted
  // independent baseline adapter is implemented and validated.
  const GridGeometry geometry{
      master.getSizeInCellsX(), master.getSizeInCellsY(),
      master.getResolution(), master.getOriginX(), master.getOriginY()};
  const auto previous_generation = map_generation_.generation();
  const auto observed_generation = map_generation_.observe(geometry);
  if (!observed_generation || observed_generation != previous_generation) {
    baseline_gate_.invalidate();
    baseline_pipeline_.abort();
    coordinator_.revoke_all();
  }
  // No costmap writes: bearing-only and diagnostic hypotheses stay inert.
}
void AcousticCostmapLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_.revoke_all();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  current_ = true;
}
}  // namespace audition_nav2

PLUGINLIB_EXPORT_CLASS(audition_nav2::AcousticCostmapLayer, nav2_costmap_2d::Layer)
