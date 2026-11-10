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
}
void AcousticCostmapLayer::updateBounds(double, double, double,
                                       double* min_x, double* min_y,
                                       double* max_x, double* max_y) {
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
  const auto node = node_.lock();
  if (!node) return;
  const auto cycle = coordinator_.tick(node->now().nanoseconds(), geometry);
  if (!cycle.rebuild_required) return;
  const auto bounds = AcousticWorldBounds::from_cells(cycle.dirty, geometry);
  AcousticWorldBounds::expand(bounds, min_x, min_y, max_x, max_y);
}
void AcousticCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D&,
                                      int, int, int, int) {
  // Do not overlay soft costs without a verified master reset contract.
}
void AcousticCostmapLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_.revoke_all();
  current_ = true;
}
}  // namespace audition_nav2

PLUGINLIB_EXPORT_CLASS(audition_nav2::AcousticCostmapLayer, nav2_costmap_2d::Layer)
