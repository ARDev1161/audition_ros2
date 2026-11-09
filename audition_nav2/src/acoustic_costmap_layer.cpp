#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace audition_nav2 {
void AcousticCostmapLayer::onInitialize() {
  // No subscription is installed: there is no trusted metric position source.
  // This plugin is intentionally inert until an independently verified
  // evidence/TF adapter and master-map clearing integration are implemented.
  enabled_ = false;
  current_ = true;
  coordinator_.disable();
}
void AcousticCostmapLayer::updateBounds(double, double, double,
                                       double*, double*, double*, double*) {
  // No dirty bounds while inactive; never modify another layer's bounds.
}
void AcousticCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D&,
                                      int, int, int, int) {
  // Do not overlay soft costs without a verified master reset contract.
}
void AcousticCostmapLayer::reset() {
  coordinator_.revoke_all();
  current_ = true;
}
}  // namespace audition_nav2

PLUGINLIB_EXPORT_CLASS(audition_nav2::AcousticCostmapLayer, nav2_costmap_2d::Layer)
