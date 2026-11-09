#pragma once
#include <memory>
#include <mutex>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "audition_nav2/acoustic_layer_coordinator.hpp"

namespace audition_nav2 {
// Fail-closed plugin shell. No subscriptions until authenticated metric evidence
// and stamped TF validation are connected; never synthesize map cells.
class AcousticCostmapLayer final : public nav2_costmap_2d::Layer {
 public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }
 private:
  // Expire ownership on a wall-clock cadence even without incoming messages.
  // No acoustic observation source is wired yet; the timer is fail-closed.
  void expireOnTimer();
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  AcousticLayerCoordinator coordinator_;
};
}  // namespace audition_nav2
