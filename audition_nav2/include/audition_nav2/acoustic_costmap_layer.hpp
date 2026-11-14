#pragma once
#include <memory>
#include <mutex>
#include <cstdint>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "audition_msgs/msg/localized_source_array.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "audition_nav2/acoustic_layer_coordinator.hpp"
#include "audition_nav2/acoustic_map_generation.hpp"
#include "audition_nav2/acoustic_baseline_gate.hpp"
#include "audition_nav2/acoustic_baseline_tile_pipeline.hpp"
#include "audition_nav2/acoustic_nav2_cycle_adapter.hpp"

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
  // Narrow white-box regression seam for the bounds/ingress/costs race.
  // Never exposed by the plugin ABI or production ROS interface.
  friend struct AcousticCostmapLayerTestAccess;
  // Expire ownership on a wall-clock cadence even without incoming messages.
  // No acoustic observation source is wired yet; the timer is fail-closed.
  void expireOnTimer();
  // Explicitly opt-in only to an independently validated metric ROS stream.
  // The ordinary M4 diagnostic output is never automatically subscribed.
  void onTrustedMetricArray(const audition_msgs::msg::LocalizedSourceArray& msg);
  rclcpp::Subscription<audition_msgs::msg::LocalizedSourceArray>::SharedPtr
      verified_metric_subscription_;
  std::uint64_t last_metric_generation_ = 0;
  // Bounded ingress watermark. Repeated reliable ROS snapshots must not
  // revoke an otherwise live, verified source; older timestamps cannot
  // resurrect one after TTL expiry.
  std::map<std::uint64_t, std::int64_t> last_metric_stamp_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  AcousticLayerCoordinator coordinator_;
  // Tracks Nav2 map geometry, but does not certify a non-acoustic baseline.
  AcousticMapGeneration map_generation_;
  // Every map reset/geometry transition revokes any pending baseline admission.
  // No ticket is issued until an independent non-acoustic rebuild exists.
  AcousticBaselineGate baseline_gate_;
  // Staging is revoked on every lifecycle/geometry/expiry transition.
  // It cannot authorize writes until a separate non-acoustic Nav2 producer exists.
  AcousticBaselineTilePipeline baseline_pipeline_;
  // A cycle is captured in updateBounds and consumed at most once in
  // updateCosts. It must match the Nav2 geometry before any master write.
  AcousticCycle pending_cycle_;
  GridGeometry pending_geometry_{};
  bool pending_valid_ = false;
};
}  // namespace audition_nav2
