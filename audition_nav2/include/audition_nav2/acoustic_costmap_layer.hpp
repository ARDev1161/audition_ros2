#pragma once
#include <memory>
#include <mutex>
#include <cstdint>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
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
  // Operator rollback without restarting Nav2. Re-enable requires a new
  // verified metric generation, so queued DDS data cannot restore costs.
  void setSoftCosts(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  // Read-only operator diagnostics: no implicit enable or costmap mutation.
  void getSoftCostsStatus(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // Explicitly opt-in only to an independently validated metric ROS stream.
  // The ordinary M4 diagnostic output is never automatically subscribed.
  void onTrustedMetricArray(const audition_msgs::msg::LocalizedSourceArray& msg);
  rclcpp::Subscription<audition_msgs::msg::LocalizedSourceArray>::SharedPtr
      verified_metric_subscription_;
  std::uint64_t last_metric_generation_ = 0;
  // Complete-snapshot order fence, independent of per-source timestamps.
  std::int64_t last_snapshot_stamp_ns_ = 0;
  // Invalid/future-stamped snapshots cannot establish an ordering watermark.
  // Quarantine their generation until a strictly newer verified epoch.
  bool quarantined_generation_ = false;
  // Bounded ingress watermark. Repeated reliable ROS snapshots must not
  // revoke an otherwise live, verified source; older timestamps cannot
  // resurrect one after TTL expiry.
  std::map<std::uint64_t, std::int64_t> last_metric_stamp_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr soft_costs_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr soft_costs_status_service_;
  bool trusted_topic_configured_ = false;
  // Nav2 registers plugins after onInitialize(); authorize startup opt-in
  // only when the complete plugin roster can be inspected in updateBounds.
  bool startup_enable_requested_ = false;
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
  // A staged overlay cannot outlive the verified metric TTL while Nav2 stalls.
  std::int64_t pending_stamp_ns_ = 0;
  static bool pending_fresh(std::int64_t staged_ns, std::int64_t now_ns);
};
}  // namespace audition_nav2
