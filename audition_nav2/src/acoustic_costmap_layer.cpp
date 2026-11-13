#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include "audition_nav2/acoustic_world_bounds.hpp"
#include "audition_nav2/acoustic_verified_metric_input.hpp"
#include <string>
#include <vector>
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace audition_nav2 {

void AcousticCostmapLayer::onInitialize() {
  // No verified ROS metric-evidence ingress has been connected yet. Keep
  // disabled until a trusted, timestamped TF + provenance adapter exists.
  enabled_ = false;
  current_ = true;
  coordinator_.disable();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  pending_valid_ = false;
  auto node = node_.lock();
  if (node) {
    declareParameter("enable_soft_costs", rclcpp::ParameterValue(false));
    declareParameter("trusted_metric_topic", rclcpp::ParameterValue(std::string("")));
    bool requested = false;
    std::string topic;
    node->get_parameter(name_ + ".enable_soft_costs", requested);
    node->get_parameter(name_ + ".trusted_metric_topic", topic);
    // This topic is an OPERATOR-APPROVED upstream trust boundary, not
    // M4's unverified diagnostic localization stream. Its messages still
    // undergo strict generation, timestamp, method, covariance and
    // same-global-frame checks before any cell is admitted.
    if (requested && !topic.empty()) {
      enabled_ = true;
      coordinator_.enable();
      verified_metric_subscription_ =
          node->create_subscription<audition_msgs::msg::LocalizedSourceArray>(
              topic, rclcpp::QoS(10).reliable(),
              [this](const audition_msgs::msg::LocalizedSourceArray& msg) {
                onTrustedMetricArray(msg);
              });
    } else if (requested) {
      RCLCPP_WARN(logger_,
          "Acoustic soft costs requested without trusted_metric_topic; layer disabled");
    }
    expiry_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AcousticCostmapLayer::expireOnTimer, this));
  }
}

void AcousticCostmapLayer::onTrustedMetricArray(
    const audition_msgs::msg::LocalizedSourceArray& msg) {
  if (!clock_ || !layered_costmap_) return;
  const auto now_ns = clock_->now().nanoseconds();
  std::vector<StampedMetricEvidence> evidence;
  // Decode outside the state mutex; the callback performs bounded
  // validation (at most 32 sources) and cannot change the costmap.
  const bool valid = AcousticVerifiedMetricInput::decode(
      msg, layered_costmap_->getGlobalFrameID(), now_ns, evidence);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  // Replayed older sessions cannot revoke a newer trusted observation.
  if (msg.generation < last_metric_generation_) return;
  if (!valid) {
    coordinator_.revoke_all();
    return;
  }
  if (msg.generation != last_metric_generation_) {
    coordinator_.revoke_all();
    last_metric_generation_ = msg.generation;
  }
  if (evidence.empty()) {
    coordinator_.revoke_all();  // Explicit empty verified snapshot.
    return;
  }
  for (const auto& item : evidence) {
    if (!coordinator_.ingest_evidence(item, now_ns)) {
      // Do not leave a partially updated batch visible to Nav2.
      coordinator_.revoke_all();
      return;
    }
  }
}

void AcousticCostmapLayer::expireOnTimer() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Only inactive/untrusted evidence is revoked here. Once a verified
  // ingress exists, TTL is checked by coordinator.tick() on each Nav2
  // updateBounds call. Do NOT tick here: that would consume retired/dirty
  // cells before Nav2 has a chance to rebuild them.
  if (!enabled_) coordinator_.revoke_all();
  baseline_pipeline_.expire_if_stalled();
}

void AcousticCostmapLayer::updateBounds(
    double, double, double, double* min_x, double* min_y,
    double* max_x, double* max_y) {
  if (!min_x || !min_y || !max_x || !max_y) return;
  std::lock_guard<std::mutex> lock(mutex_);
  pending_valid_ = false;
  if (!layered_costmap_) return;
  const auto* map = layered_costmap_->getCostmap();
  if (!map) return;

  const GridGeometry geometry{
      map->getSizeInCellsX(), map->getSizeInCellsY(),
      map->getResolution(), map->getOriginX(), map->getOriginY()};
  const auto previous_generation = map_generation_.generation();
  const auto observed_generation = map_generation_.observe(geometry);
  if (!observed_generation || observed_generation != previous_generation) {
    baseline_gate_.invalidate();
    baseline_pipeline_.abort();
    // A rolling origin or resize must NOT invalidate still-fresh world-frame
    // evidence: the cell buffer reprojects world positions on the new grid.
    if (!observed_generation) {
      coordinator_.revoke_all();
      return;
    }
  }

  if (!enabled_) return;
  // An acoustic plugin before any ordinary layer might write costs to the
  // previous master. A terminal plugin is called AFTER Nav2's normal
  // resetMap + non-acoustic updateCosts sequence.
  const auto* plugins = layered_costmap_->getPlugins();
  const auto* filters = layered_costmap_->getFilters();
  if (!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
          plugins, filters, static_cast<const nav2_costmap_2d::Layer*>(this))) {
    coordinator_.revoke_all();
    return;
  }

  const auto node = node_.lock();
  if (!node) return;
  pending_cycle_ = coordinator_.tick(node->now().nanoseconds(), geometry);
  pending_geometry_ = geometry;
  pending_valid_ = true;

  if (!pending_cycle_.rebuild_required) return;
  const auto bounds =
      AcousticWorldBounds::from_cells(pending_cycle_.dirty, geometry);
  AcousticWorldBounds::expand(bounds, min_x, min_y, max_x, max_y);
}

void AcousticCostmapLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master,
    int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2's updateMap() always resets the requested master window and
  // executes non-acoustic plugins in order before invoking a terminal
  // acoustic plugin. Never recursively call other layers or copy from the
  // previous master to build a baseline.
  const GridGeometry geometry{
      master.getSizeInCellsX(), master.getSizeInCellsY(),
      master.getResolution(), master.getOriginX(), master.getOriginY()};
  const auto previous_generation = map_generation_.generation();
  const auto observed_generation = map_generation_.observe(geometry);
  if (!observed_generation || observed_generation != previous_generation) {
    baseline_gate_.invalidate();
    baseline_pipeline_.abort();
    pending_valid_ = false;
    // Geometry changed between the bounds and costs phases. Fail closed
    // and wait for the next complete Nav2 updateMap cycle.
    return;
  }
  if (!enabled_ || !pending_valid_ || !layered_costmap_) return;
  pending_valid_ = false;  // consume once, including on any failed gate
  const auto& g = pending_geometry_;
  if (g.width != geometry.width || g.height != geometry.height ||
      g.resolution != geometry.resolution ||
      g.origin_x != geometry.origin_x || g.origin_y != geometry.origin_y)
    return;

  const auto* plugins = layered_costmap_->getPlugins();
  const auto* filters = layered_costmap_->getFilters();
  if (!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
          plugins, filters, static_cast<const nav2_costmap_2d::Layer*>(this)))
    return;

  // Without filters Nav2 passes combined_costmap_; with filters it passes
  // private primary_costmap_, then copies it and applies filters. Reject
  // unexpected buffers in either mode, without dereferencing stale pointers.
  const auto* combined = layered_costmap_->getCostmap();
  if (!combined || !filters) return;
  if (filters->empty() ? (&master != combined) : (&master == combined))
    return;

  const auto count = static_cast<std::uint64_t>(geometry.width) * geometry.height;
  if (count == 0 || count > 4000000ULL) return;
  AcousticNav2CycleAdapter::overlay_after_nav2_reset(
      pending_cycle_, geometry, min_i, min_j, max_i, max_j,
      master.getCharMap(), static_cast<std::size_t>(count), true);
}

void AcousticCostmapLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_.revoke_all();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  pending_valid_ = false;
  current_ = true;
}
}  // namespace audition_nav2

PLUGINLIB_EXPORT_CLASS(audition_nav2::AcousticCostmapLayer, nav2_costmap_2d::Layer)
