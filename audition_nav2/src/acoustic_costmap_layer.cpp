#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
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
  // Fail closed until a trusted metric topic is explicitly configured and
  // the complete Nav2 plugin roster is available for authorization.
  enabled_ = false;
  startup_enable_requested_ = false;
  lifecycle_active_ = true;
  current_ = true;
  coordinator_.disable();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  pending_valid_ = false;
  last_metric_generation_ = 0;
  last_snapshot_stamp_ns_ = 0;
  quarantined_generation_ = false;
  last_metric_stamp_.clear();
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
    trusted_topic_configured_ = !topic.empty();
    if (!topic.empty()) {
      verified_metric_subscription_ =
          node->create_subscription<audition_msgs::msg::LocalizedSourceArray>(
              topic, rclcpp::QoS(10).reliable(),
              [this](const audition_msgs::msg::LocalizedSourceArray& msg) {
                onTrustedMetricArray(msg);
              });
    }
    if (requested && trusted_topic_configured_) {
      // onInitialize runs before Nav2 finishes registering its plugin list.
      // Delay authorization to the first updateBounds, where the complete
      // roster can be checked. Never enable an incorrectly ordered layer.
      startup_enable_requested_ = true;
    } else if (requested) {
      RCLCPP_WARN(logger_,
          "Acoustic soft costs requested without trusted_metric_topic; layer disabled");
    }
    soft_costs_service_ = node->create_service<std_srvs::srv::SetBool>(
        name_ + "/set_soft_costs",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          setSoftCosts(request, response);
        });
    soft_costs_status_service_ = node->create_service<std_srvs::srv::Trigger>(
        name_ + "/get_soft_costs_status",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          getSoftCostsStatus(request, response);
        });
    expiry_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AcousticCostmapLayer::expireOnTimer, this));
  }
}

void AcousticCostmapLayer::setSoftCosts(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request->data && !lifecycle_active_) {
    response->success = false;
    response->message = "Nav2 costmap layer is deactivated; enable rejected";
    return;
  }
  if (request->data && !trusted_topic_configured_) {
    response->success = false;
    response->message = "trusted_metric_topic is required; no acoustic cells admitted";
    return;
  }
  // Reject operator enable when Nav2 has not registered this plugin as the
  // terminal overlay. Otherwise the service could report success while the
  // update cycle silently drops every acoustic observation.
  if (request->data) {
    if (!layered_costmap_ ||
        !AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
            layered_costmap_->getPlugins(), layered_costmap_->getFilters(),
            static_cast<const nav2_costmap_2d::Layer*>(this))) {
      response->success = false;
      response->message =
          "acoustic_layer must be the terminal Nav2 costmap plugin";
      return;
    }
  }
  // An explicit operator request overrides any deferred startup opt-in.
  startup_enable_requested_ = false;
  if (enabled_ == request->data) {
    response->success = true;
    response->message = enabled_ ? "already enabled" : "already disabled";
    return;
  }
  // Disabling is an immediate admission barrier. The previous Nav2 master
  // is cleared by its next normal reset/updateMap pass, never by directly
  // writing into a map owned by other plugins.
  if (pending_valid_) coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
  pending_valid_ = false;
  coordinator_.revoke_all();
  last_metric_stamp_.clear();
  last_snapshot_stamp_ns_ = 0;
  // Require a fresh epoch after either transition. Do not reaccept a
  // queued snapshot from before rollback when the layer is re-enabled.
  quarantined_generation_ = last_metric_generation_ != 0;
  enabled_ = request->data;
  if (enabled_) coordinator_.enable();
  else coordinator_.disable();
  response->success = true;
  response->message = enabled_ ? "enabled; waiting for new verified generation"
                               : "disabled; costs retire on next Nav2 update";
}

void AcousticCostmapLayer::getSoftCostsStatus(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool terminal = layered_costmap_ &&
      AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
          layered_costmap_->getPlugins(), layered_costmap_->getFilters(),
          static_cast<const nav2_costmap_2d::Layer*>(this));
  // Report state rather than silently claiming a usable overlay. A valid
  // metric topic is an operator authorization boundary, not a calibration.
  response->success = true;
  response->message =
      std::string("enabled=") + (enabled_ ? "true" : "false") +
      " trusted_topic_configured=" + (trusted_topic_configured_ ? "true" : "false") +
      " terminal_plugin=" + (terminal ? "true" : "false") +
      " lifecycle_active=" + (lifecycle_active_ ? "true" : "false") +
      " quarantined=" + (quarantined_generation_ ? "true" : "false") +
      " active_sources=" + std::to_string(coordinator_.source_count()) +
      " generation=" + std::to_string(last_metric_generation_);
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
  // updateBounds() and updateCosts() are separate Nav2 callbacks. A trusted
  // ingress may arrive between them, so the previously staged overlay is no
  // longer a snapshot of the coordinator. Reject that half-cycle and rebuild
  // on the next complete updateMap() pass. This also fences revocations and
  // clock/epoch changes against stale soft-cost writes.
  // Old-generation replay is a no-op and must not disrupt a valid cycle.
  if (msg.generation < last_metric_generation_ ||
      (quarantined_generation_ && msg.generation == last_metric_generation_)) return;
  // A delayed complete snapshot must not resurrect a source that a newer
  // snapshot already omitted. Compare the array's own publication stamp,
  // not individual source observation stamps (which may legitimately repeat).
  const std::int64_t snapshot_ns =
      static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg.header.stamp.nanosec);
  // An older *valid* publication is a harmless DDS reorder. An invalid
  // same-generation publication is different: it is contradictory evidence
  // on the operator-approved trust boundary and must revoke the overlay,
  // even if its publication stamp precedes the latest accepted snapshot.
  // Otherwise a malformed replay can be silently ignored while soft costs
  // remain active. Old generations are still ignored by the epoch fence.
  if (valid && msg.generation == last_metric_generation_ &&
      snapshot_ns < last_snapshot_stamp_ns_) return;
  // Publication time identifies a complete snapshot within an epoch.
  // A valid packet with the same timestamp is a DDS replay, not a new
  // authorization to move/add acoustic cells. Ignore it before invalidating
  // a prepared Nav2 half-cycle. Invalid packets are deliberately NOT ignored:
  // conflicting unverified evidence must still revoke and fence this epoch.
  if (valid && msg.generation == last_metric_generation_ &&
      snapshot_ns > 0 && snapshot_ns == last_snapshot_stamp_ns_) return;
  if (pending_valid_) coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
  pending_valid_ = false;
  if (!valid) {
    coordinator_.revoke_all();
    // A rejected complete snapshot is still a revocation barrier. Without
    // advancing its generation/publication watermark, an older valid DDS
    // snapshot can arrive later and resurrect a source we just revoked.
    // Do not trust malformed/future timestamps as replay fences: generation
    // remains the conservative barrier until a newer valid snapshot arrives.
    if (msg.generation > last_metric_generation_) {
      last_metric_generation_ = msg.generation;
      last_snapshot_stamp_ns_ = 0;
    }
    if (snapshot_ns > 0 && snapshot_ns <= now_ns &&
        static_cast<std::uint64_t>(now_ns - snapshot_ns) <=
            static_cast<std::uint64_t>(AcousticVerifiedMetricInput::kTtlNs)) {
      // A rejected snapshot must also fence a valid replay with the
      // exact same publication stamp (DDS duplicates can differ in payload).
      // Advance the watermark by one nanosecond; the timestamp is already
      // bounded by now_ns and cannot overflow int64_t here.
      last_snapshot_stamp_ns_ =
          std::max(last_snapshot_stamp_ns_, snapshot_ns + 1);
    } else {
      // No trustworthy publication time: reject all subsequent packets in
      // this epoch, including previously valid DDS replays.
      quarantined_generation_ = true;
    }
    last_metric_stamp_.clear();
    return;
  }
  if (msg.generation != last_metric_generation_) {
    quarantined_generation_ = false;
    last_snapshot_stamp_ns_ = 0;
    coordinator_.revoke_all();
    last_metric_stamp_.clear();
    last_metric_generation_ = msg.generation;
  }
  if (evidence.empty()) {
    if (valid) last_snapshot_stamp_ns_ = snapshot_ns;
    coordinator_.revoke_all();  // Explicit empty verified snapshot.
    last_metric_stamp_.clear();
    return;
  }
  // Replay fences older than the hard TTL can be pruned: decode() rejects
  // observations older than 250 ms, so they cannot be resurrected.
  for (auto it = last_metric_stamp_.begin();
       it != last_metric_stamp_.end();) {
    if (it->second <= now_ns &&
        static_cast<std::uint64_t>(now_ns) -
        static_cast<std::uint64_t>(it->second) >
        static_cast<std::uint64_t>(AcousticVerifiedMetricInput::kTtlNs))
      it = last_metric_stamp_.erase(it);
    else
      ++it;
  }
  // Preflight the whole batch before accepting its first new observation.
  // Reliable DDS may legitimately replay an unchanged complete snapshot:
  // equality is a no-op, not a reason to revoke a still-live source.
  std::size_t unseen = 0;
  for (const auto& item : evidence) {
    const auto it = last_metric_stamp_.find(item.cell.source_id);
    if (it != last_metric_stamp_.end()) {
      if (item.cell.stamp_ns < it->second) {
        // A newer complete snapshot carrying an older observation is not a
        // harmless DDS replay. Its publication timestamp passed the replay
        // fence above, so retaining the previous soft costs would allow a
        // stale/reassigned source to remain in the master map until TTL.
        // Reject the entire snapshot and fence this generation. A fresh,
        // strictly newer verified generation is required to resume.
        coordinator_.revoke_all();
        last_metric_stamp_.clear();
        quarantined_generation_ = true;
        return;
      }
    } else {
      ++unseen;
    }
  }
  if (last_metric_stamp_.size() + unseen > 128) {
    coordinator_.revoke_all();
    last_metric_stamp_.clear();
    quarantined_generation_ = true;
    return;
  }
  for (const auto& item : evidence) {
    const auto it = last_metric_stamp_.find(item.cell.source_id);
    if (it != last_metric_stamp_.end() &&
        item.cell.stamp_ns == it->second) continue;
    if (!coordinator_.ingest_evidence(item, now_ns)) {
      // A partially accepted batch is never exposed to the next Nav2 tick.
      coordinator_.revoke_all();
      last_metric_stamp_.clear();
      quarantined_generation_ = true;
      return;
    }
    last_metric_stamp_[item.cell.source_id] = item.cell.stamp_ns;
  }
  last_snapshot_stamp_ns_ = snapshot_ns;
  // The trusted array is a complete snapshot, not an incremental stream.
  // A source omitted from a newer valid snapshot must lose ownership now,
  // rather than retaining a ghost soft cost until its previous TTL. Keep
  // replay watermarks only for members of the current snapshot; a later
  // reappearance still undergoes timestamp/epoch admission.
  for (auto it = last_metric_stamp_.begin();
       it != last_metric_stamp_.end();) {
    const bool present = std::any_of(evidence.begin(), evidence.end(),
        [id = it->first](const StampedMetricEvidence& item) {
          return item.cell.source_id == id;
        });
    if (!present) {
      coordinator_.revoke(it->first);
      it = last_metric_stamp_.erase(it);
    } else {
      ++it;
    }
  }
}

void AcousticCostmapLayer::expireOnTimer() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Expire hypotheses even when Nav2 updateMap is paused. Do not tick:
  // consuming dirty cells here would lose the master-map rebuild request.
  if (!enabled_) coordinator_.revoke_all();
  else if (clock_) {
    const auto now_ns = clock_->now().nanoseconds();
    const auto count_before = coordinator_.source_count();
    coordinator_.expire_without_tick(now_ns);
    // A normal timer tick must not invalidate an otherwise fresh Nav2
    // bounds/costs pair. Only a real source retirement fences staged cells;
    // updateCosts independently checks the staging timestamp and clock.
    if (coordinator_.source_count() != count_before) {
      if (pending_valid_) coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
      pending_valid_ = false;
    }
  }
  baseline_pipeline_.expire_if_stalled();
}

bool AcousticCostmapLayer::pending_fresh(std::int64_t staged_ns,
                                          std::int64_t now_ns) {
  return staged_ns > 0 && now_ns >= staged_ns &&
      (static_cast<std::uint64_t>(now_ns) -
       static_cast<std::uint64_t>(staged_ns)) <=
          static_cast<std::uint64_t>(AcousticVerifiedMetricInput::kTtlNs);
}

void AcousticCostmapLayer::updateBounds(
    double, double, double, double* min_x, double* min_y,
    double* max_x, double* max_y) {
  if (!min_x || !min_y || !max_x || !max_y) return;
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2 may invoke updateBounds twice without a matching updateCosts
  // (interrupted updateMap, lifecycle transition, or a changed window).
  // The first pass already consumed coordinator dirty ownership. Preserve
  // that region for the next full pass instead of silently dropping it.
  if (pending_valid_)
    coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
  pending_valid_ = false;
  if (!layered_costmap_) return;
  if (startup_enable_requested_ && lifecycle_active_) {
    // First updateMap after Nav2 registration: apply the same terminal
    // plugin authorization as the operator SetBool service. A bad roster
    // remains disabled instead of silently discarding costmap updates.
    startup_enable_requested_ = false;
    if (trusted_topic_configured_ &&
        AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
            layered_costmap_->getPlugins(), layered_costmap_->getFilters(),
            static_cast<const nav2_costmap_2d::Layer*>(this))) {
      enabled_ = true;
      coordinator_.enable();
      RCLCPP_INFO(logger_, "Acoustic soft costs enabled after Nav2 plugin roster validation");
    } else {
      RCLCPP_ERROR(logger_,
          "Acoustic soft costs remain disabled: acoustic_layer must be the terminal Nav2 plugin");
    }
  }
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

  if (!enabled_ || !lifecycle_active_) {
    // Disabling must still advertise previously owned cells as dirty so
    // Nav2 rebuilds them from non-acoustic layers. Simply returning here
    // leaves old soft costs in the master until some unrelated map update.
    if (clock_) {
      const auto cycle = coordinator_.tick(clock_->now().nanoseconds(), geometry);
      if (cycle.rebuild_required) {
        const auto bounds = AcousticWorldBounds::from_cells(cycle.dirty, geometry);
        AcousticWorldBounds::expand(bounds, min_x, min_y, max_x, max_y);
      }
    }
    return;
  }
  // An acoustic plugin before any ordinary layer might write costs to the
  // previous master. A terminal plugin is called AFTER Nav2's normal
  // resetMap + non-acoustic updateCosts sequence.
  const auto* plugins = layered_costmap_->getPlugins();
  const auto* filters = layered_costmap_->getFilters();
  if (!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
          plugins, filters, static_cast<const nav2_costmap_2d::Layer*>(this))) {
    // A previously authorized layer can lose its terminal position after
    // a Nav2 reconfiguration. Revoke ownership AND request a master rebuild
    // of its old cells; returning here would strand stale acoustic costs.
    coordinator_.disable();
    enabled_ = false;
    coordinator_.revoke_all();
    last_metric_stamp_.clear();
    last_snapshot_stamp_ns_ = 0;
    quarantined_generation_ = last_metric_generation_ != 0;
    // Keep the revoked ownership dirty until a subsequent updateBounds.
    // A roster change can interrupt the current Nav2 updateMap pass; consuming
    // dirty cells here could acknowledge cleanup before any non-acoustic
    // baseline has been rebuilt. The disabled branch below requests the
    // rebuild on the next complete Nav2 pass, including after a reordering.
    RCLCPP_ERROR(logger_, "Acoustic layer disabled: no longer terminal in Nav2 plugin roster");
    return;
  }

  // Use the same clock domain as trusted ingress and updateCosts. A node
  // clock can differ from the layer clock during /clock activation or test
  // injection; mixing them would admit an expired or future staged overlay.
  if (!clock_) return;
  pending_stamp_ns_ = clock_->now().nanoseconds();
  if (pending_stamp_ns_ <= 0) return;
  pending_cycle_ = coordinator_.tick(pending_stamp_ns_, geometry);
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
  // A rejected costs callback has not completed Nav2's baseline rebuild.
  // Return its consumed bounds so a subsequent complete cycle can clear
  // previously written acoustic costs. The coordinator checks geometry
  // before replaying them (resize/rolling origin requires a full rebuild).
  const auto defer_rejected_cycle = [this]() {
    coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
  };
  // Nav2 may stall between bounds and costs. Never apply an expired staged
  // overlay, including after ROS simulation time jumps backwards.
  if (!clock_ || !pending_fresh(pending_stamp_ns_, clock_->now().nanoseconds())) {
    // updateBounds already consumed the dirty region. If this half-cycle
    // expires before updateCosts, Nav2 must revisit the same region on the
    // next full reset/rebuild pass; otherwise stale master costs can survive.
    defer_rejected_cycle();
    return;
  }
  const auto& g = pending_geometry_;
  if (g.width != geometry.width || g.height != geometry.height ||
      g.resolution != geometry.resolution ||
      g.origin_x != geometry.origin_x || g.origin_y != geometry.origin_y) {
    defer_rejected_cycle();
    return;
  }

  const auto* plugins = layered_costmap_->getPlugins();
  const auto* filters = layered_costmap_->getFilters();
  if (!AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
          plugins, filters, static_cast<const nav2_costmap_2d::Layer*>(this))) {
    defer_rejected_cycle();
    return;
  }

  // Without filters Nav2 passes combined_costmap_; with filters it passes
  // private primary_costmap_, then copies it and applies filters. Reject
  // unexpected buffers in either mode, without dereferencing stale pointers.
  const auto* combined = layered_costmap_->getCostmap();
  if (!combined || !filters) {
    defer_rejected_cycle();
    return;
  }
  if (filters->empty() ? (&master != combined) : (&master == combined)) {
    defer_rejected_cycle();
    return;
  }

  const auto count = static_cast<std::uint64_t>(geometry.width) * geometry.height;
  if (count == 0 || count > 4000000ULL) {
    defer_rejected_cycle();
    return;
  }
  AcousticNav2CycleAdapter::overlay_after_nav2_reset(
      pending_cycle_, geometry, min_i, min_j, max_i, max_j,
      master.getCharMap(), static_cast<std::size_t>(count), true);
}

void AcousticCostmapLayer::activate() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Nav2 may reactivate a layer after its subscriptions have queued old
  // observations. Never restore acoustic costs without operator opt-in and
  // a strictly newer verified generation.
  lifecycle_active_ = true;
  current_ = true;
}

void AcousticCostmapLayer::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  lifecycle_active_ = false;
  startup_enable_requested_ = false;
  enabled_ = false;
  coordinator_.disable();
  coordinator_.revoke_all();
  if (pending_valid_) coordinator_.defer_staged_cycle(pending_cycle_, pending_geometry_);
  pending_valid_ = false;
  last_metric_stamp_.clear();
  last_snapshot_stamp_ns_ = 0;
  quarantined_generation_ = last_metric_generation_ != 0;
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  current_ = false;
}

void AcousticCostmapLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  coordinator_.revoke_all();
  map_generation_.invalidate();
  baseline_gate_.invalidate();
  baseline_pipeline_.abort();
  pending_valid_ = false;
  // Reset is a rollback barrier, not authorization to replay pre-reset
  // observations. Preserve the last epoch number and quarantine it until a
  // strictly newer verified generation arrives. Fresh uninitialized layers
  // (generation zero) still admit their first valid snapshot normally.
  quarantined_generation_ = last_metric_generation_ != 0;
  last_snapshot_stamp_ns_ = 0;
  last_metric_stamp_.clear();
  current_ = true;
}
}  // namespace audition_nav2

PLUGINLIB_EXPORT_CLASS(audition_nav2::AcousticCostmapLayer, nav2_costmap_2d::Layer)
