#pragma once
// M5 staging contract for a future Nav2 plugin. No master-map ownership.
// The caller MUST rebuild the dirty master region from non-acoustic layers
// before overlaying own_cells; stale costs must never be erased in-place.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_cell_buffer.hpp"
#include "audition_nav2/acoustic_metric_admission.hpp"
#include "audition_nav2/acoustic_dirty_bounds.hpp"
#include "audition_nav2/acoustic_master_merge.hpp"

namespace audition_nav2 {
struct AcousticCycle {
  LayerUpdate update;
  CellBounds dirty;
  bool rebuild_required = false;
};
class AcousticLayerCoordinator {
 public:
  explicit AcousticLayerCoordinator(std::size_t max_sources = 32,
                                    std::uint8_t soft_cost = 100)
      : buffer_(max_sources, soft_cost) {}
  // Caller owns authorization and timestamped TF: no bearing-only path.
  bool ingest_validated(const MetricCell& observation, std::int64_t now_ns) {
    return enabled_ && buffer_.ingest(observation, now_ns);
  }
  // Subscriber-facing API: provenance and stamped TF are mandatory.
  bool ingest_evidence(const StampedMetricEvidence& evidence, std::int64_t now_ns) {
    if (!enabled_) return false;
    if (!AcousticMetricAdmission::admit(evidence, now_ns)) {
      // An explicit invalidation of a known source revokes its previous
      // ownership immediately, without releasing its anti-replay watermark.
      // This is intentionally conservative: an invalid TF or lost metric
      // confirmation must never leave the previous cost active until TTL.
      // A rejected packet is not necessarily a trustworthy revocation.
      // Never accept future-dated or clock-invalid invalidations: those
      // could advance a replay barrier beyond the receiver's current time.
      // Explicit source revocation still requires an ordered timestamp.
      // A bearing-only or unverified packet must not impersonate a metric
      // source and revoke its independently confirmed ownership. Only a
      // a metric-provenance stream can report TF/metric loss. The adapter
      // must authenticate that provenance independently of this struct.
      if (evidence.provenance == MetricProvenance::IndependentlyVerifiedMetric &&
          now_ns > 0 && evidence.cell.source_id != 0 &&
          evidence.cell.stamp_ns > 0 && evidence.cell.stamp_ns <= now_ns) {
        buffer_.revoke_if_not_older(evidence.cell.source_id,
                                    evidence.cell.stamp_ns);
      }
      return false;
    }
    return buffer_.ingest(evidence.cell, now_ns);
  }
  void disable() { enabled_ = false; buffer_.revoke_all(); }
  void enable() { enabled_ = true; }
  void revoke_all() { buffer_.revoke_all(); }
  void expire_without_tick(std::int64_t now_ns) {
    buffer_.expire_without_tick(now_ns);
  }
  void revoke(std::uint64_t id) { buffer_.revoke(id); }
  std::size_t source_count() const { return buffer_.source_count(); }
  // Requeue a staged bounds region if ingress invalidates the half-cycle.
  // tick() already advances ownership, so without this a revoked cost could
  // survive forever when no further source changes occur.
  void defer_staged_cycle(const AcousticCycle& cycle,
                          const GridGeometry& geometry) {
    if (!cycle.rebuild_required) return;
    if (!deferred_valid_ || !same_geometry(deferred_geometry_, geometry)) {
      deferred_dirty_ = cycle.dirty;
    } else {
      deferred_dirty_.min_i = std::min(deferred_dirty_.min_i, cycle.dirty.min_i);
      deferred_dirty_.min_j = std::min(deferred_dirty_.min_j, cycle.dirty.min_j);
      deferred_dirty_.max_i = std::max(deferred_dirty_.max_i, cycle.dirty.max_i);
      deferred_dirty_.max_j = std::max(deferred_dirty_.max_j, cycle.dirty.max_j);
    }
    deferred_geometry_ = geometry;
    deferred_valid_ = true;
  }
  // Called on a timer regardless of incoming messages.
  AcousticCycle tick(std::int64_t now_ns, const GridGeometry& geometry) {
    AcousticCycle cycle;
    cycle.update = buffer_.tick(now_ns, geometry);
    cycle.dirty = AcousticDirtyBounds::compute(cycle.update, geometry);
    if (deferred_valid_) {
      if (same_geometry(deferred_geometry_, geometry) &&
          !deferred_dirty_.empty()) {
        if (cycle.dirty.empty()) cycle.dirty = deferred_dirty_;
        else {
          cycle.dirty.min_i = std::min(cycle.dirty.min_i, deferred_dirty_.min_i);
          cycle.dirty.min_j = std::min(cycle.dirty.min_j, deferred_dirty_.min_j);
          cycle.dirty.max_i = std::max(cycle.dirty.max_i, deferred_dirty_.max_i);
          cycle.dirty.max_j = std::max(cycle.dirty.max_j, deferred_dirty_.max_j);
        }
      }
      deferred_valid_ = false;
      deferred_dirty_ = {};
    }
    cycle.rebuild_required = !cycle.dirty.empty();
    return cycle;
  }
  // A Nav2 adapter may call this ONLY after the master region has been reset
  // from other layers; do not use this method as an in-place clear operation.
  static std::size_t overlay_after_rebuild(
      const AcousticCycle& cycle, const GridGeometry& geometry,
      unsigned min_i, unsigned min_j, unsigned max_i, unsigned max_j,
      std::uint8_t* master, std::size_t master_size) {
    if (!cycle.rebuild_required) return 0;
    return AcousticMasterMerge::apply(cycle.update, geometry, min_i, min_j,
                                     max_i, max_j, master, master_size);
  }
 private:
  static bool same_geometry(const GridGeometry& a, const GridGeometry& b) {
    return a.width == b.width && a.height == b.height &&
           a.resolution == b.resolution && a.origin_x == b.origin_x &&
           a.origin_y == b.origin_y;
  }
  CellBounds deferred_dirty_{};
  GridGeometry deferred_geometry_{};
  bool deferred_valid_ = false;
  AcousticCellBuffer buffer_;
  // Optional acoustic navigation is fail-closed until explicitly authorized.
  bool enabled_ = false;
};
}  // namespace audition_nav2
