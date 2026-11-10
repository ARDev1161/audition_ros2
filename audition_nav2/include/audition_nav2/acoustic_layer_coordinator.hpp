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
    return enabled_ && AcousticMetricAdmission::admit(evidence, now_ns) &&
           buffer_.ingest(evidence.cell, now_ns);
  }
  void disable() { enabled_ = false; buffer_.revoke_all(); }
  void enable() { enabled_ = true; }
  void revoke_all() { buffer_.revoke_all(); }
  void revoke(std::uint64_t id) { buffer_.revoke(id); }
  std::size_t source_count() const { return buffer_.source_count(); }
  // Called on a timer regardless of incoming messages.
  AcousticCycle tick(std::int64_t now_ns, const GridGeometry& geometry) {
    AcousticCycle cycle;
    cycle.update = buffer_.tick(now_ns, geometry);
    cycle.dirty = AcousticDirtyBounds::compute(cycle.update, geometry);
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
  AcousticCellBuffer buffer_;
  // Optional acoustic navigation is fail-closed until explicitly authorized.
  bool enabled_ = false;
};
}  // namespace audition_nav2
