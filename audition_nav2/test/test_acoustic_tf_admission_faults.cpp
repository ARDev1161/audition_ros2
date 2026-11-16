#include <cassert>
#include <cstdint>
#include <limits>
#include "audition_nav2/acoustic_metric_admission.hpp"
#include "audition_nav2/acoustic_layer_coordinator.hpp"

// ROS-independent, deterministic fault injection for the M5 metric/TF
// admission boundary. This does NOT claim a real tf2::Buffer lookup.
using namespace audition_nav2;

static StampedMetricEvidence verified(std::int64_t observed,
                                      std::int64_t transform) {
  StampedMetricEvidence e{};
  e.cell = {17, observed, observed + 500000000LL, 1.25, 2.25, true};
  e.provenance = MetricProvenance::IndependentlyVerifiedMetric;
  e.transform_stamp_ns = transform;
  e.transform_age_limit_ns = AcousticMetricAdmission::kMaxTransformAgeNs;
  e.transform_lookup_succeeded = true;
  e.frame_matches_costmap = true;
  e.independent_metric_confirmation = true;
  return e;
}

int main() {
  constexpr std::int64_t second = 1000000000LL;
  constexpr std::int64_t observed = 10 * second;
  constexpr std::int64_t max_tf_age =
      AcousticMetricAdmission::kMaxTransformAgeNs;
  constexpr std::int64_t max_skew =
      AcousticMetricAdmission::kMaxStampSkewNs;
  const auto now = observed + 100000000LL;
  const auto original = verified(observed, observed);
  assert(AcousticMetricAdmission::admit(original, now));

  // An unverified transform cannot be inferred from matching frame names,
  // nor can an otherwise fresh transform authorize a mismatched frame.
  auto e = original;
  e.transform_lookup_succeeded = false;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.frame_matches_costmap = false;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.independent_metric_confirmation = false;
  assert(!AcousticMetricAdmission::admit(e, now));

  // Every non-metric upstream hypothesis is forbidden even with a fresh TF.
  for (const auto p : {MetricProvenance::BearingOnly,
                       MetricProvenance::AvAssociation,
                       MetricProvenance::DiagnosticIntersection2D,
                       MetricProvenance::UnverifiedMetric}) {
    e = original;
    e.provenance = p;
    assert(!AcousticMetricAdmission::admit(e, now));
  }

  // Exact TF freshness and observation/TF skew boundaries, in nanoseconds.
  e = verified(observed, observed + max_skew);
  assert(AcousticMetricAdmission::admit(e, now));
  e.transform_stamp_ns += 1;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = verified(observed, observed - max_skew);
  assert(AcousticMetricAdmission::admit(e, now));
  e.transform_stamp_ns -= 1;
  assert(!AcousticMetricAdmission::admit(e, now));

  e = verified(observed, observed);
  assert(AcousticMetricAdmission::admit(e, observed + max_tf_age));
  assert(!AcousticMetricAdmission::admit(e, observed + max_tf_age + 1));
  // A caller cannot expand the hard freshness ceiling.
  e.transform_age_limit_ns = 10 * second;
  assert(!AcousticMetricAdmission::admit(e, observed + max_tf_age + 1));
  e.transform_age_limit_ns = 50000000LL;
  assert(!AcousticMetricAdmission::admit(e, observed + 50000001LL));
  assert(AcousticMetricAdmission::admit(e, observed + 50000000LL));

  e = original;
  e.transform_stamp_ns = now + 1;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.cell.stamp_ns = now + 1;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.transform_stamp_ns = 0;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.transform_age_limit_ns = 0;
  assert(!AcousticMetricAdmission::admit(e, now));
  assert(!AcousticMetricAdmission::admit(original, 0));

  // Fail closed on invalid metric geometry and a fabricated validation bit.
  // Matching frame names or successful lookup cannot repair bad coordinates.
  e = original;
  e.cell.physically_validated = false;
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.cell.x_m = std::numeric_limits<double>::quiet_NaN();
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.cell.y_m = std::numeric_limits<double>::infinity();
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.cell.x_m = -std::numeric_limits<double>::infinity();
  assert(!AcousticMetricAdmission::admit(e, now));

  // Clock-domain discontinuities and signed 64-bit extremes must not
  // overflow age/skew arithmetic or accidentally authorize a stale TF.
  e = original;
  e.transform_stamp_ns = std::numeric_limits<std::int64_t>::min();
  assert(!AcousticMetricAdmission::admit(e, now));
  e = original;
  e.cell.stamp_ns = std::numeric_limits<std::int64_t>::min();
  assert(!AcousticMetricAdmission::admit(e, now));
  e = verified(std::numeric_limits<std::int64_t>::max() - 100000000LL,
               std::numeric_limits<std::int64_t>::max() - 100000000LL);
  assert(AcousticMetricAdmission::admit(
      e, std::numeric_limits<std::int64_t>::max()));
  e.transform_stamp_ns = 1;
  assert(!AcousticMetricAdmission::admit(
      e, std::numeric_limits<std::int64_t>::max()));
  e = original;
  assert(!AcousticMetricAdmission::admit(e, observed - 1));
  assert(!AcousticMetricAdmission::admit(e, -1));

  // A rejected stale transform must never reach the live costmap buffer.
  AcousticLayerCoordinator coordinator;
  coordinator.enable();
  const GridGeometry grid{16, 16, 0.5, 0.0, 0.0};
  e = original;
  e.transform_lookup_succeeded = false;
  assert(!coordinator.ingest_evidence(e, now));
  assert(coordinator.source_count() == 0);
  auto cycle = coordinator.tick(now, grid);
  assert(cycle.update.own_cells.empty());

  // A new, independently verified observation may recover; its ownership
  // then expires even if no new messages arrive.
  e = original;
  assert(coordinator.ingest_evidence(e, now));
  assert(coordinator.source_count() == 1);
  cycle = coordinator.tick(now, grid);
  assert(!cycle.update.own_cells.empty());
  coordinator.expire_without_tick(observed + 500000001LL);
  assert(coordinator.source_count() == 0);
  cycle = coordinator.tick(observed + 500000001LL, grid);
  assert(cycle.update.own_cells.empty());
  assert(cycle.rebuild_required);
  return 0;
}
