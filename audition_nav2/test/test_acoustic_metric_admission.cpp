#include "audition_nav2/acoustic_metric_admission.hpp"
#include "audition_nav2/acoustic_layer_coordinator.hpp"
#include <cassert>
#include <cstdint>
#include <limits>
using namespace audition_nav2;
int main() {
  const std::int64_t now = 1000000000LL;
  StampedMetricEvidence e{{7, now - 10000000LL, now + 100000000LL,
                           2.0, 3.0, true},
      MetricProvenance::IndependentlyVerifiedMetric,
      now - 10000000LL, 50000000LL, true, true, true};
  assert(AcousticMetricAdmission::admit(e, now));
  AcousticLayerCoordinator coordinator;
  coordinator.enable();
  assert(coordinator.ingest_evidence(e, now));
  assert(coordinator.source_count() == 1);
  for (auto provenance : {MetricProvenance::BearingOnly,
                          MetricProvenance::AvAssociation,
                          MetricProvenance::DiagnosticIntersection2D,
                          MetricProvenance::UnverifiedMetric}) {
    auto rejected = e;
    rejected.cell.source_id++;
    rejected.provenance = provenance;
    assert(!AcousticMetricAdmission::admit(rejected, now));
    assert(!coordinator.ingest_evidence(rejected, now));
  }
  auto bad = e;
  bad.transform_lookup_succeeded = false;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.frame_matches_costmap = false;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.independent_metric_confirmation = false;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.transform_stamp_ns = now - 90000000LL;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.transform_stamp_ns = now + 1;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.transform_stamp_ns = 1;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.cell.stamp_ns = now - 90000000LL;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.cell.x_m = std::numeric_limits<double>::infinity();
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.transform_age_limit_ns = 0;
  assert(!AcousticMetricAdmission::admit(bad, now));
  bad = e; bad.cell.stamp_ns = std::numeric_limits<std::int64_t>::min();
  assert(!AcousticMetricAdmission::admit(bad, now));
  assert(!AcousticMetricAdmission::admit(e, 0));
  // An untrusted caller must not defeat TF freshness with a huge age limit.
  bad = e; bad.transform_age_limit_ns = std::numeric_limits<std::int64_t>::max();
  bad.transform_stamp_ns = now - 300000000LL;
  bad.cell.stamp_ns = bad.transform_stamp_ns;
  assert(!AcousticMetricAdmission::admit(bad, now));
  // Even a fresh TF cannot be reused for a much older observation.
  bad = e; bad.transform_age_limit_ns = std::numeric_limits<std::int64_t>::max();
  bad.cell.stamp_ns = now - 100000000LL;
  assert(!AcousticMetricAdmission::admit(bad, now));
  // Boundary conditions: exactly 50ms skew is permitted; >50ms is not.
  bad = e; bad.transform_age_limit_ns = 250000000LL;
  bad.cell.stamp_ns = now - 60000000LL;
  assert(AcousticMetricAdmission::admit(bad, now));
  bad.cell.stamp_ns--;
  assert(!AcousticMetricAdmission::admit(bad, now));
  // Delayed invalidation must not erase a newer independently verified source.
  auto fresh = e;
  fresh.cell.source_id = 42;
  fresh.cell.stamp_ns = now - 1000000LL;
  fresh.transform_stamp_ns = fresh.cell.stamp_ns;
  assert(coordinator.ingest_evidence(fresh, now));
  assert(coordinator.source_count() == 2);
  auto delayed = fresh;
  delayed.cell.stamp_ns -= 1000000LL;
  delayed.transform_stamp_ns = delayed.cell.stamp_ns;
  delayed.transform_lookup_succeeded = false;
  assert(!coordinator.ingest_evidence(delayed, now));
  assert(coordinator.source_count() == 2);
  // A current invalidation revokes ownership, while preserving the replay
  // watermark so the old valid packet cannot restore a cost.
  auto invalidated = fresh;
  invalidated.transform_lookup_succeeded = false;
  assert(!coordinator.ingest_evidence(invalidated, now));
  assert(coordinator.source_count() == 1);
  assert(!coordinator.ingest_evidence(fresh, now));
  auto renewed = fresh;
  renewed.cell.stamp_ns += 1;
  renewed.transform_stamp_ns = renewed.cell.stamp_ns;
  assert(coordinator.ingest_evidence(renewed, now));
  assert(coordinator.source_count() == 2);
  coordinator.disable();
  bad = e; bad.cell.source_id = 8;
  assert(!coordinator.ingest_evidence(bad, now));
}
