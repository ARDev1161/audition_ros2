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
  coordinator.disable();
  bad = e; bad.cell.source_id = 8;
  assert(!coordinator.ingest_evidence(bad, now));
}
