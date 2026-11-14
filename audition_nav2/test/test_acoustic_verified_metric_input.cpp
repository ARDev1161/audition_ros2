#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include "audition_nav2/acoustic_verified_metric_input.hpp"
#include "audition_msgs/msg/localized_source.hpp"
#include "audition_msgs/msg/localized_source_array.hpp"

using audition_nav2::AcousticVerifiedMetricInput;
using audition_nav2::StampedMetricEvidence;
using audition_msgs::msg::LocalizedSource;
using audition_msgs::msg::LocalizedSourceArray;

static LocalizedSourceArray verified_array() {
  LocalizedSourceArray batch;
  batch.header.frame_id = "map";
  batch.header.stamp.sec = 10;
  batch.generation = 7;
  LocalizedSource s;
  s.header.frame_id = "map";
  s.header.stamp.sec = 10;
  s.last_observed.sec = 10;
  s.source_id = "speaker_A";
  s.source_generation = 3;
  s.localization_method = "independently_verified_metric_3d";
  s.position.x = 1.5;
  s.position.y = 2.5;
  s.position.z = 0.2;
  s.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  batch.sources.push_back(s);
  return batch;
}

TEST(VerifiedMetricInput, AcceptsOnlyFreshIndependentSameFrameMetricPosition) {
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> admitted;
  const std::int64_t now = 10100000000LL;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, admitted));
  ASSERT_EQ(admitted.size(), 1u);
  EXPECT_EQ(admitted.front().cell.x_m, 1.5);
  EXPECT_EQ(admitted.front().cell.y_m, 2.5);
  EXPECT_TRUE(admitted.front().cell.physically_validated);
  EXPECT_TRUE(admitted.front().independent_metric_confirmation);
  EXPECT_EQ(admitted.front().cell.expires_ns, 10500000000LL);
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "odom", now, admitted));
  EXPECT_EQ(admitted.size(), 1u);  // Failed preflight is non-destructive.
}

TEST(VerifiedMetricInput, RejectsDiagnosticUncertainAndUnstampedSources) {
  const std::int64_t now = 10100000000LL;
  const auto good = verified_array();
  std::vector<StampedMetricEvidence> output;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(good, "map", now, output));
  auto invalid = good;
  const char* forbidden[] = {
      "", "bearing_only", "av_pair", "diagnostic_intersection_2d",
      "unverified_metric", "triangulation"};
  for (const auto* method : forbidden) {
    invalid = good;
    invalid.sources[0].localization_method = method;
    EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
        invalid, "map", now, output));
    EXPECT_EQ(output.size(), 1u);
  }
  invalid = good;
  invalid.sources[0].source_generation = 0;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].header.frame_id = "camera_optical";
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].covariance[8] = 0.;  // No metric depth variance.
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].covariance[0] = 1e10;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].covariance[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].last_observed.sec = 9;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  invalid = good;
  invalid.sources[0].header.stamp.sec = 0;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      invalid, "map", now, output));
  EXPECT_EQ(output.size(), 1u);
}

TEST(VerifiedMetricInput, RejectsBatchOverflowDuplicatesAndClockSkew) {
  const std::int64_t now = 10100000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  batch.sources.push_back(batch.sources.front());
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_TRUE(output.empty());
  batch = verified_array();
  batch.sources.resize(33, batch.sources.front());
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch = verified_array();
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", 10300000000LL, output));  // Stale.
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", 9900000000LL, output));  // Future evidence.
  batch.header.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", now, output));
  batch = verified_array();
  batch.generation = 0;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", now, output));
}

TEST(VerifiedMetricInput, EmptyVerifiedBatchClearsAndIdsFollowGenerations) {
  const std::int64_t now = 10100000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  const auto first_id = output[0].cell.source_id;
  batch.sources[0].source_generation++;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_NE(output[0].cell.source_id, first_id);
  batch.sources.clear();
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_TRUE(output.empty());
}

TEST(VerifiedMetricInput, RejectsSameLogicalSourceAcrossGenerationsAtomically) {
  const std::int64_t now = 10100000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  const auto retained_id = output.front().cell.source_id;

  // A single logical source cannot simultaneously occupy two cells merely
  // by presenting two distinct source generations in one snapshot.
  auto second = batch.sources.front();
  second.source_generation += 1;
  second.position.x = 5.0;
  batch.sources.push_back(second);
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  EXPECT_EQ(output.front().cell.source_id, retained_id);

  // Distinct names may legitimately coexist in the same verified snapshot.
  batch.sources.back().source_id = "speaker_B";
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 2u);
  EXPECT_NE(output[0].cell.source_id, output[1].cell.source_id);

  // Duplicate logical names also fail when generations are identical.
  batch.sources.back().source_generation = batch.sources.front().source_generation;
  batch.sources.back().source_id = batch.sources.front().source_id;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 2u);
}

TEST(VerifiedMetricInput, RejectsAdversarialTimeAndCovarianceWithoutPartialCommit) {
  constexpr std::int64_t now = 10250000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  const auto retained = output.front().cell.source_id;

  // A valid first source must not leak through if a later source fails.
  auto second = batch.sources.front();
  second.source_id = "speaker_B";
  second.position.x = 4.0;
  batch.sources.push_back(second);
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 2u);
  batch.sources.back().covariance[1] = 0.19;
  batch.sources.back().covariance[3] = 0.19;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 2u);
  EXPECT_EQ(output.front().cell.source_id, retained);

  // Off-diagonal covariance can be symmetric yet not positive definite.
  batch.sources.back().covariance[1] = 0.0;
  batch.sources.back().covariance[3] = 0.0;
  batch.sources.back().covariance[0] = -0.01;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.back().covariance[0] = 0.04;

  // All three timestamps must be valid and in the declared clock domain.
  batch.sources.back().last_observed.nanosec = 1000000000u;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.back().last_observed.nanosec = 0;
  batch.sources.back().header.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.back().header.stamp.nanosec = 0;
  batch.sources.back().header.stamp.sec = 9;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.back().header.stamp.sec = 10;

  // No silent acceptance of a source newer than the enclosing snapshot.
  batch.sources.back().last_observed.sec = 11;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.back().last_observed.sec = 10;

  // A single out-of-frame source invalidates the entire snapshot.
  batch.sources.back().header.frame_id = "odom";
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 2u);
  batch.sources.back().header.frame_id = "map";
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 2u);
}

TEST(VerifiedMetricInput, EnforcesExactFreshnessBoundaryAndAtomicRecovery) {
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  constexpr std::int64_t stamp = 10000000000LL;
  constexpr std::int64_t max_age = AcousticVerifiedMetricInput::kMaxAgeNs;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(
      batch, "map", stamp + max_age, output));
  ASSERT_EQ(output.size(), 1u);
  const auto id = output.front().cell.source_id;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", stamp + max_age + 1, output));
  ASSERT_EQ(output.size(), 1u);
  EXPECT_EQ(output.front().cell.source_id, id);

  // Future-dated snapshots must fail even by one nanosecond.
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      batch, "map", stamp - 1, output));
  EXPECT_EQ(output.size(), 1u);

  // A fresh empty snapshot is an explicit revocation, not a parse failure.
  batch.sources.clear();
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(
      batch, "map", stamp + max_age, output));
  EXPECT_TRUE(output.empty());
  batch = verified_array();
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(
      batch, "map", stamp + max_age, output));
  ASSERT_EQ(output.size(), 1u);
  EXPECT_EQ(output.front().cell.source_id, id);
}


TEST(VerifiedMetricInput, CapacityBoundaryAndLateInvalidSourceAreAtomic) {
  constexpr std::int64_t now = 10100000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  for (unsigned i = 1; i < AcousticVerifiedMetricInput::kMaxSources; ++i) {
    auto source = batch.sources.front();
    source.source_id = "speaker_" + std::to_string(i);
    source.position.x = 1.5 + static_cast<double>(i) * 0.01;
    batch.sources.push_back(source);
  }
  ASSERT_EQ(batch.sources.size(), AcousticVerifiedMetricInput::kMaxSources);
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), AcousticVerifiedMetricInput::kMaxSources);
  const auto original = output.front().cell.source_id;

  // A late malformed entry must not publish the valid prefix, even when
  // the batch is at the exact configured capacity.
  auto malformed = batch;
  malformed.sources.back().localization_method = "diagnostic_intersection_2d";
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      malformed, "map", now, output));
  ASSERT_EQ(output.size(), AcousticVerifiedMetricInput::kMaxSources);
  EXPECT_EQ(output.front().cell.source_id, original);

  malformed = batch;
  malformed.sources.back().position.z =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      malformed, "map", now, output));
  EXPECT_EQ(output.size(), AcousticVerifiedMetricInput::kMaxSources);

  malformed = batch;
  malformed.sources.push_back(batch.sources.front());
  malformed.sources.back().source_id = "overflow";
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(
      malformed, "map", now, output));
  EXPECT_EQ(output.size(), AcousticVerifiedMetricInput::kMaxSources);

  // A subsequent valid snapshot is still accepted after each rejection.
  batch.sources.back().position.x += 0.5;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(
      batch, "map", now, output));
  ASSERT_EQ(output.size(), AcousticVerifiedMetricInput::kMaxSources);
  EXPECT_DOUBLE_EQ(output.back().cell.x_m, batch.sources.back().position.x);
}

TEST(VerifiedMetricInput, RejectsLateFrameAndObservationSkewWithoutMutation) {
  constexpr std::int64_t now = 10100000000LL;
  auto batch = verified_array();
  std::vector<StampedMetricEvidence> output;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  const auto source_id = output.front().cell.source_id;

  // A 50 ms discrepancy between source header and observation is allowed;
  // 50 ms + 1 ns is not. The surrounding snapshot remains fresh.
  batch.sources.front().header.stamp.nanosec = 50000000u;
  ASSERT_TRUE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  batch.sources.front().header.stamp.nanosec = 50000001u;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  ASSERT_EQ(output.size(), 1u);
  EXPECT_EQ(output.front().cell.source_id, source_id);

  batch = verified_array();
  batch.sources.front().last_observed.nanosec = 1u;
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 1u);
  batch = verified_array();
  batch.header.frame_id = "odom";
  EXPECT_FALSE(AcousticVerifiedMetricInput::decode(batch, "map", now, output));
  EXPECT_EQ(output.size(), 1u);
}
