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
