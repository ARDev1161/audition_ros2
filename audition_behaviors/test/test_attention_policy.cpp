#include "audition_behaviors/attention_policy.hpp"

#include <gtest/gtest.h>

using audition_behaviors::AttentionPolicy;
using audition_msgs::msg::AcousticScene;
using audition_msgs::msg::AcousticSource;
using namespace std::chrono_literals;

static AcousticSource source(const char * id, float score, std::uint64_t generation = 1) {
  AcousticSource s;
  s.source_id = id;
  s.source_generation = generation;
  s.header.frame_id = "mic";
  s.state = AcousticSource::ACTIVE;
  s.has_direction = true;
  s.direction_unit.x = 1.0;
  s.activity_score = score;
  return s;
}
static AcousticScene scene(std::uint64_t generation = 1) {
  AcousticScene s;
  s.header.frame_id = "mic";
  s.generation = generation;
  return s;
}
TEST(AttentionPolicy, SelectAndHysteresis) {
  AttentionPolicy p({});
  auto s = scene();
  s.sources = {source("a", 0.7f), source("b", 0.6f)};
  auto now = AttentionPolicy::Clock::now();
  EXPECT_EQ(p.observe(s, now).source_id, "a");
  s.sources[1].activity_score = 0.8f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
}
TEST(AttentionPolicy, InvalidAndStaleAreNotActionable) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, now).valid);
  EXPECT_FALSE(p.expire(now + 1501ms).valid);
  EXPECT_FALSE(p.expire(now + 1600ms).valid);
  s.sources[0].has_direction = false;
  EXPECT_FALSE(p.observe(s, now + 1700ms).valid);
}
TEST(AttentionPolicy, GenerationAndFrameFence) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, now).valid);
  s.generation = 2;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources = {source("b", 0.8f, 2)};
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
  s.header.frame_id = "other";
  EXPECT_FALSE(p.observe(s, now + 300ms).valid);
}
TEST(AttentionPolicy, RejectMalformedBearingAndEnded) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  auto a = source("a", 0.9f);
  a.direction_unit.x = 0.0;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, now).valid);
  a = source("a", 0.9f);
  a.state = AcousticSource::ENDED;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
}
TEST(AttentionPolicy, ConfigValidation) {
  AttentionPolicy::Config c;
  c.scene_timeout = 0ms;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
  c.scene_timeout = 1500ms;
  c.switch_margin = -0.1;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
}

TEST(AttentionPolicy, RejectDelayedGenerationWithoutReactivatingOldSource) {
  AttentionPolicy p({});
  auto now = AttentionPolicy::Clock::now();
  auto newer = scene(2);
  newer.sources = {source("new", 0.8f, 2)};
  EXPECT_TRUE(p.observe(newer, now).valid);
  auto old = scene(1);
  old.sources = {source("old", 0.99f)};
  auto rejected = p.observe(old, now + 100ms);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(rejected.reason, "regressed_generation");
  EXPECT_EQ(p.expire(now + 200ms).source_id, "new");
}
TEST(AttentionPolicy, RejectOutOfOrderTimestampWithoutExtendingFreshness) {
  AttentionPolicy p({});
  auto now = AttentionPolicy::Clock::now();
  auto s = scene();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.8f)};
  EXPECT_TRUE(p.observe(s, now).valid);
  s.header.stamp.sec = 9;
  s.sources = {source("b", 0.99f)};
  EXPECT_EQ(p.observe(s, now + 1000ms).reason, "regressed_scene_stamp");
  EXPECT_FALSE(p.expire(now + 1501ms).valid);
}
TEST(AttentionPolicy, DuplicateSourceIdsCannotWinSelection) {
  AttentionPolicy p({});
  auto s = scene();
  s.sources = {source("duplicate", 0.99f), source("duplicate", 0.9f),
               source("unique", 0.5f)};
  EXPECT_EQ(p.observe(s, AttentionPolicy::Clock::now()).source_id, "unique");
  s.sources.pop_back();
  EXPECT_FALSE(p.observe(s, AttentionPolicy::Clock::now()).valid);
}
TEST(AttentionPolicy, SameGenerationFrameChangeResetsIncumbent) {
  AttentionPolicy p({});
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.8f), source("b", 0.7f)};
  EXPECT_EQ(p.observe(s, now).source_id, "a");
  s.header.frame_id = "new_mic";
  for (auto & item : s.sources) item.header.frame_id = "new_mic";
  s.sources[0].activity_score = 0.6f;
  s.sources[1].activity_score = 0.7f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "b");
}


TEST(AttentionPolicy, ConfirmChallengerBeforeSwitching) {
  AttentionPolicy::Config config;
  config.switch_confirmations = 3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.7f), source("b", 0.5f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "a");
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "b");
}
TEST(AttentionPolicy, InterruptedChallengerMustReconfirm) {
  AttentionPolicy::Config config;
  config.switch_confirmations = 2;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.7f), source("b", 0.5f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  s.sources[1].activity_score = 0.6f;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "a");
  EXPECT_EQ(p.observe(s, now + 400ms).source_id, "b");
}
TEST(AttentionPolicy, ConfirmationWindowAndRegressedScene) {
  AttentionPolicy::Config config;
  config.switch_confirmations = 2;
  config.switch_confirmation_window = 200ms;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.7f), source("b", 0.5f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  s.header.stamp.sec = 9;
  EXPECT_EQ(p.observe(s, now + 150ms).reason, "regressed_scene_stamp");
  s.header.stamp.sec = 11;
  EXPECT_EQ(p.observe(s, now + 400ms).source_id, "a");
  EXPECT_EQ(p.observe(s, now + 450ms).source_id, "b");
}
TEST(AttentionPolicy, ConfirmationConfigValidation) {
  AttentionPolicy::Config c;
  c.switch_confirmations = 0;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
  c.switch_confirmations = 101;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
  c.switch_confirmations = 2;
  c.switch_confirmation_window = 0ms;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
}

TEST(AttentionPolicy, AngularUncertaintyGateRejectsMissingAndUncertainBearings) {
  AttentionPolicy::Config config;
  config.maximum_angular_variance_rad2 = 0.05;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  auto a = source("a", 0.9f);
  auto b = source("b", 0.6f);
  b.has_angular_variance = true;
  b.angular_variance_rad2 = 0.02;
  s.sources = {a, b};
  EXPECT_EQ(p.observe(s, now).source_id, "b");
  s.sources[0].has_angular_variance = true;
  s.sources[0].angular_variance_rad2 = 0.06;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.sources[0].angular_variance_rad2 = 0.01;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "a");
}
TEST(AttentionPolicy, AngularUncertaintyGateRejectsMalformedVariance) {
  AttentionPolicy::Config config;
  config.maximum_angular_variance_rad2 = 0.05;
  AttentionPolicy p(config);
  auto s = scene();
  auto a = source("a", 0.9f);
  a.has_angular_variance = true;
  a.angular_variance_rad2 = -1.0;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, AttentionPolicy::Clock::now()).valid);
  s.sources[0].angular_variance_rad2 = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(p.observe(s, AttentionPolicy::Clock::now()).valid);
  s.sources[0].angular_variance_rad2 = 0.05;
  EXPECT_TRUE(p.observe(s, AttentionPolicy::Clock::now()).valid);
}
TEST(AttentionPolicy, DisabledAngularGateAcceptsMissingVariance) {
  AttentionPolicy p({});
  auto s = scene();
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, AttentionPolicy::Clock::now()).valid);
}
TEST(AttentionPolicy, AngularGateConfigValidation) {
  AttentionPolicy::Config c;
  c.maximum_angular_variance_rad2 = -0.01;
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
  c.maximum_angular_variance_rad2 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(AttentionPolicy p(c), std::invalid_argument);
}

TEST(AttentionPolicy, SourceAgeGateFiltersStaleSources) {
  AttentionPolicy::Config config;
  config.maximum_source_age = 200ms;
  AttentionPolicy p(config);
  auto s = scene();
  s.header.stamp.sec = 10;
  s.header.stamp.nanosec = 100000000;
  auto stale = source("stale", 0.99f);
  stale.last_seen.sec = 9;
  stale.last_seen.nanosec = 500000000;
  auto fresh = source("fresh", 0.6f);
  fresh.last_seen.sec = 10;
  fresh.last_seen.nanosec = 0;
  s.sources = {stale, fresh};
  EXPECT_EQ(p.observe(s, AttentionPolicy::Clock::now()).source_id, "fresh");
}
TEST(AttentionPolicy, SourceAgeGateRejectsFutureAndMalformedStamps) {
  AttentionPolicy::Config config;
  config.maximum_source_age = 200ms;
  AttentionPolicy p(config);
  auto s = scene();
  s.header.stamp.sec = 10;
  auto a = source("a", 0.9f);
  a.last_seen.sec = 11;
  s.sources = {a};
  EXPECT_FALSE(p.observe(s, AttentionPolicy::Clock::now()).valid);
  s.sources[0].last_seen.sec = 10;
  s.sources[0].last_seen.nanosec = 1000000000u;
  EXPECT_FALSE(p.observe(s, AttentionPolicy::Clock::now()).valid);
  s.sources[0].last_seen.nanosec = 0;
  EXPECT_TRUE(p.observe(s, AttentionPolicy::Clock::now()).valid);
}
TEST(AttentionPolicy, SourceAgeGateRejectsMissingLastSeenButDefaultDoesNot) {
  auto s = scene();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.9f)};
  AttentionPolicy default_policy({});
  EXPECT_TRUE(default_policy.observe(s, AttentionPolicy::Clock::now()).valid);
  AttentionPolicy::Config config;
  config.maximum_source_age = 200ms;
  AttentionPolicy strict(config);
  EXPECT_FALSE(strict.observe(s, AttentionPolicy::Clock::now()).valid);
}
TEST(AttentionPolicy, SourceAgeConfigValidation) {
  AttentionPolicy::Config config;
  config.maximum_source_age = -1ms;
  EXPECT_THROW(AttentionPolicy p(config), std::invalid_argument);
}

TEST(AttentionPolicy, DirectionJumpGateRejectsAbruptSameIdentityChange) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  EXPECT_FALSE(p.expire(now + 150ms).valid);
}
TEST(AttentionPolicy, DirectionJumpGateAllowsSmallChangeAndNewIdentity) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = std::cos(0.2);
  s.sources[0].direction_unit.y = std::sin(0.2);
  EXPECT_TRUE(p.observe(s, now + 100ms).valid);
  s.sources = {source("b", 0.8f)};
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
}
TEST(AttentionPolicy, DirectionJumpGateResetsOnNewGeneration) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s = scene(2);
  s.sources = {source("a", 0.9f, 2)};
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_TRUE(p.observe(s, now + 100ms).valid);
}
TEST(AttentionPolicy, DirectionJumpGateDisabledAndInvalidConfig) {
  auto s = scene();
  s.sources = {source("a", 0.9f)};
  AttentionPolicy p({});
  auto now = AttentionPolicy::Clock::now();
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = -1.0;
  EXPECT_TRUE(p.observe(s, now + 100ms).valid);
  AttentionPolicy::Config c;
  c.maximum_direction_jump_rad = -0.1;
  EXPECT_THROW(AttentionPolicy invalid(c), std::invalid_argument);
  c.maximum_direction_jump_rad = 3.2;
  EXPECT_THROW(AttentionPolicy invalid(c), std::invalid_argument);
}

TEST(AttentionPolicy, StrictSceneStampRejectsDuplicateWithoutRefreshingTimeout) {
  AttentionPolicy::Config config;
  config.require_advancing_scene_stamp = true;
  AttentionPolicy p(config);
  auto s = scene();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.9f)};
  auto now = AttentionPolicy::Clock::now();
  ASSERT_TRUE(p.observe(s, now).valid);
  EXPECT_EQ(p.observe(s, now + 1000ms).reason, "regressed_scene_stamp");
  EXPECT_FALSE(p.expire(now + 1501ms).valid);
}
TEST(AttentionPolicy, StrictSceneStampPreventsReplayConfirmations) {
  AttentionPolicy::Config config;
  config.require_advancing_scene_stamp = true;
  config.switch_confirmations = 2;
  AttentionPolicy p(config);
  auto s = scene();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.7f), source("b", 0.5f)};
  auto now = AttentionPolicy::Clock::now();
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.header.stamp.sec = 11;
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "a");
  EXPECT_EQ(p.observe(s, now + 200ms).reason, "regressed_scene_stamp");
  s.header.stamp.sec = 12;
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "b");
}
TEST(AttentionPolicy, StrictSceneStampRejectsMalformedAndAcceptsGenerationReset) {
  AttentionPolicy::Config config;
  config.require_advancing_scene_stamp = true;
  AttentionPolicy p(config);
  auto s = scene();
  s.sources = {source("a", 0.9f)};
  auto now = AttentionPolicy::Clock::now();
  EXPECT_EQ(p.observe(s, now).reason, "invalid_scene_stamp");
  s.header.stamp.sec = 10;
  EXPECT_TRUE(p.observe(s, now + 100ms).valid);
  s.header.stamp.nanosec = 1000000000u;
  EXPECT_EQ(p.observe(s, now + 200ms).reason, "invalid_scene_stamp");
  s = scene(2);
  s.header.stamp.sec = 1;
  s.sources = {source("b", 0.9f, 2)};
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "b");
}
TEST(AttentionPolicy, DefaultAllowsEqualSceneStampsForCompatibility) {
  AttentionPolicy p({});
  auto s = scene();
  s.sources = {source("a", 0.9f)};
  auto now = AttentionPolicy::Clock::now();
  EXPECT_TRUE(p.observe(s, now).valid);
  EXPECT_TRUE(p.observe(s, now + 100ms).valid);
}

TEST(AttentionPolicy, TimeoutRetainsGenerationFenceAndAllowsFreshRecovery) {
  AttentionPolicy::Config config;
  config.require_advancing_scene_stamp = true;
  config.scene_timeout = 200ms;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(7);
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.9f, 7)};
  ASSERT_TRUE(p.observe(s, now).valid);
  EXPECT_EQ(p.expire(now + 201ms).reason, "scene_timeout");
  auto old = scene(6);
  old.header.stamp.sec = 50;
  old.sources = {source("old", 0.99f, 6)};
  EXPECT_EQ(p.observe(old, now + 210ms).reason, "regressed_generation");
  EXPECT_FALSE(p.expire(now + 220ms).valid);
  EXPECT_EQ(p.observe(s, now + 230ms).reason, "regressed_scene_stamp");
  s.header.stamp.sec = 11;
  EXPECT_EQ(p.observe(s, now + 240ms).source_id, "a");
}
TEST(AttentionPolicy, MalformedSceneDoesNotEraseAcceptedWatermark) {
  AttentionPolicy::Config config;
  config.require_advancing_scene_stamp = true;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(4);
  s.header.stamp.sec = 100;
  s.sources = {source("a", 0.8f, 4)};
  ASSERT_TRUE(p.observe(s, now).valid);
  auto malformed = scene(0);
  EXPECT_EQ(p.observe(malformed, now + 100ms).reason, "invalid_scene");
  auto old = scene(3);
  old.header.stamp.sec = 200;
  old.sources = {source("old", 0.99f, 3)};
  EXPECT_EQ(p.observe(old, now + 200ms).reason, "regressed_generation");
  EXPECT_EQ(p.observe(s, now + 300ms).reason, "regressed_scene_stamp");
  s.header.stamp.sec = 101;
  EXPECT_TRUE(p.observe(s, now + 400ms).valid);
}
TEST(AttentionPolicy, TimeoutClearsPendingChallengerConfirmation) {
  AttentionPolicy::Config config;
  config.scene_timeout = 200ms;
  config.switch_confirmations = 2;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(3);
  s.sources = {source("a", 0.7f, 3), source("b", 0.5f, 3)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[1].activity_score = 0.95f;
  EXPECT_EQ(p.observe(s, now + 50ms).source_id, "a");
  EXPECT_FALSE(p.expire(now + 251ms).valid);
  // No incumbent after timeout: acquisition is immediate, not a continuation
  // of the expired confirmation counter.
  EXPECT_EQ(p.observe(s, now + 260ms).source_id, "b");
}
TEST(AttentionPolicy, ExplicitResetClearsWatermarkForNewSession) {
  AttentionPolicy p({});
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(10);
  s.sources = {source("a", 0.9f, 10)};
  ASSERT_TRUE(p.observe(s, now).valid);
  p.reset();
  auto old = scene(1);
  old.sources = {source("new_session", 0.9f, 1)};
  EXPECT_EQ(p.observe(old, now + 100ms).source_id, "new_session");
}

// A queued scene can arrive after a long executor stall before the wall timer
// gets an opportunity to expire the previous selection.
TEST(AttentionPolicy, ObserveAfterTimeoutDoesNotRetainExpiredIncumbent) {
  AttentionPolicy::Config config;
  config.scene_timeout = 200ms;
  config.switch_margin = 0.4;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(5);
  s.sources = {source("a", 0.8f, 5), source("b", 0.6f, 5)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].activity_score = 0.4f;
  s.sources[1].activity_score = 0.7f;
  // Without implicit expiry, incumbent a would remain selected by hysteresis.
  EXPECT_EQ(p.observe(s, now + 201ms).source_id, "b");
}
TEST(AttentionPolicy, ObserveAfterClockRollbackStartsNewSelection) {
  AttentionPolicy::Config config;
  config.scene_timeout = 200ms;
  config.switch_margin = 0.4;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(5);
  s.sources = {source("a", 0.8f, 5), source("b", 0.6f, 5)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].activity_score = 0.4f;
  s.sources[1].activity_score = 0.7f;
  EXPECT_EQ(p.observe(s, now - 1ms).source_id, "b");
}
TEST(AttentionPolicy, ObserveAfterTimeoutStillRejectsOlderGeneration) {
  AttentionPolicy::Config config;
  config.scene_timeout = 200ms;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(8);
  s.sources = {source("a", 0.9f, 8)};
  ASSERT_TRUE(p.observe(s, now).valid);
  auto older = scene(7);
  older.sources = {source("old", 0.99f, 7)};
  EXPECT_EQ(p.observe(older, now + 201ms).reason, "regressed_generation");
  EXPECT_FALSE(p.expire(now + 202ms).valid);
  EXPECT_EQ(p.observe(s, now + 210ms).source_id, "a");
}
TEST(AttentionPolicy, ObserveWithinTimeoutPreservesIncumbentHysteresis) {
  AttentionPolicy::Config config;
  config.scene_timeout = 200ms;
  config.switch_margin = 0.4;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(5);
  s.sources = {source("a", 0.8f, 5), source("b", 0.6f, 5)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].activity_score = 0.4f;
  s.sources[1].activity_score = 0.7f;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "a");
}


TEST(AttentionPolicy, DirectionDiscontinuityQuarantinesSameSourceAcrossSnapshots) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(3);
  s.sources = {source("a", 0.9f, 3)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  // Invalidation must not turn a discontinuous track into a new acquisition.
  EXPECT_FALSE(p.observe(s, now + 200ms).valid);
  EXPECT_FALSE(p.observe(s, now + 300ms).valid);
}
TEST(AttentionPolicy, QuarantinedSourceDoesNotBlockIndependentCandidate) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(3);
  s.sources = {source("a", 0.9f, 3)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  s.sources.push_back(source("b", 0.6f, 3));
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.sources[0].activity_score = 0.99f;
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
}
TEST(AttentionPolicy, QuarantineClearsAfterAcceptedAbsence) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(3);
  s.sources = {source("a", 0.9f, 3)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources.clear();
  EXPECT_FALSE(p.observe(s, now + 200ms).valid);
  s.sources = {source("a", 0.9f, 3)};
  EXPECT_TRUE(p.observe(s, now + 300ms).valid);
}
TEST(AttentionPolicy, QuarantineClearsOnNewGenerationButNotTimeout) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  config.scene_timeout = 200ms;
  AttentionPolicy p(config);
  auto now = AttentionPolicy::Clock::now();
  auto s = scene(3);
  s.sources = {source("a", 0.9f, 3)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  EXPECT_FALSE(p.expire(now + 400ms).valid);
  EXPECT_FALSE(p.observe(s, now + 500ms).valid);
  s.generation = 4;
  s.sources[0].source_generation = 4;
  EXPECT_TRUE(p.observe(s, now + 600ms).valid);
}

TEST(AttentionPolicy, MultipleDiscontinuousIdentitiesRemainQuarantined) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.95f), source("b", 0.8f), source("c", 0.6f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.sources[1].direction_unit.x = 0.0;
  s.sources[1].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 200ms).source_id, "c");
  // Both higher-ranked sources are still present and must stay fenced.
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "c");
  s.sources.erase(s.sources.begin() + 2);
  EXPECT_FALSE(p.observe(s, now + 400ms).valid);
  EXPECT_FALSE(p.observe(s, now + 500ms).valid);
}

TEST(AttentionPolicy, QuarantineReleaseIsPerIdentityAndRequiresAcceptedAbsence) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  config.require_advancing_scene_stamp = true;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.header.stamp.sec = 10;
  s.sources = {source("a", 0.95f), source("b", 0.8f), source("c", 0.6f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.header.stamp.sec = 11;
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.header.stamp.sec = 12;
  s.sources[1].direction_unit.x = 0.0;
  s.sources[1].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 200ms).source_id, "c");
  // Replayed absence cannot release a quarantine.
  auto absent = s;
  absent.header.stamp.sec = 11;
  absent.sources.erase(absent.sources.begin());
  EXPECT_EQ(p.observe(absent, now + 250ms).reason, "regressed_scene_stamp");
  s.header.stamp.sec = 13;
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "c");
  // Accepted absence of a releases a only; b remains quarantined.
  absent.header.stamp.sec = 14;
  ASSERT_EQ(p.observe(absent, now + 400ms).source_id, "c");
  s.header.stamp.sec = 15;
  EXPECT_EQ(p.observe(s, now + 500ms).source_id, "a");
  s.sources.erase(s.sources.begin());
  s.header.stamp.sec = 16;
  EXPECT_EQ(p.observe(s, now + 600ms).source_id, "c");
}

TEST(AttentionPolicy, QuarantinesClearTogetherOnNewGeneration) {
  AttentionPolicy::Config config;
  config.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(config);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.95f), source("b", 0.8f), source("c", 0.6f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.sources[1].direction_unit.x = 0.0;
  s.sources[1].direction_unit.y = 1.0;
  ASSERT_EQ(p.observe(s, now + 200ms).source_id, "c");
  s.generation = 2;
  for (auto & src : s.sources) src.source_generation = 2;
  EXPECT_EQ(p.observe(s, now + 300ms).source_id, "a");
}

TEST(AttentionPolicy, InvalidIncumbentBearingIsQuarantinedUntilAbsence) {
  AttentionPolicy::Config c;
  c.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(c);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].has_direction = false;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources[0].has_direction = true;
  s.sources[0].direction_unit.x = 0.0;
  s.sources[0].direction_unit.y = 1.0;
  EXPECT_FALSE(p.observe(s, now + 200ms).valid);
  s.sources.clear();
  EXPECT_FALSE(p.observe(s, now + 300ms).valid);
  s.sources = {source("a", 0.9f)};
  EXPECT_TRUE(p.observe(s, now + 400ms).valid);
}
TEST(AttentionPolicy, NonfiniteIncumbentDoesNotBlockIndependentCandidate) {
  AttentionPolicy::Config c;
  c.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(c);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f), source("b", 0.6f)};
  ASSERT_EQ(p.observe(s, now).source_id, "a");
  s.sources[0].direction_unit.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(p.observe(s, now + 100ms).source_id, "b");
  s.sources[0] = source("a", 0.99f);
  EXPECT_EQ(p.observe(s, now + 200ms).source_id, "b");
}
TEST(AttentionPolicy, LowActivityDoesNotCauseDirectionQuarantine) {
  AttentionPolicy::Config c;
  c.maximum_direction_jump_rad = 0.3;
  AttentionPolicy p(c);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  s.sources = {source("a", 0.9f)};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].activity_score = 0.1f;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources[0].activity_score = 0.9f;
  EXPECT_TRUE(p.observe(s, now + 200ms).valid);
}
TEST(AttentionPolicy, InvalidIncumbentUncertaintyRequiresAbsence) {
  AttentionPolicy::Config c;
  c.maximum_direction_jump_rad = 0.3;
  c.maximum_angular_variance_rad2 = 0.05;
  AttentionPolicy p(c);
  auto s = scene();
  auto now = AttentionPolicy::Clock::now();
  auto a = source("a", 0.9f);
  a.has_angular_variance = true;
  a.angular_variance_rad2 = 0.01;
  s.sources = {a};
  ASSERT_TRUE(p.observe(s, now).valid);
  s.sources[0].angular_variance_rad2 = 0.2;
  EXPECT_FALSE(p.observe(s, now + 100ms).valid);
  s.sources[0].angular_variance_rad2 = 0.01;
  EXPECT_FALSE(p.observe(s, now + 200ms).valid);
  s.sources.clear();
  EXPECT_FALSE(p.observe(s, now + 300ms).valid);
  s.sources = {a};
  EXPECT_TRUE(p.observe(s, now + 400ms).valid);
}
