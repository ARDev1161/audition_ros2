#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <chrono>
#include <thread>
#include <limits>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "audition_nav2/acoustic_verified_metric_input.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_loader.hpp"

TEST(AcousticCostmapPlugin, ExportedAndInertByDefault) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  ASSERT_TRUE(loader.isClassAvailable("audition_nav2::AcousticCostmapLayer"));
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  EXPECT_TRUE(layer->isClearable());
  nav2_costmap_2d::Costmap2D master(8, 8, 0.5, -2.0, -2.0, 0);
  master.setCost(2, 2, 254);  // Other layer's lethal obstacle.
  master.setCost(3, 3, 255);  // Unknown must remain unknown.
  master.setCost(4, 4, 175);  // Other layer's soft cost.
  double min_x = -1.0, min_y = -1.0, max_x = 1.0, max_y = 1.0;
  layer->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, -1.0);
  EXPECT_DOUBLE_EQ(max_y, 1.0);
  layer->updateCosts(master, 0, 0, 8, 8);
  EXPECT_EQ(master.getCost(2, 2), 254);
  EXPECT_EQ(master.getCost(3, 3), 255);
  EXPECT_EQ(master.getCost(4, 4), 175);
  layer->reset();
  layer->updateCosts(master, 0, 0, 8, 8);
  EXPECT_EQ(master.getCost(2, 2), 254);
  EXPECT_EQ(master.getCost(3, 3), 255);
  EXPECT_EQ(master.getCost(4, 4), 175);
}

TEST(AcousticCostmapPlugin, NeverMutatesMasterAcrossResizeRollingAndRepeatedReset) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  for (unsigned cycle = 0; cycle < 12; ++cycle) {
    const unsigned width = 4 + cycle;
    const unsigned height = 5 + cycle;
    nav2_costmap_2d::Costmap2D master(
        width, height, 0.1 + 0.01 * cycle, -2.0 + cycle, -3.0, 0);
    for (unsigned j = 0; j < height; ++j) {
      for (unsigned i = 0; i < width; ++i) {
        const unsigned selector = (i + 3 * j + cycle) % 5;
        master.setCost(i, j, selector == 0 ? 254 :
                                 selector == 1 ? 255 :
                                 selector == 2 ? 175 : 0);
      }
    }
    std::vector<std::uint8_t> before(master.getCharMap(),
                                     master.getCharMap() + width * height);
    double min_x = -12.0, min_y = -9.0, max_x = 12.0, max_y = 9.0;
    layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
    EXPECT_DOUBLE_EQ(min_x, -12.0);
    EXPECT_DOUBLE_EQ(min_y, -9.0);
    EXPECT_DOUBLE_EQ(max_x, 12.0);
    EXPECT_DOUBLE_EQ(max_y, 9.0);
    layer->updateCosts(master, 0, 0, static_cast<int>(width),
                       static_cast<int>(height));
    layer->reset();
    layer->updateCosts(master, 0, 0, static_cast<int>(width),
                       static_cast<int>(height));
    EXPECT_EQ(std::vector<std::uint8_t>(
                  master.getCharMap(), master.getCharMap() + width * height),
              before);
  }
}

TEST(AcousticCostmapPlugin, RejectsUninitializedEvidenceAndNoImplicitEnable) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  audition_nav2::AcousticLayerCoordinator coordinator;
  const audition_nav2::GridGeometry geometry{8, 8, 0.5, 0, 0};
  EXPECT_FALSE(coordinator.ingest_validated(
      {1, 100, 200, 1.0, 1.0, true}, 110));
  const auto cycle = coordinator.tick(110, geometry);
  EXPECT_TRUE(cycle.update.own_cells.empty());
  EXPECT_EQ(coordinator.source_count(), 0u);
}

TEST(AcousticCostmapPlugin, IncompleteBoundsNeverMutateOtherLayers) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  for (int missing = 0; missing < 4; ++missing) {
    double values[4] = {-5.0, -6.0, 7.0, 8.0};
    double* bounds[4] = {&values[0], &values[1], &values[2], &values[3]};
    bounds[missing] = nullptr;
    layer->updateBounds(0, 0, 0, bounds[0], bounds[1], bounds[2], bounds[3]);
    EXPECT_DOUBLE_EQ(values[0], -5.0);
    EXPECT_DOUBLE_EQ(values[1], -6.0);
    EXPECT_DOUBLE_EQ(values[2], 7.0);
    EXPECT_DOUBLE_EQ(values[3], 8.0);
  }
}

TEST(AcousticCostmapPlugin, FailClosedAcrossClippedAndInvalidUpdateWindows) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  nav2_costmap_2d::Costmap2D master(19, 13, 0.25, -2.5, 1.0, 0);
  for (unsigned y = 0; y < 13; ++y) {
    for (unsigned x = 0; x < 19; ++x) {
      const unsigned kind = (x + 7 * y) % 4;
      master.setCost(x, y, kind == 0 ? 254 : kind == 1 ? 255 :
                             kind == 2 ? 175 : 0);
    }
  }
  const std::vector<std::uint8_t> expected(
      master.getCharMap(), master.getCharMap() + 19 * 13);
  const int windows[][4] = {{0, 0, 19, 13}, {4, 3, 7, 8},
                             {0, 0, 0, 0}, {18, 12, 19, 13},
                             {9, 9, 3, 3}, {-2, -2, 2, 2}};
  for (const auto& window : windows) {
    layer->updateCosts(master, window[0], window[1], window[2], window[3]);
    layer->reset();
    EXPECT_EQ(std::vector<std::uint8_t>(
                  master.getCharMap(), master.getCharMap() + 19 * 13),
              expected);
  }
}

TEST(AcousticCostmapPlugin, ReobservesGeometryBetweenCallbacksWithoutWriting) {
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(layer, nullptr);
  nav2_costmap_2d::Costmap2D master(9, 7, 0.2, -1.0, -1.0, 0);
  for (unsigned y = 0; y < 7; ++y)
    for (unsigned x = 0; x < 9; ++x)
      master.setCost(x, y, static_cast<unsigned char>((x + y) % 2 ? 254 : 175));
  double min_x = -4, min_y = -4, max_x = 4, max_y = 4;
  layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  layer->updateCosts(master, 0, 0, 9, 7);
  // Geometry changes without an intervening bounds callback.
  master.updateOrigin(-0.8, -1.0);
  const std::vector<std::uint8_t> shifted(master.getCharMap(),
                                           master.getCharMap() + 63);
  layer->updateCosts(master, 0, 0, 9, 7);
  EXPECT_EQ(std::vector<std::uint8_t>(master.getCharMap(),
                                      master.getCharMap() + 63), shifted);
  // Repeated reset and ABA return cannot authorize stale acoustic cells.
  master.updateOrigin(-1.0, -1.0);
  const std::vector<std::uint8_t> restored(master.getCharMap(),
                                            master.getCharMap() + 63);
  layer->reset();
  layer->updateCosts(master, 0, 0, 9, 7);
  EXPECT_EQ(std::vector<std::uint8_t>(master.getCharMap(),
                                      master.getCharMap() + 63), restored);
}

TEST(AcousticCostmapPlugin, DisabledRealLayeredCostmapNeverCreatesAudioCosts) {
  // Exercise Nav2's actual reset/updateBounds/updateCosts ordering with the
  // exported production plugin, not a coordinator or substitute Layer.
  // Without explicit trusted metric opt-in it must remain inert through
  // rolling-window movement, resize, repeated updates and reset.
  nav2_costmap_2d::LayeredCostmap layered("map", true, false);
  layered.resizeMap(12, 12, 0.5, -1.0, -1.0);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto acoustic = loader.createSharedInstance(
      "audition_nav2::AcousticCostmapLayer");
  ASSERT_NE(acoustic, nullptr);
  layered.addPlugin(acoustic);
  const auto check_empty = [&layered](double x, double y) {
    layered.updateMap(x, y, 0.0);
    const auto* map = layered.getCostmap();
    ASSERT_NE(map, nullptr);
    for (unsigned j = 0; j < map->getSizeInCellsY(); ++j) {
      for (unsigned i = 0; i < map->getSizeInCellsX(); ++i) {
        EXPECT_EQ(map->getCost(i, j), 0)
            << "unexpected cost at " << i << "," << j;
      }
    }
  };
  check_empty(2.0, 2.0);
  check_empty(3.0, 2.0);
  acoustic->reset();
  check_empty(3.0, 2.0);
  layered.resizeMap(15, 9, 0.25, -2.0, -0.5);
  check_empty(0.0, 0.0);
  check_empty(1.0, 0.0);
}

namespace audition_nav2 {
// This accessor exercises the production subscription callback's invalidation
// contract without DDS timing or a simulated substitute costmap plugin.
struct AcousticCostmapLayerTestAccess {
  static bool fresh(std::int64_t staged, std::int64_t now) {
    return AcousticCostmapLayer::pending_fresh(staged, now);
  }
  static void prepare(AcousticCostmapLayer& layer,
                      nav2_costmap_2d::LayeredCostmap& map) {
    layer.clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    layer.layered_costmap_ = &map;
    layer.enabled_ = true;
    layer.coordinator_.enable();
    layer.last_metric_generation_ = 5;
    layer.pending_valid_ = true;  // updateBounds already staged a cycle.
  }
  static bool seed_staged_dirty(AcousticCostmapLayer& layer,
                                const GridGeometry& geometry,
                                std::int64_t now) {
    layer.pending_valid_ = false;
    if (!layer.coordinator_.ingest_validated(
            {42, now, now + 200000000LL, 1.25, 1.25, true}, now))
      return false;
    layer.pending_cycle_ = layer.coordinator_.tick(now, geometry);
    layer.pending_geometry_ = geometry;
    layer.pending_valid_ = true;
    return layer.pending_cycle_.rebuild_required;
  }
  static bool seed_live(AcousticCostmapLayer& layer,
                        std::uint64_t id, std::int64_t now) {
    return layer.coordinator_.ingest_validated(
        {id, now, now + 120000000LL, 1.25, 1.25, true}, now);
  }
  static void expire(AcousticCostmapLayer& layer) {
    layer.expireOnTimer();
  }
  static void prime_expired_costs(AcousticCostmapLayer& layer,
                                  const GridGeometry& geometry) {
    layer.map_generation_.observe(geometry);
    layer.pending_stamp_ns_ = 1;  // Valid but far older than wall-clock TTL.
  }
  static void prime_fresh_costs(AcousticCostmapLayer& layer,
                                std::int64_t now) {
    layer.pending_stamp_ns_ = now;
  }
  static void set_clock(AcousticCostmapLayer& layer,
                        const rclcpp::Clock::SharedPtr& clock) {
    layer.clock_ = clock;
  }
  static std::size_t staged_cells(const AcousticCostmapLayer& layer) {
    return layer.pending_cycle_.update.own_cells.size();
  }
  static AcousticCycle retry(AcousticCostmapLayer& layer,
                             const GridGeometry& geometry,
                             std::int64_t now) {
    return layer.coordinator_.tick(now, geometry);
  }
  static bool pending(const AcousticCostmapLayer& layer) {
    return layer.pending_valid_;
  }
  static void seed(AcousticCostmapLayer& layer, std::uint64_t id, std::int64_t stamp) {
    layer.last_metric_stamp_[id] = stamp;
  }
  static std::size_t watermark_count(const AcousticCostmapLayer& layer) {
    return layer.last_metric_stamp_.size();
  }
  static std::uint64_t generation(const AcousticCostmapLayer& layer) {
    return layer.last_metric_generation_;
  }
  static std::size_t sources(const AcousticCostmapLayer& layer) {
    return layer.coordinator_.source_count();
  }
  static void stage(AcousticCostmapLayer& layer) {
    layer.pending_valid_ = true;
  }
  static void subscribe(AcousticCostmapLayer& layer,
                        const rclcpp::Node::SharedPtr& node,
                        const std::string& topic) {
    layer.verified_metric_subscription_ =
        node->create_subscription<audition_msgs::msg::LocalizedSourceArray>(
            topic, rclcpp::QoS(10).reliable(),
            [&layer](const audition_msgs::msg::LocalizedSourceArray& message) {
              layer.onTrustedMetricArray(message);
            });
  }
  static void ingress_message(AcousticCostmapLayer& layer,
                              const audition_msgs::msg::LocalizedSourceArray& message) {
    layer.onTrustedMetricArray(message);
  }
  static void ingress(AcousticCostmapLayer& layer, std::uint64_t generation) {
    audition_msgs::msg::LocalizedSourceArray message;
    message.generation = generation;
    layer.onTrustedMetricArray(message);
  }
};
}  // namespace audition_nav2

TEST(AcousticCostmapPlugin, TrustedIngressFencesStagedBoundsCycle) {
  // The callbacks are intentionally invoked in the interleaving that caused
  // the race: bounds staged -> subscription callback -> costs. A packet from
  // the current or a newer epoch must revoke the staged overlay, including
  // malformed/empty packets. A replay from an older epoch is a no-op.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(8, 8, 0.5, -2.0, -2.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  Access::ingress(layer, 4);
  EXPECT_TRUE(Access::pending(layer))
      << "old-generation replay must not invalidate current bounds";
  Access::ingress(layer, 5);
  EXPECT_FALSE(Access::pending(layer))
      << "same-generation ingress must discard staged acoustic costs";
  Access::stage(layer);
  Access::ingress(layer, 6);
  EXPECT_FALSE(Access::pending(layer))
      << "new-generation ingress must discard staged acoustic costs";

  // Even if Nav2 calls updateCosts before another updateBounds, the
  // invalidated half-cycle cannot write to its master costmap.
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  master->setCost(2, 2, 254);
  master->setCost(3, 3, 175);
  layer.updateCosts(*master, 0, 0, 8, 8);
  EXPECT_EQ(master->getCost(2, 2), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}

TEST(AcousticCostmapPlugin, RevocationClearsReplayWatermarksAndResetEpoch) {
  // A rejected same-epoch snapshot must not poison deduplication: a later
  // independently verified retry with the same stamp must remain admissible.
  // An older epoch, by contrast, cannot revoke a current staged cycle.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(8, 8, 0.5, -2.0, -2.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  Access::seed(layer, 101, 100);
  Access::seed(layer, 102, 200);
  ASSERT_EQ(Access::watermark_count(layer), 2u);
  Access::ingress(layer, 4);
  EXPECT_EQ(Access::watermark_count(layer), 2u);
  EXPECT_TRUE(Access::pending(layer));
  Access::ingress(layer, 5);  // Empty trusted snapshot revokes all.
  EXPECT_EQ(Access::watermark_count(layer), 0u);
  EXPECT_FALSE(Access::pending(layer));
  EXPECT_EQ(Access::sources(layer), 0u);
  EXPECT_EQ(Access::generation(layer), 5u);

  Access::seed(layer, 103, 300);
  Access::stage(layer);
  Access::ingress(layer, 6);  // Missing header: invalid, not a verified epoch.
  EXPECT_EQ(Access::watermark_count(layer), 0u);
  EXPECT_FALSE(Access::pending(layer));
  EXPECT_EQ(Access::generation(layer), 5u)
      << "invalid higher-generation input must not advance the replay fence";
  // The same invalid generation may be retried; it must remain fail-closed.
  Access::stage(layer);
  Access::ingress(layer, 6);
  EXPECT_FALSE(Access::pending(layer));
  EXPECT_EQ(Access::generation(layer), 5u);

  Access::seed(layer, 104, 400);
  layer.reset();
  EXPECT_EQ(Access::watermark_count(layer), 0u);
  EXPECT_EQ(Access::generation(layer), 0u);
  EXPECT_EQ(Access::sources(layer), 0u);
  EXPECT_FALSE(Access::pending(layer));
}

TEST(AcousticCostmapPlugin, StagedOverlayExpiresAcrossNav2CallbackDelay) {
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  constexpr std::int64_t kTtl =
      audition_nav2::AcousticVerifiedMetricInput::kTtlNs;
  constexpr std::int64_t kStart = 1000000000LL;
  EXPECT_FALSE(Access::fresh(0, kStart));
  EXPECT_FALSE(Access::fresh(kStart, kStart - 1));
  EXPECT_TRUE(Access::fresh(kStart, kStart));
  EXPECT_TRUE(Access::fresh(kStart, kStart + kTtl));
  EXPECT_FALSE(Access::fresh(kStart, kStart + kTtl + 1));
}

TEST(AcousticCostmapPlugin, StagedOverlayTimestampArithmeticIsOverflowSafe) {
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  const auto max = std::numeric_limits<std::int64_t>::max();
  const auto min = std::numeric_limits<std::int64_t>::min();
  constexpr auto ttl = audition_nav2::AcousticVerifiedMetricInput::kTtlNs;
  // A large positive clock jump must fail closed without signed overflow.
  EXPECT_FALSE(Access::fresh(1, max));
  EXPECT_FALSE(Access::fresh(1, max - 1));
  EXPECT_FALSE(Access::fresh(1, min));
  EXPECT_FALSE(Access::fresh(min, max));
  // The upper representable timestamp boundary remains inclusive at TTL.
  EXPECT_TRUE(Access::fresh(max - ttl, max));
  EXPECT_FALSE(Access::fresh(max - ttl - 1, max));
  EXPECT_FALSE(Access::fresh(max, max - 1));
  EXPECT_FALSE(Access::fresh(max, min));
}

TEST(AcousticCostmapPlugin, DiscardedBoundsAreRebuiltAfterMetricRevocation) {
  // A bounds pass consumes dirty ownership. If ingress invalidates that
  // pass before costs, its dirty region must be included in the next pass.
  audition_nav2::AcousticLayerCoordinator coordinator;
  coordinator.enable();
  const audition_nav2::GridGeometry grid{12, 12, 0.5, 0.0, 0.0};
  constexpr std::int64_t now = 1000000000LL;
  ASSERT_TRUE(coordinator.ingest_validated(
      {7, now, now + 500000000LL, 2.0, 2.0, true}, now));
  const auto staged = coordinator.tick(now, grid);
  ASSERT_TRUE(staged.rebuild_required);
  ASSERT_EQ(staged.update.own_cells.size(), 1u);
  coordinator.defer_staged_cycle(staged, grid);
  coordinator.revoke_all();
  const auto retry = coordinator.tick(now + 1, grid);
  EXPECT_TRUE(retry.rebuild_required);
  EXPECT_TRUE(retry.update.own_cells.empty());
  EXPECT_LE(retry.dirty.min_i, 4u);
  EXPECT_GT(retry.dirty.max_i, 4u);
  EXPECT_LE(retry.dirty.min_j, 4u);
  EXPECT_GT(retry.dirty.max_j, 4u);
  // Once the rebuild has been consumed, no phantom dirty work remains.
  EXPECT_FALSE(coordinator.tick(now + 2, grid).rebuild_required);
}

TEST(AcousticCostmapPlugin, DeferredBoundsNeverCrossRollingMapGeometry) {
  audition_nav2::AcousticLayerCoordinator coordinator;
  coordinator.enable();
  const audition_nav2::GridGeometry first{12, 12, 0.5, 0.0, 0.0};
  const audition_nav2::GridGeometry shifted{12, 12, 0.5, 1.0, 0.0};
  constexpr std::int64_t now = 1000000000LL;
  ASSERT_TRUE(coordinator.ingest_validated(
      {7, now, now + 500000000LL, 2.0, 2.0, true}, now));
  const auto staged = coordinator.tick(now, first);
  ASSERT_TRUE(staged.rebuild_required);
  coordinator.defer_staged_cycle(staged, first);
  coordinator.revoke_all();
  const auto retry = coordinator.tick(now + 1, shifted);
  EXPECT_TRUE(retry.rebuild_required);
  EXPECT_EQ(retry.dirty.min_i, 0u);
  EXPECT_EQ(retry.dirty.min_j, 0u);
  EXPECT_EQ(retry.dirty.max_i, shifted.width);
  EXPECT_EQ(retry.dirty.max_j, shifted.height);
}

TEST(AcousticCostmapPlugin, RepeatedBoundsCannotLoseStagedDirtyRegion) {
  // A second bounds callback may supersede a staged cycle without costs.
  // The dirty region from the first pass must survive the interruption.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(8, 8, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  const audition_nav2::GridGeometry geometry{8, 8, 0.5, 0.0, 0.0};
  constexpr std::int64_t now = 1000000000LL;
  ASSERT_TRUE(Access::seed_staged_dirty(layer, geometry, now));
  double min_x = -1, min_y = -1, max_x = 5, max_y = 5;
  layer.updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_FALSE(Access::pending(layer));
  const auto retry = Access::retry(layer, geometry, now + 1);
  EXPECT_TRUE(retry.rebuild_required);
  EXPECT_LE(retry.dirty.min_i, 2u);
  EXPECT_GT(retry.dirty.max_i, 2u);
  EXPECT_LE(retry.dirty.min_j, 2u);
  EXPECT_GT(retry.dirty.max_j, 2u);
}

TEST(AcousticCostmapPlugin, TimerRetirementPreservesStagedDirtyWithoutNav2Tick) {
  // Real production timer callback, with no incoming ROS messages and no
  // updateMap() in between. A staged soft cost must be cleared on the next
  // complete Nav2 cycle, even when its bounds were already consumed.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const std::int64_t observed = clock->now().nanoseconds();
  Access::set_clock(layer, clock);
  const audition_nav2::GridGeometry geometry{12, 12, 0.5, 0.0, 0.0};
  ASSERT_TRUE(Access::seed_staged_dirty(layer, geometry, observed));
  ASSERT_EQ(Access::staged_cells(layer), 1u);
  ASSERT_TRUE(Access::pending(layer));
  ASSERT_EQ(Access::sources(layer), 1u);

  // Timer before TTL is a no-op for a pending Nav2 half-cycle.
  Access::expire(layer);
  EXPECT_EQ(Access::sources(layer), 1u);
  EXPECT_TRUE(Access::pending(layer));

  // After expiry the timer must invalidate the half-cycle and requeue its
  // consumed dirty bounds, without ticking or modifying the master map.
  std::this_thread::sleep_for(std::chrono::milliseconds(310));
  Access::expire(layer);
  EXPECT_EQ(Access::sources(layer), 0u);
  EXPECT_FALSE(Access::pending(layer));
  const auto next = Access::retry(layer, geometry, clock->now().nanoseconds());
  EXPECT_TRUE(next.rebuild_required);
  EXPECT_TRUE(next.update.own_cells.empty());
  EXPECT_LE(next.dirty.min_i, 2u);
  EXPECT_GT(next.dirty.max_i, 2u);
  EXPECT_LE(next.dirty.min_j, 2u);
  EXPECT_GT(next.dirty.max_j, 2u);
  EXPECT_FALSE(Access::retry(layer, geometry, clock->now().nanoseconds())
                   .rebuild_required);
}

TEST(AcousticCostmapPlugin, ExpiredBoundsToCostsPassRetainsDirtyForRebuild) {
  // The real production updateCosts callback must not consume dirty bounds
  // permanently when a prepared overlay expires during a Nav2 stall.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  const audition_nav2::GridGeometry geometry{12, 12, 0.5, 0.0, 0.0};
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                       ->now().nanoseconds();
  ASSERT_TRUE(Access::seed_staged_dirty(layer, geometry, now));
  Access::prime_expired_costs(layer, geometry);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  master->setCost(2, 2, 175);  // Must never overwrite a non-acoustic cost.
  layer.updateCosts(*master, 0, 0, 12, 12);
  EXPECT_FALSE(Access::pending(layer));
  EXPECT_EQ(master->getCost(2, 2), 175);
  const auto retry = Access::retry(layer, geometry, now + 1);
  EXPECT_TRUE(retry.rebuild_required);
  EXPECT_LE(retry.dirty.min_i, 2u);
  EXPECT_GT(retry.dirty.max_i, 2u);
  EXPECT_LE(retry.dirty.min_j, 2u);
  EXPECT_GT(retry.dirty.max_j, 2u);
}

TEST(AcousticCostmapPlugin, RejectedCostsGateRequeuesConsumedDirtyBounds) {
  // Nav2 may reject a costs pass when the acoustic plugin is not terminal.
  // Bounds were already consumed by updateBounds; rejecting the overlay
  // must not lose the region needed to clear old master acoustic costs.
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  Access::prepare(layer, map);
  const audition_nav2::GridGeometry geometry{12, 12, 0.5, 0.0, 0.0};
  const auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(layer, clock);
  const auto now = clock->now().nanoseconds();
  ASSERT_TRUE(Access::seed_staged_dirty(layer, geometry, now));
  // Establish a matching map generation, as updateBounds normally does.
  Access::prime_expired_costs(layer, geometry);
  // Override the artificial expired timestamp with a fresh one.
  Access::prime_fresh_costs(layer, now);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  master->setCost(2, 2, 175);
  // No plugin is registered with the layered map: terminal baseline gate
  // must reject the update, preserving the independent master cost.
  layer.updateCosts(*master, 0, 0, 12, 12);
  EXPECT_FALSE(Access::pending(layer));
  EXPECT_EQ(master->getCost(2, 2), 175);
  const auto retry = Access::retry(layer, geometry, now + 1);
  EXPECT_TRUE(retry.rebuild_required);
  EXPECT_LE(retry.dirty.min_i, 2u);
  EXPECT_GT(retry.dirty.max_i, 2u);
  EXPECT_LE(retry.dirty.min_j, 2u);
  EXPECT_GT(retry.dirty.max_j, 2u);
}

// Exercise the production plugin through LayeredCostmap::updateMap, including
// Nav2's actual reset/rebuild order. Unlike coordinator-only tests, this
// verifies the master grid after TTL without any subsequent ROS observation.
namespace {
class AcousticTestPhysicalLayer final : public nav2_costmap_2d::Layer {
 public:
  void updateBounds(double, double, double, double* x0, double* y0,
                    double* x1, double* y1) override {
    *x0 = std::min(*x0, 0.0);
    *y0 = std::min(*y0, 0.0);
    *x1 = std::max(*x1, 4.0);
    *y1 = std::max(*y1, 4.0);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D& master,
                   int, int, int, int) override {
    master.setCost(1, 1, 254);
    master.setCost(3, 3, 175);
  }
  void reset() override {}
  bool isClearable() override { return true; }
};
}  // namespace

TEST(AcousticCostmapPlugin, ProductionLayeredMapClearsExpiredSoftCosts) {
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(8, 8, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);  // Terminal acoustic layer after physical baseline.
  Access::prepare(*acoustic, map);
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)->now().nanoseconds();
  ASSERT_TRUE(Access::seed_live(*acoustic, 77, now));
  map.updateMap(1.0, 1.0, 0.0);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // No new DDS messages or Nav2 updates during the pause. The actual
  // production expiry callback retires the source without consuming dirty.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  Access::expire(*acoustic);
  EXPECT_EQ(Access::sources(*acoustic), 0u);
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(2, 2), 0);
}

TEST(AcousticCostmapPlugin, CompleteSnapshotRevokesOmittedMetricSourceImmediately) {
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  Access::prepare(layer, map);
  const auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(layer, clock);
  const auto now = clock->now().nanoseconds();
  auto stamp = [](std::int64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return t;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp(now);
  snapshot.generation = 5;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = snapshot.header.stamp;
  first.source_id = "first";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "second";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  Access::ingress_message(layer, snapshot);
  ASSERT_EQ(Access::sources(layer), 2u);
  ASSERT_EQ(Access::watermark_count(layer), 2u);

  // The next complete verified snapshot still contains the first source,
  // but explicitly omits the second. Do not wait 500 ms for its TTL.
  snapshot.sources.pop_back();
  Access::ingress_message(layer, snapshot);
  EXPECT_EQ(Access::sources(layer), 1u);
  EXPECT_EQ(Access::watermark_count(layer), 1u);

  const audition_nav2::GridGeometry geometry{12, 12, 0.5, 0., 0.};
  const auto cycle = Access::retry(layer, geometry, clock->now().nanoseconds());
  ASSERT_TRUE(cycle.rebuild_required);
  ASSERT_EQ(cycle.update.own_cells.size(), 1u);
  EXPECT_EQ(cycle.update.own_cells.begin()->first.first, 2u);
  EXPECT_EQ(cycle.update.own_cells.begin()->first.second, 2u);
}


TEST(AcousticCostmapPlugin, ProductionLayeredMapClearsOmittedSourceWithoutTTL) {
  // Exercise complete-snapshot revocation through the real Nav2 master
  // reset/rebuild sequence. No timer or synthetic coordinator tick is used.
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const auto ns = clock->now().nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);

  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 8;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = stamp;
  first.source_id = "first";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "second";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  Access::ingress_message(*acoustic, snapshot);
  ASSERT_EQ(Access::sources(*acoustic), 2u);
  map.updateMap(1.0, 1.0, 0.0);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // An unchanged observation for the retained source is legal in a
  // complete snapshot; omission immediately retires the other source.
  snapshot.sources.pop_back();
  Access::ingress_message(*acoustic, snapshot);
  ASSERT_EQ(Access::sources(*acoustic), 1u);
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // Repeated cycles must not resurrect retired costs.
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(4, 2), 0);
  EXPECT_EQ(master->getCost(2, 2), 100);
}

TEST(AcousticCostmapPlugin, ReorderedCompleteSnapshotCannotResurrectOmittedSource) {
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  audition_nav2::AcousticCostmapLayer layer;
  Access::prepare(layer, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(layer, clock);
  const auto now = clock->now().nanoseconds();
  const auto to_stamp = [](std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = to_stamp(now - 30000000LL);
  snapshot.generation = 5;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = snapshot.header.stamp;
  first.source_id = "first";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "second";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  const auto old_snapshot = snapshot;
  Access::ingress_message(layer, snapshot);
  ASSERT_EQ(Access::sources(layer), 2u);

  // A newer complete snapshot omits the second source while retaining
  // the first source's unchanged observation timestamp.
  snapshot.header.stamp = to_stamp(now - 10000000LL);
  snapshot.sources.pop_back();
  Access::ingress_message(layer, snapshot);
  ASSERT_EQ(Access::sources(layer), 1u);
  ASSERT_EQ(Access::watermark_count(layer), 1u);

  // Reliable DDS can deliver the older complete array late. It must not
  // resurrect the omitted source or invalidate an in-flight Nav2 cycle.
  Access::stage(layer);
  Access::ingress_message(layer, old_snapshot);
  EXPECT_EQ(Access::sources(layer), 1u);
  EXPECT_EQ(Access::watermark_count(layer), 1u);
  EXPECT_TRUE(Access::pending(layer));

  // A still-newer valid snapshot may legitimately reintroduce the source.
  // Use a timestamp strictly in the past. The callback reads its own clock;
  // stamping with the earlier test-side `now` can otherwise be future-dated
  // on clock implementations with coarse/adjusted wall-time resolution.
  snapshot.header.stamp = to_stamp(now - 1000000LL);
  // Reappearance is a new metric observation, not a replay of the retired
  // source's old timestamp. Refresh both per-source observation stamps to
  // satisfy the coordinator's strict per-source ordering contract.
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  second.header.stamp = snapshot.header.stamp;
  second.last_observed = snapshot.header.stamp;
  snapshot.sources.push_back(second);
  Access::ingress_message(layer, snapshot);
  EXPECT_EQ(Access::sources(layer), 2u);
}


TEST(AcousticCostmapPlugin, InvalidMetricSnapshotClearsMasterWithoutTouchingPhysicalCosts) {
  // A formerly trusted producer can lose metric validity at runtime. An
  // invalid replacement snapshot must revoke the old acoustic overlay, not
  // retain a ghost cell until TTL or overwrite non-acoustic ownership.
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const auto ns = clock->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);

  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 42;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources.push_back(source);
  Access::ingress_message(*acoustic, snapshot);
  ASSERT_EQ(Access::sources(*acoustic), 1u);
  map.updateMap(1.0, 1.0, 0.0);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(1, 1), 254);
  ASSERT_EQ(master->getCost(3, 3), 175);

  // A bearing-only method cannot retain an old metric position. Revoke on
  // invalid input and perform a genuine Nav2 reset/rebuild pass.
  snapshot.sources.front().localization_method = "bearing_only";
  Access::ingress_message(*acoustic, snapshot);
  EXPECT_EQ(Access::sources(*acoustic), 0u);
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1.0, 1.0, 0.0);
  EXPECT_EQ(master->getCost(2, 2), 0);
}

TEST(AcousticCostmapPlugin, RealDdsIngressRevokesSourceAndClearsMaster) {
  // Real DDS delivery into the production callback, followed by Nav2's
  // production updateMap reset/rebuild. The test-only seam installs the same
  // reliable subscription as onInitialize without lifecycle dependencies.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_costmap_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  Access::set_clock(*acoustic, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const std::string topic = "/audition/test_verified_metric_costmap";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                       ->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(now / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 21;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // A complete empty trusted snapshot retires the source over DDS.
  // Increment publication time without changing the evidence epoch.
  const auto newer = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                         ->now().nanoseconds() - 1000000LL;
  snapshot.header.stamp.sec = static_cast<std::int32_t>(newer / 1000000000LL);
  snapshot.header.stamp.nanosec = static_cast<std::uint32_t>(newer % 1000000000LL);
  snapshot.sources.clear();
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
}

TEST(AcousticCostmapPlugin, RealDdsInvalidMetricRevokesAndClearsMaster) {
  // Real DDS delivery into the production callback, followed by Nav2's
  // production updateMap reset/rebuild. The test-only seam installs the same
  // reliable subscription as onInitialize without lifecycle dependencies.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_invalid_metric_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  Access::set_clock(*acoustic, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const std::string topic = "/audition/test_invalid_metric_costmap";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                       ->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(now / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 22;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // The same producer loses its independent metric verification. A bearing
  // remains a direction only: the old metric cell must be withdrawn, never
  // reused as an acoustic obstacle.
  const auto newer = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                         ->now().nanoseconds() - 1000000LL;
  snapshot.header.stamp.sec = static_cast<std::int32_t>(newer / 1000000000LL);
  snapshot.header.stamp.nanosec = static_cast<std::uint32_t>(newer % 1000000000LL);
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  snapshot.sources.front().localization_method = "bearing_only";
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
}

TEST(AcousticCostmapPlugin, RealDdsReorderedSnapshotCannotRestoreRetiredCost) {
  // Exercise the full DDS -> production subscription -> Nav2 master path.
  // Replaying an older complete snapshot must not resurrect retired cells.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_reorder_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_reordered_metric_costmap";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto to_stamp = [](std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray first;
  first.header.frame_id = "map";
  first.header.stamp = to_stamp(clock->now().nanoseconds() - 1000000LL);
  first.generation = 91;
  audition_msgs::msg::LocalizedSource source;
  source.header = first.header;
  source.last_observed = first.header.stamp;
  source.source_id = "reordered_dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  first.sources = {source};
  publisher->publish(first);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);

  // Ensure strict ordering even on coarse wall clocks; the snapshot epoch
  // remains unchanged, while the complete publication timestamp advances.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  auto retired = first;
  retired.header.stamp = to_stamp(clock->now().nanoseconds() - 1000000LL);
  ASSERT_GT(retired.header.stamp.sec * 1000000000LL +
                retired.header.stamp.nanosec,
            first.header.stamp.sec * 1000000000LL +
                first.header.stamp.nanosec);
  retired.sources.clear();
  publisher->publish(retired);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  ASSERT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // The delayed packet contains valid metric evidence but is older than
  // the complete retirement snapshot. It must be rejected before mutation.
  publisher->publish(first);
  // Spin the executor after publish; a stale packet cannot be detected
  // through source_count alone because it must leave that count unchanged.
  for (unsigned i = 0; i < 30; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(Access::sources(*acoustic), 0u);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}

TEST(AcousticCostmapPlugin, RealDdsSilenceExpiresMetricAndClearsMaster) {
  // A single valid DDS snapshot must not leave permanent costmap ownership
  // when its publisher disappears. Exercise the production timer expiry path
  // without sending an explicit empty snapshot or invoking the callback.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_silence_ttl_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_metric_silence_ttl";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto ns = clock->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 94;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "silent_dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(1, 1), 254);
  ASSERT_EQ(master->getCost(3, 3), 175);

  // No more DDS messages. Wait beyond the verified-evidence TTL, then
  // exercise the exact production expiry callback used by its wall timer.
  const auto expiry_deadline = std::chrono::steady_clock::now() +
      std::chrono::nanoseconds(
          audition_nav2::AcousticVerifiedMetricInput::kTtlNs) +
      std::chrono::milliseconds(50);
  std::this_thread::sleep_until(expiry_deadline);
  Access::expire(*acoustic);
  ASSERT_EQ(Access::sources(*acoustic), 0u);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
}

TEST(AcousticCostmapPlugin, RealDdsCompleteSnapshotOmitsOneOfTwoMetricSources) {
  // Real DDS delivery into the production callback, followed by Nav2's
  // production updateMap reset/rebuild. The test-only seam installs the same
  // reliable subscription as onInitialize without lifecycle dependencies.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_multi_source_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  Access::set_clock(*acoustic, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const std::string topic = "/audition/test_multi_source_metric_costmap";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                       ->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(now / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 95;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = source;
  second.source_id = "dds_metric_second";
  second.position.x = 2.25;
  second.position.y = 2.25;
  snapshot.sources = {source, second};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  ASSERT_EQ(master->getCost(4, 4), 100);

  // A complete snapshot retaining only the first source must revoke the
  // omitted second source immediately, without waiting for TTL.
  const auto newer = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                         ->now().nanoseconds() - 1000000LL;
  snapshot.header.stamp.sec = static_cast<std::int32_t>(newer / 1000000000LL);
  snapshot.header.stamp.nanosec = static_cast<std::uint32_t>(newer % 1000000000LL);
  snapshot.sources = {source};
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 4), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  // A second Nav2 cycle must preserve the retained source while never
  // resurrecting the omitted source or overwriting independent layers.
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 4), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}


TEST(AcousticCostmapPlugin, RealDdsMetricReprojectsAfterNav2Resize) {
  // Verify a live world-frame metric hypothesis survives a resolution change
  // without retaining its old grid index or corrupting independent layers.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_resize_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_metric_resize";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto ns = clock->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 96;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "resize_dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  ASSERT_NE(map.getCostmap(), nullptr);
  ASSERT_EQ(map.getCostmap()->getCost(2, 2), 100);

  // No new DDS message: the same metric world point moves from cell (2,2)
  // to cell (5,5) after resolution changes from 0.5 to 0.25 metres.
  map.resizeMap(24, 24, 0.25, 0.0, 0.0);
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(Access::sources(*acoustic), 1u);
  EXPECT_EQ(master->getCost(5, 5), 100);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(5, 5), 100);
  EXPECT_EQ(master->getCost(2, 2), 0);
}


TEST(AcousticCostmapPlugin, RealDdsMetricReprojectsAfterRollingOriginShift) {
  // A rolling window changes grid ownership, not the trusted world-frame
  // hypothesis. Verify Nav2's real master reset/rebuild without new DDS data.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_rolling_origin_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_metric_rolling_origin";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto ns = clock->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 97;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "rolling_dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  ASSERT_NE(map.getCostmap(), nullptr);
  ASSERT_EQ(map.getCostmap()->getCost(2, 2), 100);

  // Shift the rolling origin by two cells in each axis. The same world
  // coordinate (1.25, 1.25) now occupies (4,4), not (2,2).
  map.resizeMap(12, 12, 0.5, -1.0, -1.0);
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(Access::sources(*acoustic), 1u);
  EXPECT_EQ(master->getCost(4, 4), 100);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(4, 4), 100);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}

TEST(AcousticCostmapPlugin, RealDdsWrongFrameRevokesMetricUntilReverified) {
  // Real DDS delivery into the production callback, followed by Nav2's
  // production updateMap reset/rebuild. The test-only seam installs the same
  // reliable subscription as onInitialize without lifecycle dependencies.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_wrong_frame_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0.0, 0.0);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  Access::set_clock(*acoustic, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const std::string topic = "/audition/test_wrong_frame_costmap";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto now = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                       ->now().nanoseconds() - 1000000LL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(now / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(now % 1000000000LL);
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp;
  snapshot.generation = 122;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = stamp;
  source.source_id = "dds_metric";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // The upstream TF adapter must not relabel a sensor-frame position as map.
  // Even a metric method with finite covariance is invalid in the wrong frame.
  // The previously owned map cell must be cleared on this complete snapshot.
  const auto newer = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                         ->now().nanoseconds() - 1000000LL;
  snapshot.header.stamp.sec = static_cast<std::int32_t>(newer / 1000000000LL);
  snapshot.header.stamp.nanosec = static_cast<std::uint32_t>(newer % 1000000000LL);
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  snapshot.header.frame_id = "microphone_array";
  snapshot.sources.front().header.frame_id = "microphone_array";
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);

  // A fresh, correctly transformed and independently verified snapshot may
  // restore the soft cost; invalid frame data itself must never do so.
  snapshot.header.frame_id = "map";
  snapshot.sources.front().header.frame_id = "map";
  snapshot.generation += 1;
  const auto recovered = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)
                             ->now().nanoseconds() - 1000000LL;
  snapshot.header.stamp.sec = static_cast<std::int32_t>(recovered / 1000000000LL);
  snapshot.header.stamp.nanosec = static_cast<std::uint32_t>(recovered % 1000000000LL);
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}



TEST(AcousticCostmapPlugin, RealDdsStaleMetricEvidenceRevokesUntilFreshVerification) {
  // A current snapshot generation does not refresh an old observation.
  // In particular, a stale TF-derived metric estimate cannot keep a cell
  // alive merely because its wrapper was republished with a fresh header.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_stale_metric_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0., 0.);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_stale_metric_evidence";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto as_stamp = [](std::int64_t ns) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = as_stamp(clock->now().nanoseconds() - 1000000LL);
  snapshot.generation = 130;
  audition_msgs::msg::LocalizedSource source;
  source.header = snapshot.header;
  source.last_observed = snapshot.header.stamp;
  source.source_id = "stale_metric_dds";
  source.source_generation = 1;
  source.localization_method = "independently_verified_metric_3d";
  source.position.x = 1.25;
  source.position.y = 1.25;
  source.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  snapshot.sources = {source};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(1, 1), 254);
  ASSERT_EQ(master->getCost(3, 3), 175);

  // The envelope is fresh, but the metric observation predates the TTL.
  // A complete snapshot must revoke the old cost immediately.
  snapshot.generation++;
  snapshot.header.stamp = as_stamp(clock->now().nanoseconds() - 1000000LL);
  const auto stale = as_stamp(clock->now().nanoseconds() -
      audition_nav2::AcousticVerifiedMetricInput::kTtlNs - 100000000LL);
  snapshot.sources.front().header.stamp = stale;
  snapshot.sources.front().last_observed = stale;
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);

  // Only a newly verified metric observation can restore the soft cost.
  snapshot.generation++;
  snapshot.header.stamp = as_stamp(clock->now().nanoseconds() - 1000000LL);
  snapshot.sources.front().header.stamp = snapshot.header.stamp;
  snapshot.sources.front().last_observed = snapshot.header.stamp;
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 1u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}


TEST(AcousticCostmapPlugin, RealDdsInvalidCovarianceRevokesEntireMetricBatch) {
  // Reject a complete snapshot atomically: one valid source must not survive
  // alongside a second source with an unbounded/degenerate range covariance.
  // A later fresh, fully verified snapshot is required to restore both cells.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_covariance_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0., 0.);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_covariance_batch_gate";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto stamp_now = [&] {
    const auto ns = clock->now().nanoseconds() - 1000000LL;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp_now();
  snapshot.generation = 140;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = snapshot.header.stamp;
  first.source_id = "verified_left";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "verified_right";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(4, 2), 100);

  // A newer snapshot is invalid as a whole even though the first member
  // remains well-formed. Neither previous acoustic cell may persist.
  snapshot.generation++;
  snapshot.header.stamp = stamp_now();
  for (auto& source : snapshot.sources) {
    source.header.stamp = snapshot.header.stamp;
    source.last_observed = snapshot.header.stamp;
  }
  snapshot.sources[1].covariance[8] = 0.0;  // Unknown depth is not zero variance.
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(4, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(4, 2), 0);

  // Recovery requires a new, fully validated complete snapshot.
  snapshot.generation++;
  snapshot.header.stamp = stamp_now();
  snapshot.sources[1].covariance[8] = 0.04;
  for (auto& source : snapshot.sources) {
    source.header.stamp = snapshot.header.stamp;
    source.last_observed = snapshot.header.stamp;
  }
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}

TEST(AcousticCostmapPlugin, RealDdsUnverifiedLocalizationMethodsRevokeEntireMetricBatch) {
  // Reject mixed snapshots atomically for every unverified localization class.
  // A 3-D-looking position/covariance is not proof of metric localization.
  // Only a subsequent independently verified complete snapshot can recover.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_bearing_batch_gate");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0., 0.);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_bearing_batch_gate";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto stamp_now = [&] {
    const auto ns = clock->now().nanoseconds() - 1000000LL;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp_now();
  snapshot.generation = 150;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = snapshot.header.stamp;
  first.source_id = "verified_left";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "verified_right";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(4, 2), 100);

  // Every case starts with two trusted cells. The unverified second member
  // must revoke the entire snapshot, not merely its own cell. Repeat after
  // recovery to catch stale generation/watermark or dirty-region leakage.
  const std::array<std::string, 4> invalid_methods = {
      "bearing_only", "av_pair", "diagnostic_2d_intersection",
      "unverified_metric"};
  for (const auto& method : invalid_methods) {
    SCOPED_TRACE(method);
    snapshot.generation++;
    snapshot.header.stamp = stamp_now();
    for (auto& member : snapshot.sources) {
      member.header.stamp = snapshot.header.stamp;
      member.last_observed = snapshot.header.stamp;
    }
    snapshot.sources[1].localization_method = method;
    publisher->publish(snapshot);
    ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
    map.updateMap(1., 1., 0.);
    EXPECT_EQ(master->getCost(2, 2), 0);
    EXPECT_EQ(master->getCost(4, 2), 0);
    EXPECT_EQ(master->getCost(1, 1), 254);
    EXPECT_EQ(master->getCost(3, 3), 175);
    map.updateMap(1., 1., 0.);
    EXPECT_EQ(master->getCost(2, 2), 0);
    EXPECT_EQ(master->getCost(4, 2), 0);

    snapshot.generation++;
    snapshot.header.stamp = stamp_now();
    snapshot.sources[1].localization_method = "independently_verified_metric_3d";
    for (auto& member : snapshot.sources) {
      member.header.stamp = snapshot.header.stamp;
      member.last_observed = snapshot.header.stamp;
    }
    publisher->publish(snapshot);
    ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
    map.updateMap(1., 1., 0.);
    EXPECT_EQ(master->getCost(2, 2), 100);
    EXPECT_EQ(master->getCost(4, 2), 100);
    EXPECT_EQ(master->getCost(1, 1), 254);
    EXPECT_EQ(master->getCost(3, 3), 175);
  }
}

TEST(AcousticCostmapPlugin, RealDdsDuplicateIdentityRevokesWholeSnapshot) {
  // A complete DDS snapshot with duplicate source identity is ambiguous.
  // It must not partially overwrite existing costmap ownership or allow
  // the duplicate to silently replace a verified independent source.
  const bool own_context = !rclcpp::ok();
  if (own_context) rclcpp::init(0, nullptr);
  struct ContextGuard {
    bool owned;
    ~ContextGuard() { if (owned && rclcpp::ok()) rclcpp::shutdown(); }
  } guard{own_context};
  using Access = audition_nav2::AcousticCostmapLayerTestAccess;
  auto node = std::make_shared<rclcpp::Node>("acoustic_dds_duplicate_identity");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(12, 12, 0.5, 0., 0.);
  auto physical = std::make_shared<AcousticTestPhysicalLayer>();
  auto acoustic = std::make_shared<audition_nav2::AcousticCostmapLayer>();
  map.addPlugin(physical);
  map.addPlugin(acoustic);
  Access::prepare(*acoustic, map);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  Access::set_clock(*acoustic, clock);
  const std::string topic = "/audition/test_duplicate_metric_identity";
  Access::subscribe(*acoustic, node, topic);
  auto publisher = node->create_publisher<audition_msgs::msg::LocalizedSourceArray>(
      topic, rclcpp::QoS(10).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto pump_until = [&](const std::function<bool()>& ready) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ready();
  };
  ASSERT_TRUE(pump_until([&] { return publisher->get_subscription_count() > 0; }));
  const auto stamp_now = [&] {
    const auto ns = clock->now().nanoseconds() - 1000000LL;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  };
  audition_msgs::msg::LocalizedSourceArray snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp = stamp_now();
  snapshot.generation = 160;
  audition_msgs::msg::LocalizedSource first;
  first.header = snapshot.header;
  first.last_observed = snapshot.header.stamp;
  first.source_id = "independent_left";
  first.source_generation = 1;
  first.localization_method = "independently_verified_metric_3d";
  first.position.x = 1.25;
  first.position.y = 1.25;
  first.covariance = {0.04, 0., 0., 0., 0.04, 0., 0., 0., 0.04};
  auto second = first;
  second.source_id = "independent_right";
  second.position.x = 2.25;
  snapshot.sources = {first, second};
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  auto* master = map.getCostmap();
  ASSERT_NE(master, nullptr);
  ASSERT_EQ(master->getCost(2, 2), 100);
  ASSERT_EQ(master->getCost(4, 2), 100);

  snapshot.generation++;
  snapshot.header.stamp = stamp_now();
  for (auto& member : snapshot.sources) {
    member.header.stamp = snapshot.header.stamp;
    member.last_observed = snapshot.header.stamp;
  }
  snapshot.sources[1].source_id = snapshot.sources[0].source_id;
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 0u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(4, 2), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);
  EXPECT_EQ(master->getCost(4, 2), 0);

  // Re-admit only a new, unambiguous and independently verified snapshot.
  snapshot.generation++;
  snapshot.header.stamp = stamp_now();
  snapshot.sources[1].source_id = "independent_right";
  for (auto& member : snapshot.sources) {
    member.header.stamp = snapshot.header.stamp;
    member.last_observed = snapshot.header.stamp;
  }
  publisher->publish(snapshot);
  ASSERT_TRUE(pump_until([&] { return Access::sources(*acoustic) == 2u; }));
  map.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 100);
  EXPECT_EQ(master->getCost(4, 2), 100);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}
