#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include <memory>
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
