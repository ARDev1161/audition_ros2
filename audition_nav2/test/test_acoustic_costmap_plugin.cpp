#include <gtest/gtest.h>
#include <cstdint>
#include <memory>
#include <vector>
#include "audition_nav2/acoustic_costmap_layer.hpp"
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
