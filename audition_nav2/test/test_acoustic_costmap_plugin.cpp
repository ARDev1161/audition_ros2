#include <gtest/gtest.h>
#include <cstdint>
#include <memory>
#include <vector>
#include "audition_nav2/acoustic_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
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
