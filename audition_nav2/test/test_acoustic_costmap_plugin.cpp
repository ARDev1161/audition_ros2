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
