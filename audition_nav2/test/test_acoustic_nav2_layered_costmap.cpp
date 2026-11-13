#include <gtest/gtest.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include "audition_nav2/acoustic_nav2_cycle_adapter.hpp"
#include "audition_nav2/acoustic_world_bounds.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace {
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::Layer;
using nav2_costmap_2d::LayeredCostmap;
using audition_nav2::AcousticCellBuffer;
using audition_nav2::AcousticCycle;
using audition_nav2::AcousticLayerCoordinator;
using audition_nav2::AcousticNav2CycleAdapter;
using audition_nav2::AcousticWorldBounds;
using audition_nav2::GridGeometry;

class FixedObstacleLayer : public Layer {
 public:
  void updateBounds(double, double, double, double* x0, double* y0,
                    double* x1, double* y1) override {
    *x0 = std::min(*x0, 1.);
    *y0 = std::min(*y0, 1.);
    *x1 = std::max(*x1, 4.);
    *y1 = std::max(*y1, 4.);
  }
  void updateCosts(Costmap2D& master, int min_i, int min_j,
                   int max_i, int max_j) override {
    // This simulates a non-acoustic Nav2 source that rebuilds its own
    // obstacles on each update. The acoustic plugin must never erase them.
    if (min_i <= 1 && min_j <= 1 && max_i > 1 && max_j > 1)
      master.setCost(1, 1, 254);
    if (min_i <= 3 && min_j <= 3 && max_i > 3 && max_j > 3)
      master.setCost(3, 3, 175);
  }
  void reset() override {}
  bool isClearable() override { return true; }
};

class MockVerifiedAcousticLayer : public Layer {
 public:
  explicit MockVerifiedAcousticLayer(LayeredCostmap& parent) : parent_(parent) {
    coordinator_.enable();
  }
  bool ingest(std::uint64_t id, std::int64_t observed, std::int64_t expires,
              double x, double y, std::int64_t now) {
    return coordinator_.ingest_validated(
        {id, observed, expires, x, y, true}, now);
  }
  void set_time(std::int64_t now) { now_ns_ = now; }
  void updateBounds(double, double, double, double* x0, double* y0,
                    double* x1, double* y1) override {
    auto* map = parent_.getCostmap();
    const GridGeometry geometry{
        map->getSizeInCellsX(), map->getSizeInCellsY(),
        map->getResolution(), map->getOriginX(), map->getOriginY()};
    cycle_ = coordinator_.tick(now_ns_, geometry);
    if (cycle_.rebuild_required) {
      const auto bounds = AcousticWorldBounds::from_cells(cycle_.dirty, geometry);
      AcousticWorldBounds::expand(bounds, x0, y0, x1, y1);
    }
  }
  void updateCosts(Costmap2D& master, int min_i, int min_j,
                   int max_i, int max_j) override {
    const auto* plugins = parent_.getPlugins();
    const auto* filters = parent_.getFilters();
    const auto allowed = AcousticNav2CycleAdapter::terminal_nonacoustic_baseline(
        plugins, filters, static_cast<const Layer*>(this));
    const GridGeometry g{master.getSizeInCellsX(), master.getSizeInCellsY(),
                         master.getResolution(), master.getOriginX(),
                         master.getOriginY()};
    const std::size_t size =
        static_cast<std::size_t>(g.width) * g.height;
    AcousticNav2CycleAdapter::overlay_after_nav2_reset(
        cycle_, g, min_i, min_j, max_i, max_j,
        master.getCharMap(), size, allowed);
  }
  void reset() override { coordinator_.revoke_all(); }
  bool isClearable() override { return true; }

 private:
  LayeredCostmap& parent_;
  AcousticLayerCoordinator coordinator_;
  AcousticCycle cycle_{};
  std::int64_t now_ns_ = 0;
};
}  // namespace

TEST(Nav2RealUpdateMap, PreservesOtherLayersAndExpiresWithoutNewMessages) {
  LayeredCostmap layered("map", false, false);
  layered.resizeMap(8, 8, 0.5, 0., 0.);
  auto obstacle = std::make_shared<FixedObstacleLayer>();
  auto acoustic = std::make_shared<MockVerifiedAcousticLayer>(layered);
  layered.addPlugin(obstacle);
  layered.addPlugin(acoustic);

  constexpr std::int64_t t0 = 1000000000LL;
  ASSERT_TRUE(acoustic->ingest(1, t0, t0 + 400000000LL,
                              1.25, 1.25, t0 + 10000000LL));
  // One source sits on a physical obstacle and must not reduce its cost.
  ASSERT_TRUE(acoustic->ingest(2, t0, t0 + 400000000LL,
                              0.75, 0.75, t0 + 10000000LL));
  acoustic->set_time(t0 + 10000000LL);
  layered.updateMap(1., 1., 0.);
  const auto* master = layered.getCostmap();
  ASSERT_NE(master, nullptr);
  EXPECT_EQ(master->getCost(2, 2), 100);  // Acoustic-only soft cost.
  EXPECT_EQ(master->getCost(1, 1), 254);  // Physical obstacle untouched.
  EXPECT_EQ(master->getCost(3, 3), 175);  // Existing non-acoustic cost.

  // No acoustic message follows. The normal Nav2 update cycle must
  // reset the acoustic dirty region and reconstruct the obstacle layer.
  acoustic->set_time(t0 + 450000000LL);
  layered.updateMap(1., 1., 0.);
  EXPECT_EQ(master->getCost(2, 2), 0);    // Expired soft cost cleared.
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(3, 3), 175);
}

TEST(Nav2RealUpdateMap, ReorderedPluginCannotOverlayOldMaster) {
  LayeredCostmap layered("map", false, false);
  layered.resizeMap(8, 8, 0.5, 0., 0.);
  auto obstacle = std::make_shared<FixedObstacleLayer>();
  auto acoustic = std::make_shared<MockVerifiedAcousticLayer>(layered);
  layered.addPlugin(acoustic);  // Invalid: acoustic is not the last plugin.
  layered.addPlugin(obstacle);
  constexpr std::int64_t t0 = 1000000000LL;
  ASSERT_TRUE(acoustic->ingest(1, t0, t0 + 400000000LL,
                              1.25, 1.25, t0 + 10000000LL));
  acoustic->set_time(t0 + 10000000LL);
  layered.updateMap(1., 1., 0.);
  EXPECT_EQ(layered.getCostmap()->getCost(2, 2), 0);
  EXPECT_EQ(layered.getCostmap()->getCost(1, 1), 254);
}
