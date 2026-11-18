#include "audition_nav2/acoustic_world_bounds.hpp"
#include <cassert>
#include <limits>
using namespace audition_nav2;
int main() {
  const GridGeometry g{8, 6, 0.5, -2., -1.};
  auto w = AcousticWorldBounds::from_cells({2, 1, 4, 3}, g);
  assert(w.valid && w.min_x < -1. && w.max_x > 0.);
  assert(w.min_y < -0.5 && w.max_y > 0.5);
  double x0 = -3, y0 = -0.25, x1 = -0.5, y1 = 2;
  assert(AcousticWorldBounds::expand(w, &x0, &y0, &x1, &y1));
  assert(x0 == -3 && y0 < -0.5 && x1 > 0 && y1 == 2);
  // Nav2 initializes the world bounds to its empty sentinel on each update.
  // Acoustic arrival or TTL expiry may be the ONLY dirty layer.
  const double high = std::numeric_limits<double>::max();
  const double low = std::numeric_limits<double>::lowest();
  double empty_x0 = high, empty_y0 = high, empty_x1 = low, empty_y1 = low;
  assert(AcousticWorldBounds::expand(w, &empty_x0, &empty_y0,
                                      &empty_x1, &empty_y1));
  assert(empty_x0 == w.min_x && empty_y0 == w.min_y);
  assert(empty_x1 == w.max_x && empty_y1 == w.max_y);
  assert(AcousticWorldBounds::expand(w, &empty_x0, &empty_y0,
                                      &empty_x1, &empty_y1));
  assert(empty_x0 == w.min_x && empty_x1 == w.max_x);
  // A partial sentinel is not a legitimate Nav2 accumulator.
  double partial_x0 = high, partial_y0 = high, partial_x1 = low,
         partial_y1 = 0.;
  assert(!AcousticWorldBounds::expand(w, &partial_x0, &partial_y0,
                                       &partial_x1, &partial_y1));
  assert(partial_x0 == high && partial_y1 == 0.);
  assert(!AcousticWorldBounds::from_cells({}, g).valid);
  assert(!AcousticWorldBounds::from_cells({0,0,9,3}, g).valid);
  assert(!AcousticWorldBounds::from_cells({8,0,9,1}, g).valid);
  assert(!AcousticWorldBounds::from_cells({1,1,1,2}, g).valid);
  const auto inf = std::numeric_limits<double>::infinity();
  assert(!AcousticWorldBounds::from_cells({0,0,1,1}, {8,6,inf,0,0}).valid);
  assert(!AcousticWorldBounds::from_cells({0,0,1,1}, {8,6,1,inf,0}).valid);
  assert(!AcousticWorldBounds::from_cells({0,0,8,6},
                                         {8,6,1e308,1e308,0}).valid);
  // Fractional cell edges must expand outward, never contract due to
  // floating-point roundoff during Nav2 world-to-map conversion.
  const GridGeometry fractional{10,10,0.1,0.2,-0.3};
  const auto edge = AcousticWorldBounds::from_cells({1,2,4,5}, fractional);
  assert(edge.valid);
  assert(edge.min_x < fractional.origin_x + fractional.resolution);
  assert(edge.max_x > fractional.origin_x + 4 * fractional.resolution);
  assert(edge.min_y < fractional.origin_y + 2 * fractional.resolution);
  assert(edge.max_y > fractional.origin_y + 5 * fractional.resolution);
  assert(!AcousticWorldBounds::from_cells({5,1,4,2}, fractional).valid);
  assert(!AcousticWorldBounds::from_cells({1,5,2,4}, fractional).valid);
  // An outward ULP at DBL_MAX must fail closed instead of producing infinity.
  assert(!AcousticWorldBounds::from_cells({0,0,1,1},
      {1,1,1.,std::numeric_limits<double>::max(),0.}).valid);
  // Reject a corrupt accumulated rectangle without mutating any bound.
  double bad_min_x = 3., bad_min_y = 4., bad_max_x = 2., bad_max_y = 5.;
  assert(!AcousticWorldBounds::expand(w, &bad_min_x, &bad_min_y,
                                      &bad_max_x, &bad_max_y));
  assert(bad_min_x == 3. && bad_min_y == 4. &&
         bad_max_x == 2. && bad_max_y == 5.);
  bad_min_x = 0.; bad_min_y = 8.; bad_max_x = 1.; bad_max_y = 7.;
  assert(!AcousticWorldBounds::expand(w, &bad_min_x, &bad_min_y,
                                      &bad_max_x, &bad_max_y));
  assert(bad_min_y == 8. && bad_max_y == 7.);
  // Every legal single-cell dirty rectangle must cover the entire cell,
  // including the upper edge, for a shifted fractional-resolution map.
  const GridGeometry shifted{37, 29, 0.125, -1.75, 3.375};
  for (unsigned j = 0; j < shifted.height; ++j) {
    for (unsigned i = 0; i < shifted.width; ++i) {
      const auto cell = AcousticWorldBounds::from_cells(
          {i, j, i + 1, j + 1}, shifted);
      assert(cell.valid);
      assert(cell.min_x < shifted.origin_x + i * shifted.resolution);
      assert(cell.min_y < shifted.origin_y + j * shifted.resolution);
      assert(cell.max_x > shifted.origin_x + (i + 1) * shifted.resolution);
      assert(cell.max_y > shifted.origin_y + (j + 1) * shifted.resolution);
    }
  }
  // A malformed caller-provided rectangle must not poison other Nav2 layers.
  // WorldBounds is a public value type; valid=true alone is not trustworthy.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const WorldBounds malformed[] = {
      {nan, 0., 1., 1., true}, {0., nan, 1., 1., true},
      {0., 0., inf, 1., true}, {0., 0., 1., -inf, true},
      {2., 0., 1., 1., true}, {0., 2., 1., 1., true},
      {1., 0., 1., 1., true}, {0., 1., 1., 1., true}};
  for (const auto& invalid : malformed) {
    double min_x = -3., min_y = -2., max_x = 4., max_y = 5.;
    assert(!AcousticWorldBounds::expand(invalid, &min_x, &min_y,
                                        &max_x, &max_y));
    assert(min_x == -3. && min_y == -2. && max_x == 4. && max_y == 5.);
  }
  x0 = inf;
  assert(!AcousticWorldBounds::expand(w, &x0, &y0, &x1, &y1));
  assert(x0 == inf);
  assert(!AcousticWorldBounds::expand(w, nullptr, &y0, &x1, &y1));
}
