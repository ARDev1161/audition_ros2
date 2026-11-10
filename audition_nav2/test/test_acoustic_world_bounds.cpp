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
  x0 = inf;
  assert(!AcousticWorldBounds::expand(w, &x0, &y0, &x1, &y1));
  assert(x0 == inf);
  assert(!AcousticWorldBounds::expand(w, nullptr, &y0, &x1, &y1));
}
