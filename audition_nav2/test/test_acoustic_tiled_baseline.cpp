#include <cassert>
#include <cstdint>
#include <vector>
#include "audition_nav2/acoustic_tiled_baseline.hpp"
using namespace audition_nav2;
int main() {
  const GridGeometry g{512, 256, 0.05, 0, 0};
  const BaselineTicket t{1, 1, BaselineOrigin::IndependentNonAcousticLayers, g};
  AcousticTiledBaseline stage;
  assert(stage.begin(t));
  std::vector<std::uint8_t> costs(65536, 254), marks(65536, 1);
  assert(stage.add(0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  assert(!stage.add(0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.add(256, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.ready(t));
  auto stale = t; stale.cycle = 2;
  assert(!stage.ready(stale));
  stale = t; stale.generation = 2;
  assert(!stage.ready(stale));
  stale = t; stale.geometry.origin_x = 0.05;
  assert(!stage.ready(stale));
  assert(stage.cells().front() == 254 && stage.cells().back() == 254);
  stage.reset();
  assert(!stage.ready(t));
  assert(stage.begin(t));
  marks[1] = 0;
  assert(!stage.add(0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  marks[1] = 1;
  assert(stage.add(0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.add(256, 0, 256, 256, costs.data(), marks.data(), 65535));
  assert(stage.add(256, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.ready(t));
  auto bad = t; bad.origin = BaselineOrigin::PreviousMaster;
  assert(!stage.begin(bad));
  assert(!stage.ready(t));
  bad = t; bad.geometry.resolution = 0;
  assert(!stage.begin(bad));
  bad = t; bad.geometry.width = 4001; bad.geometry.height = 1000;
  assert(!stage.begin(bad));
}
