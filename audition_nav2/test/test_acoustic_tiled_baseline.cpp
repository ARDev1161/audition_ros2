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
  assert(stage.add_checked(t,0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  assert(!stage.add_checked(t,0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  assert(stage.begin(t));
  assert(stage.add_checked(t, 0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.add_checked(t,256, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.ready(t));
  auto stale = t; stale.cycle = 2;
  assert(!stage.ready(stale));
  stale = t; stale.generation = 2;
  assert(!stage.ready(stale));
  stale = t; stale.geometry.origin_x = 0.05;
  assert(!stage.ready(stale));
  // Partial and mismatched transactions must never expose any baseline.
  std::vector<std::uint8_t> extracted{42};
  assert(!stage.take_ready(stale, extracted));
  assert(extracted.size() == 1 && extracted[0] == 42);
  assert(stage.take_ready(t, extracted));
  assert(extracted.size() == 512u * 256u);
  assert(extracted.front() == 254 && extracted.back() == 254);
  assert(!stage.take_ready(t, extracted));
  assert(!stage.ready(t));
  stage.reset();
  assert(!stage.ready(t));
  assert(stage.begin(t));
  marks[1] = 0;
  assert(!stage.add_checked(t,0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  marks[1] = 1;
  assert(stage.begin(t));
  assert(stage.add_checked(t,0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(!stage.add_checked(t,256, 0, 256, 256, costs.data(), marks.data(), 65535));
  assert(!stage.ready(t));
  assert(stage.begin(t));
  assert(stage.add_checked(t, 0, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.add_checked(t,256, 0, 256, 256, costs.data(), marks.data(), costs.size()));
  assert(stage.ready(t));
  std::vector<std::uint8_t> final_baseline;
  assert(stage.take_ready(t, final_baseline));
  assert(final_baseline.size() == 512u * 256u);
  assert(!stage.take_ready(t, final_baseline));
  auto bad = t; bad.origin = BaselineOrigin::PreviousMaster;
  assert(!stage.begin(bad));
  assert(!stage.ready(t));
  bad = t; bad.geometry.resolution = 0;
  assert(!stage.begin(bad));
  bad = t; bad.geometry.width = 4001; bad.geometry.height = 1000;
  assert(!stage.begin(bad));
  // An asynchronous tile from an old cycle must revoke the transaction.
  assert(stage.begin(t));
  assert(stage.add_checked(t, 0, 0, 256, 256,
                           costs.data(), marks.data(), costs.size()));
  auto old_cycle = t;
  old_cycle.cycle = 2;
  assert(!stage.add_checked(old_cycle, 256, 0, 256, 256,
                            costs.data(), marks.data(), costs.size()));
  assert(!stage.matches(t) && !stage.ready(t));
  assert(!stage.add_checked(t, 256, 0, 256, 256,
                            costs.data(), marks.data(), costs.size()));
  // Rolling origin ABA: equal dimensions do not make a tile reusable.
  assert(stage.begin(t));
  auto moved = t;
  moved.geometry.origin_x = 0.1;
  assert(!stage.add_checked(moved, 0, 0, 256, 256,
                            costs.data(), marks.data(), costs.size()));
  assert(!stage.ready(t));
  // A fresh transaction can still complete after an invalidated producer.
  assert(stage.begin(t));
  assert(stage.add_checked(t, 0, 0, 256, 256,
                           costs.data(), marks.data(), costs.size()));
  assert(stage.add_checked(t, 256, 0, 256, 256,
                           costs.data(), marks.data(), costs.size()));
  assert(stage.ready(t));

}
