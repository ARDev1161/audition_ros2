#include <cassert>
#include <cstdint>
#include "audition_nav2/acoustic_baseline_gate.hpp"

int main() {
  using namespace audition_nav2;
  const GridGeometry grid{16, 12, 0.25, -2.0, 1.0};
  AcousticBaselineGate gate;
  BaselineTicket ticket{7, 1, BaselineOrigin::IndependentNonAcousticLayers, grid};
  assert(gate.accept(ticket, 7, grid));
  assert(gate.active());
  // Reusing a previous master or a stale snapshot is never an independent rebuild.
  assert(!gate.accept(ticket, 7, grid));
  assert(!gate.active());
  // Invalidating must not permit replay of the same ticket.
  gate.invalidate();
  assert(!gate.accept(ticket, 7, grid));
  ticket.cycle = 2;
  ticket.origin = BaselineOrigin::PreviousMaster;
  assert(!gate.accept(ticket, 7, grid));
  ticket.origin = BaselineOrigin::Unknown;
  assert(!gate.accept(ticket, 7, grid));
  ticket.origin = BaselineOrigin::IndependentNonAcousticLayers;
  ticket.cycle = 3;
  assert(gate.accept(ticket, 7, grid));
  gate.invalidate();
  assert(!gate.accept(ticket, 7, grid));
  ticket.cycle = 4;
  assert(gate.accept(ticket, 7, grid));
  // Geometry change must be rejected even if the generation token is stale.
  auto shifted = grid;
  shifted.origin_x += grid.resolution;
  ticket.cycle = 5;
  assert(!gate.accept(ticket, 7, shifted));
  // A fresh generation after rolling shift requires an explicit reset.
  gate.invalidate();
  ticket.generation = 8;
  ticket.geometry = shifted;
  ticket.cycle = 1;
  assert(gate.accept(ticket, 8, shifted));
  // A generation rollback is forbidden, even with a greater cycle.
  gate.invalidate();
  ticket.generation = 7;
  ticket.cycle = 999;
  ticket.geometry = grid;
  assert(!gate.accept(ticket, 7, grid));
  ticket.generation = 8;
  ticket.cycle = 2;
  ticket.geometry = shifted;
  assert(gate.accept(ticket, 8, shifted));
  // ABA geometry return cannot resurrect the old generation.
  gate.invalidate();
  ticket.generation = 7;
  ticket.geometry = grid;
  ticket.cycle = 5;
  assert(!gate.accept(ticket, 8, grid));
  // Missing generation, zero cycle, invalid geometry all fail closed.
  ticket.generation = 0;
  assert(!gate.accept(ticket, 0, grid));
  ticket.generation = 8;
  ticket.cycle = 0;
  assert(!gate.accept(ticket, 8, grid));
  ticket.cycle = 2;
  ticket.geometry = grid;
  auto bad = grid;
  bad.resolution = 0.0;
  assert(!gate.accept(ticket, 8, bad));
  return 0;
}
