"""M5 nonlethal, reversible soft-cost cell ownership (not a Nav2 plugin).

This is an isolated policy model. A real Nav2 adapter must transform validated
metric evidence at its observation stamp and never overwrite other layers.
"""
import math
from nav_admission import eligible, soft_cost


class SoftCostCells:
    def __init__(self, *, resolution_m=0.1, max_cells=4096, maximum_cost=100):
        if not isinstance(resolution_m, (int, float)) or not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("invalid resolution")
        if type(max_cells) is not int or not 1 <= max_cells <= 65536:
            raise ValueError("invalid cell budget")
        if type(maximum_cost) is not int or not 1 <= maximum_cost <= 252:
            raise ValueError("invalid nonlethal cost")
        self.resolution_m = resolution_m
        self.max_cells = max_cells
        self.maximum_cost = maximum_cost
        self._owned = {}
        self._last_clock = None

    def clear(self):
        """Return cells previously owned, for explicit clearing by the adapter."""
        removed = tuple(sorted(self._owned))
        self._owned.clear()
        self._last_clock = None
        return removed

    def reconcile(self, sources, now_ns, *, transform=None):
        """Return (new_owned_costs, cells_to_clear), fail closed on invalid inputs.

        transform(source) must return a finite (x,y) in the target metric frame,
        using TF at source.stamp_ns. No transform means no admitted evidence.
        """
        previous = self._owned
        next_cells = {}
        # A ROS /clock rewind invalidates all observations from the old epoch.
        # Clear owned cells, but never republish stale evidence in this cycle.
        rewound = (type(now_ns) is int and self._last_clock is not None
                   and now_ns < self._last_clock)
        if type(now_ns) is int and now_ns > 0:
            self._last_clock = now_ns
        else:
            self._last_clock = None
        if not rewound and callable(transform) and type(now_ns) is int and now_ns > 0:
            for source in sources:
                if not eligible(source, now_ns):
                    continue
                try:
                    xy = transform(source)
                    if not isinstance(xy, (tuple, list)) or len(xy) != 2:
                        continue
                    x, y = xy
                    if any(type(v) not in (float, int) or not math.isfinite(v) for v in (x, y)):
                        continue
                    cell = (math.floor(x / self.resolution_m), math.floor(y / self.resolution_m))
                    cost = soft_cost(source, now_ns, maximum_cost=self.maximum_cost)
                    if cell not in next_cells and len(next_cells) >= self.max_cells:
                        # Never publish a truncated ambiguous map.
                        next_cells = {}
                        break
                    next_cells[cell] = max(next_cells.get(cell, 0), cost)
                except (ValueError, TypeError, LookupError, ArithmeticError):
                    continue
        self._owned = next_cells
        return dict(next_cells), tuple(sorted(previous.keys() - next_cells.keys()))
