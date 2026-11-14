"""M5 bounded grid projection for an optional acoustic Nav2 layer.

Pure policy layer: no ROS subscription, no plugin registration, no costmap
mutation. Projection is always relative to the current map geometry. Geometry
changes invalidate all previously owned cells; a future C++ Nav2 plugin must
clear only its own layer buffer and let the master costmap recombine layers.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int
                or not 1 <= self.width <= 65536 or not 1 <= self.height <= 65536):
            raise ValueError("invalid grid dimensions")
        values = (self.resolution_m, self.origin_x_m, self.origin_y_m)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("nonfinite grid geometry")
        if self.resolution_m <= 0:
            raise ValueError("invalid resolution")

    def cell(self, xy):
        if not isinstance(xy, (tuple, list)) or len(xy) != 2:
            return None
        x, y = xy
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
            return None
        col = math.floor((x - self.origin_x_m) / self.resolution_m)
        row = math.floor((y - self.origin_y_m) / self.resolution_m)
        if 0 <= col < self.width and 0 <= row < self.height:
            return (col, row)
        return None


class AcousticGridProjection:
    """Owns a bounded set of grid indices, never the master costmap.

    Caller supplies world-space (x, y, cost) from admitted metric evidence.
    It must call reconcile on each timer tick even when no new ROS messages
    arrive; stale evidence must be filtered upstream by the EvidenceCache.
    """

    def __init__(self, *, max_cells=4096):
        if type(max_cells) is not int or not 1 <= max_cells <= 65536:
            raise ValueError("invalid cell budget")
        self.max_cells = max_cells
        self._geometry = None
        self._owned = {}

    def clear(self):
        removed = tuple(sorted(self._owned))
        self._owned.clear()
        self._geometry = None
        return removed

    def reconcile(self, geometry, observations):
        """Return (owned_cells, removed_cells, geometry_changed).

        On geometry change, removed indices refer to the *previous* geometry,
        not the new grid. The consumer must reset its own layer buffer rather
        than clear those indices in the new geometry. Invalid observations or
        overflow cause a fail-closed empty result.
        """
        if not isinstance(geometry, GridGeometry):
            raise ValueError("invalid geometry")
        changed = geometry != self._geometry
        previous = self._owned
        next_cells = {}
        invalid = False
        for observation in observations:
            if not isinstance(observation, (tuple, list)) or len(observation) != 3:
                invalid = True
                break
            x, y, cost = observation
            if type(cost) is not int or not 1 <= cost <= 252:
                invalid = True
                break
            # Malformed coordinates invalidate the entire update. An ordinary
            # finite point outside the rolling map is simply out of view.
            if (type(x) not in (int, float) or type(y) not in (int, float)
                    or not math.isfinite(x) or not math.isfinite(y)):
                invalid = True
                break
            cell = geometry.cell((x, y))
            if cell is None:
                continue
            if cell not in next_cells and len(next_cells) >= self.max_cells:
                invalid = True
                break
            next_cells[cell] = max(cost, next_cells.get(cell, 0))
        if invalid:
            next_cells = {}
        self._owned = next_cells
        self._geometry = geometry
        removed = tuple(sorted(previous)) if changed else tuple(sorted(previous.keys() - next_cells.keys()))
        return dict(next_cells), removed, changed
