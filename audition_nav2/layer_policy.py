"""M5 bounded orchestration of admitted metric evidence into owned grid costs.

No ROS subscriptions or master-costmap writes. A ROS/Nav2 layer must call tick()
on a timer, even without incoming messages, and rebuild ONLY its private buffer
from the returned cells. All transforms must be stamped (RosMetricTfGate).
"""
from evidence_cache import EvidenceCache
from grid_projection import AcousticGridProjection
from nav_admission import soft_cost


class AcousticLayerPolicy:
    def __init__(self, *, capacity=32, max_age_ns=2_000_000_000,
                 max_cells=4096, maximum_cost=100):
        if type(maximum_cost) is not int or not 1 <= maximum_cost <= 252:
            raise ValueError("cost must be nonlethal")
        self.evidence = EvidenceCache(capacity=capacity, max_age_ns=max_age_ns)
        self.grid = AcousticGridProjection(max_cells=max_cells)
        self.maximum_cost = maximum_cost
        self._last_clock = None

    def ingest(self, source, now_ns):
        """Accept only independently validated metric evidence."""
        return self.evidence.ingest(source, now_ns)

    def revoke(self, source_id):
        self.evidence.revoke(source_id)

    def clear(self):
        self.evidence.clear()
        self._last_clock = None
        return self.grid.clear()

    def tick(self, geometry, now_ns, *, transform=None):
        """Return (private_costs, old_cells, geometry_changed).

        Transform is mandatory and must fail closed on missing/stale TF.
        If any admitted source cannot be transformed, clear the entire private
        acoustic layer for this tick rather than publishing partial evidence.
        """
        rewind = (type(now_ns) is int and self._last_clock is not None
                  and now_ns < self._last_clock)
        if type(now_ns) is int and now_ns > 0:
            self._last_clock = now_ns
        else:
            self._last_clock = None
        active = self.evidence.tick(now_ns)
        if rewind:
            active = ()
        observations = []
        if active and not callable(transform):
            active = ()
        for source in active:
            try:
                xy = transform(source)
                if not isinstance(xy, (tuple, list)) or len(xy) != 2:
                    raise ValueError("invalid transformed coordinates")
                cost = soft_cost(source, now_ns, maximum_cost=self.maximum_cost)
                observations.append((xy[0], xy[1], cost))
            except (ValueError, TypeError, LookupError, ArithmeticError):
                observations = []
                break
        return self.grid.reconcile(geometry, observations)
