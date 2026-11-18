"""Bounded, clock-aware M5 acoustic soft-cost evidence cache.

Not a costmap plugin: a consumer must still validate TF at observation time,
rasterize conservatively and clear its OWN previously painted cells. Never
copy bearing-only or diagnostic M4 evidence into this cache.
"""
from nav_admission import eligible


class EvidenceCache:
    def __init__(self, *, capacity=32, max_age_ns=2_000_000_000):
        if type(capacity) is not int or not 1 <= capacity <= 256:
            raise ValueError('invalid capacity')
        if type(max_age_ns) is not int or not 1 <= max_age_ns <= 30_000_000_000:
            raise ValueError('invalid age')
        self.capacity = capacity
        self.max_age_ns = max_age_ns
        self._items = {}
        self._last_clock = None

    def clear(self):
        self._items.clear()

    def tick(self, now_ns):
        """Return active evidence; revoke on time rewind, expiry or silence."""
        if type(now_ns) is not int or now_ns <= 0:
            self.clear()
            self._last_clock = None
            return ()
        if self._last_clock is not None and now_ns < self._last_clock:
            self.clear()
        self._last_clock = now_ns
        for key, source in tuple(self._items.items()):
            if (not eligible(source, now_ns)
                    or now_ns - source.stamp_ns > self.max_age_ns):
                del self._items[key]
        return tuple(self._items[key] for key in sorted(self._items))

    def ingest(self, source, now_ns):
        """Reject stale, replayed, unvalidated and over-capacity evidence."""
        self.tick(now_ns)
        if not eligible(source, now_ns):
            return False
        if now_ns - source.stamp_ns > self.max_age_ns:
            return False
        old = self._items.get(source.source_id)
        if old is not None and source.stamp_ns <= old.stamp_ns:
            return False
        if old is None and len(self._items) >= self.capacity:
            return False
        self._items[source.source_id] = source
        return True

    def revoke(self, source_id):
        self._items.pop(source_id, None)
