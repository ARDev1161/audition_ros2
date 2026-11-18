"""M5 bounded advisory action lifecycle: never directly commands robot motion.

Pure policy for a future ROS2 BT/action adapter. Only explicitly authorized
goals may be forwarded by the caller; stale acoustic evidence revokes an
advisory. No cmd_vel or navigation goal is generated here.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Advisory:
    source_id: str
    stamp_ns: int
    expires_ns: int
    confidence: float
    authorized: bool = False


class AdvisoryGate:
    def __init__(self, *, max_age_ns=2_000_000_000, min_confidence=0.7,
                 capacity=16):
        if type(max_age_ns) is not int or not 1 <= max_age_ns <= 30_000_000_000:
            raise ValueError('invalid age')
        if type(min_confidence) not in (int, float) or not 0 <= min_confidence <= 1:
            raise ValueError('invalid confidence')
        if type(capacity) is not int or not 1 <= capacity <= 256:
            raise ValueError('invalid capacity')
        self.max_age_ns = max_age_ns
        self.min_confidence = min_confidence
        self.capacity = capacity
        self._active = {}
        self._last_clock = None

    def clear(self):
        revoked = tuple(sorted(self._active))
        self._active.clear()
        self._last_clock = None
        return revoked

    def tick(self, now_ns):
        """Return (active advisories, revoked IDs) even without new messages."""
        revoked = set()
        if type(now_ns) is not int or now_ns <= 0:
            revoked.update(self.clear())
            return (), tuple(sorted(revoked))
        if self._last_clock is not None and now_ns < self._last_clock:
            revoked.update(self._active)
            self._active.clear()
        self._last_clock = now_ns
        for key, item in tuple(self._active.items()):
            if (item.stamp_ns > now_ns or item.expires_ns <= now_ns
                    or now_ns - item.stamp_ns > self.max_age_ns):
                del self._active[key]
                revoked.add(key)
        return tuple(self._active[k] for k in sorted(self._active)), tuple(sorted(revoked))

    def ingest(self, item, now_ns):
        """Reject replay, untrusted, future, expired and over-capacity events."""
        self.tick(now_ns)
        if not isinstance(item, Advisory) or not item.authorized:
            return False
        if type(item.source_id) is not str or not item.source_id:
            return False
        if (type(item.stamp_ns) is not int or type(item.expires_ns) is not int
                or item.stamp_ns <= 0 or item.stamp_ns > now_ns
                or item.expires_ns <= now_ns or item.expires_ns <= item.stamp_ns
                or now_ns - item.stamp_ns > self.max_age_ns):
            return False
        import math
        if (type(item.confidence) not in (float, int)
                or not math.isfinite(item.confidence)
                or not self.min_confidence <= item.confidence <= 1):
            return False
        old = self._active.get(item.source_id)
        if old is not None and item.stamp_ns <= old.stamp_ns:
            return False
        if old is None and len(self._active) >= self.capacity:
            return False
        self._active[item.source_id] = item
        return True

    def revoke(self, source_id):
        return self._active.pop(source_id, None) is not None
