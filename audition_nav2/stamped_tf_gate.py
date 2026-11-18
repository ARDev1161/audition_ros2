"""Strict stamped-TF2 boundary for M5 navigation evidence.

This adapter deliberately accepts an injected lookup function, not an unstamped
latest-TF fallback. The caller must implement lookup(target, source, stamp_ns)
with a bounded timeout and raise LookupError on unavailable transforms.
"""
import math
from nav_admission import eligible


class StampedTransformGate:
    def __init__(self, target_frame, lookup, *, max_translation_m=1000.0):
        if not isinstance(target_frame, str) or not target_frame or target_frame.startswith('/'):
            raise ValueError('invalid target frame')
        if not callable(lookup):
            raise ValueError('stamped lookup required')
        if (type(max_translation_m) not in (float, int)
                or not math.isfinite(max_translation_m) or max_translation_m <= 0):
            raise ValueError('invalid translation bound')
        self.target_frame = target_frame
        self.lookup = lookup
        self.max_translation_m = max_translation_m

    def __call__(self, source, now_ns):
        if not eligible(source, now_ns):
            raise ValueError('ineligible metric source')
        # No extrapolation from latest transform: lookup must use observation stamp.
        xy = self.lookup(self.target_frame, source.frame_id, source.stamp_ns, source.xyz_m)
        if not isinstance(xy, (tuple, list)) or len(xy) != 2:
            raise ValueError('invalid transformed position')
        if not all(type(v) in (int, float) and math.isfinite(v)
                   and abs(v) <= self.max_translation_m for v in xy):
            raise ValueError('out-of-bounds transformed position')
        return tuple(xy)
