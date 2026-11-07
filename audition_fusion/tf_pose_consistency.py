"""M4 diagnostic consistency guard between TF2 and measured pose origin.

This is a geometry/provenance check, not an independent position estimate:
TF and Odometry may originate from the same estimator. Reject static TF and
stale/mismatched transforms for a moving microphone, without extrapolation.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TransformOrigin:
    frame_id: str
    child_frame_id: str
    stamp_ns: int
    x_m: float
    y_m: float


def consistent_origin(sample, tf, *, expected_world, expected_child,
                      maximum_skew_ns=50_000_000,
                      maximum_disagreement_m=0.25):
    if (not isinstance(tf, TransformOrigin)
            or not isinstance(expected_world, str) or not expected_world
            or not isinstance(expected_child, str) or not expected_child
            or type(maximum_skew_ns) is not int or maximum_skew_ns < 0
            or type(maximum_disagreement_m) not in (float, int)
            or not math.isfinite(maximum_disagreement_m)
            or maximum_disagreement_m <= 0):
        return False
    try:
        if (sample.frame_id != expected_world
                or tf.frame_id != expected_world
                or tf.child_frame_id != expected_child
                or type(tf.stamp_ns) is not int or tf.stamp_ns <= 0
                or type(sample.stamp_ns) is not int or sample.stamp_ns <= 0
                or abs(tf.stamp_ns - sample.stamp_ns) > maximum_skew_ns):
            return False
        x, y = sample.origin.x_m, sample.origin.y_m
        if not all(type(v) in (int, float) and math.isfinite(v)
                   for v in (x, y, tf.x_m, tf.y_m)):
            return False
        return math.hypot(x - tf.x_m, y - tf.y_m) <= maximum_disagreement_m
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False
