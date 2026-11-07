"""Conservative temporal stability gate for planar M4 diagnostics.

Independent chronological groups of already validated diagnostic intersections
must agree. This is NOT proof that a source is stationary or that rays are
direct-path: common reflections and correlated pose errors remain possible.
No Nav2 or metric position publication may consume this alone.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Fix:
    session: str
    source_id: str
    generation: int
    first_stamp_ns: int
    last_stamp_ns: int
    x: float
    y: float
    major_std_m: float


@dataclass(frozen=True)
class Stability:
    x: float
    y: float
    disagreement_m: float
    temporal_separation_ns: int


def compare_disjoint_fixes(earlier, later, *,
                           minimum_separation_ns=200_000_000,
                           maximum_disagreement_m=0.5,
                           maximum_combined_sigma=3.0):
    """Reject overlap, identity changes and contradictory temporal solutions.

    Inputs are *independently obtained* fixes with no reused observations.
    Their major standard deviations are externally validated conservative
    uncertainty bounds. No estimation or calibration is attempted here.
    """
    if (not isinstance(earlier, Fix) or not isinstance(later, Fix)
            or type(minimum_separation_ns) is not int
            or minimum_separation_ns <= 0
            or not all(type(x) in (int, float) and math.isfinite(x)
                       for x in (maximum_disagreement_m, maximum_combined_sigma))
            or maximum_disagreement_m <= 0 or maximum_combined_sigma <= 0):
        return None
    if (not earlier.session or earlier.session != later.session
            or not earlier.source_id or earlier.source_id != later.source_id
            or earlier.generation <= 0 or earlier.generation != later.generation):
        return None
    for fix in (earlier, later):
        if (type(fix.first_stamp_ns) is not int
                or type(fix.last_stamp_ns) is not int
                or fix.first_stamp_ns <= 0
                or fix.last_stamp_ns <= fix.first_stamp_ns
                or not all(type(x) in (int, float) and math.isfinite(x)
                           for x in (fix.x, fix.y, fix.major_std_m))
                or fix.major_std_m <= 0):
            return None
    separation = later.first_stamp_ns - earlier.last_stamp_ns
    if separation < minimum_separation_ns:
        return None
    disagreement = math.hypot(later.x - earlier.x, later.y - earlier.y)
    sigma = math.hypot(earlier.major_std_m, later.major_std_m)
    if (not math.isfinite(disagreement) or not math.isfinite(sigma)
            or disagreement > maximum_disagreement_m
            or disagreement > maximum_combined_sigma * sigma):
        return None
    # Equal-weight midpoint is a diagnostic consensus, not a fused posterior.
    return Stability((earlier.x + later.x) / 2,
                     (earlier.y + later.y) / 2,
                     disagreement, separation)
