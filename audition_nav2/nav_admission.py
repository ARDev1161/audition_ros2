"""M5 fail-closed admission policy for *optional* navigation soft costs.

Pure Python contract for CI and ROS-independent testing. A bearing, AV pairing,
radar candidate or diagnostic 2D intersection is NOT a metric localized source.
No conversion from those representations is provided here.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MetricSource:
    source_id: str
    frame_id: str
    stamp_ns: int
    expires_ns: int
    xyz_m: tuple
    covariance_xyz: tuple
    physically_validated: bool = False


def eligible(source, now_ns, *, max_variance_m2=1.0):
    """Return True only for bounded, independently validated metric evidence."""
    if not isinstance(source, MetricSource) or type(now_ns) is not int or now_ns <= 0:
        return False
    if not source.physically_validated:
        return False
    if not isinstance(source.source_id, str) or not source.source_id:
        return False
    if (not isinstance(source.frame_id, str) or not source.frame_id
            or source.frame_id.startswith('/')):
        return False
    if (type(source.stamp_ns) is not int or type(source.expires_ns) is not int
            or source.stamp_ns <= 0 or source.stamp_ns > now_ns
            or source.expires_ns <= now_ns or source.expires_ns <= source.stamp_ns):
        return False
    if (not isinstance(source.xyz_m, tuple) or len(source.xyz_m) != 3
            or not isinstance(source.covariance_xyz, tuple)
            or len(source.covariance_xyz) != 9):
        return False
    values = source.xyz_m + source.covariance_xyz
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        return False
    c = source.covariance_xyz
    if any(c[i] < 0 or c[i] > max_variance_m2 for i in (0, 4, 8)):
        return False
    # Conservative symmetry and positive-semidefinite 2x2 principal minors.
    if any(abs(c[i]-c[j]) > 1e-7 for i,j in ((1,3),(2,6),(5,7))):
        return False
    if any(c[i]*c[j] < c[k]*c[k]-1e-9 for i,j,k in ((0,4,1),(0,8,2),(4,8,5))):
        return False
    # Pairwise minors alone do not guarantee a 3x3 covariance is PSD.
    determinant = (c[0]*(c[4]*c[8]-c[5]*c[7])
                   - c[1]*(c[3]*c[8]-c[5]*c[6])
                   + c[2]*(c[3]*c[7]-c[4]*c[6]))
    if determinant < -1e-9:
        return False
    return True


def soft_cost(source, now_ns, *, maximum_cost=100):
    """Optional bounded soft cost; None means no cell update, never lethal."""
    if not eligible(source, now_ns):
        return None
    if type(maximum_cost) is not int or not 1 <= maximum_cost <= 252:
        raise ValueError('maximum_cost must be a nonlethal Nav2 cost')
    return maximum_cost
