"""Conservative planar microphone-origin covariance from a robot pose.

The robot pose covariance must be measured in the same world XY frame as
the pose (x, y, yaw), and must cover x/y/yaw *joint* uncertainty.
A static calibrated base-to-microphone lever arm is required. This propagates
first-order pose uncertainty only: extrinsic calibration error, cross-time
SLAM correlation, clock skew and audio angular uncertainty remain independent
required inputs. No ROS position or Nav2 cost is produced.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    x_m: float
    y_m: float
    yaw_rad: float
    # row-major symmetric (x, y, yaw) covariance, SI units
    covariance: tuple


@dataclass(frozen=True)
class MicrophoneOrigin:
    x_m: float
    y_m: float
    covariance: tuple  # xx, xy, yx, yy


def _finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def _psd3(c):
    """Check every principal minor for a symmetric positive-semidefinite 3x3."""
    if not isinstance(c, (tuple, list)) or len(c) != 9 or not _finite(c):
        return False
    scale = max(1.0, *(abs(v) for v in c))
    tolerance = 1e-12 * scale
    if (abs(c[1] - c[3]) > tolerance
            or abs(c[2] - c[6]) > tolerance
            or abs(c[5] - c[7]) > tolerance):
        return False
    a, b, d, e, f, g = c[0], c[1], c[2], c[4], c[5], c[8]
    if min(a, e, g) < -tolerance:
        return False
    if min(a * e - b * b, a * g - d * d,
           e * g - f * f) < -tolerance * scale:
        return False
    det = a * (e * g - f * f) - b * (b * g - d * f) + d * (b * f - e * d)
    return det >= -tolerance * scale * scale


def project_microphone_origin(pose, lever_arm_xy):
    """Transform a known static base-frame offset and propagate pose covariance.

    Returns None if any geometry, units or uncertainty are unknown/invalid.
    The output is conditional on accurately calibrated fixed extrinsics;
    this function does not pretend the output is an independent observation.
    """
    if (not isinstance(pose, PlanarPose)
            or not isinstance(lever_arm_xy, (tuple, list))
            or len(lever_arm_xy) != 2
            or not _finite((pose.x_m, pose.y_m, pose.yaw_rad,
                            *lever_arm_xy))
            or not _psd3(pose.covariance)
            or not any(v > 0 for v in (
                pose.covariance[0], pose.covariance[4],
                pose.covariance[8]))):
        return None
    lx, ly = lever_arm_xy
    cy, sy = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    dx = cy * lx - sy * ly
    dy = sy * lx + cy * ly
    # Jacobian of world-frame microphone origin wrt (x, y, yaw).
    jx = (1.0, 0.0, -dy)
    jy = (0.0, 1.0, dx)
    c = pose.covariance

    def quadratic(u, v):
        return sum(u[i] * c[3*i + k] * v[k]
                   for i in range(3) for k in range(3))

    xx, xy, yy = quadratic(jx, jx), quadratic(jx, jy), quadratic(jy, jy)
    x, y = pose.x_m + dx, pose.y_m + dy
    if (not _finite((xx, xy, yy, x, y))
            or xx < 0 or yy < 0
            or xx * yy < xy * xy - 1e-12 * max(1.0, xx * yy)):
        return None
    return MicrophoneOrigin(x, y, (xx, xy, xy, yy))
