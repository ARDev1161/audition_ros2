"""Admission of a calibrated ROS bearing into M4 conditional planar evidence.

No inference of range or emitter position. Caller guarantees that TF2
and measured microphone pose agree, and that angular variance represents
the actual calibrated measurement. Correlated SLAM/extrinsic uncertainty
and multipath are NOT modeled. Return None if any requirement is unknown.
"""
import math

from ros_pose_adapter import _yaw_if_planar
from uncertainty import PlanarRay


def planar_ray_from_observation(source, transform, pose_match, *,
                                expected_stamp_ns,
                                maximum_elevation_rad=0.2,
                                maximum_angular_variance_rad2=0.1):
    """Return normalized world-XY ray if geometric, timing and covariance gates pass."""
    if (type(expected_stamp_ns) is not int or expected_stamp_ns <= 0
            or not 0 < maximum_elevation_rad <= 0.6
            or not 0 < maximum_angular_variance_rad2 <= 0.3):
        return None
    try:
        if (not source.has_direction or not source.has_angular_variance
                or source.state != 0
                or not source.source_id or source.source_generation <= 0
                or pose_match.sample.stamp_ns <= 0
                or expected_stamp_ns != pose_match.sample.stamp_ns + pose_match.skew_ns
                and expected_stamp_ns != pose_match.sample.stamp_ns - pose_match.skew_ns):
            return None
        v = source.direction_unit
        vx, vy, vz = float(v.x), float(v.y), float(v.z)
        variance = float(source.angular_variance_rad2)
        if (not all(math.isfinite(x) for x in (vx, vy, vz, variance))
                or not 0 < variance <= maximum_angular_variance_rad2
                or abs(vx * vx + vy * vy + vz * vz - 1.0) > 0.02):
            return None
        yaw = _yaw_if_planar(transform.transform.rotation)
        if yaw is None:
            return None
        # A planar world ray from calibrated microphone-frame DOA.
        cy, sy = math.cos(yaw), math.sin(yaw)
        dx, dy = cy * vx - sy * vy, sy * vx + cy * vy
        norm = math.hypot(dx, dy)
        if norm < 0.1 or abs(math.atan2(vz, norm)) > maximum_elevation_rad:
            return None
        o = pose_match.sample.origin
        covariance = tuple(float(x) for x in o.covariance)
        if len(covariance) != 4:
            return None
        return PlanarRay(expected_stamp_ns,
                         (float(o.x_m), float(o.y_m)),
                         (dx / norm, dy / norm),
                         variance, covariance)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
