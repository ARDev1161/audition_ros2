"""Fail-closed ROS 2 pose message adapter for diagnostic M4 geometry.

Accepts PoseWithCovarianceStamped and nav_msgs/Odometry by structural fields.
No ROS imports, so negative/format tests run in plain Python. This adapter
does not synchronize TF, validate estimator calibration or publish positions.
"""
from dataclasses import dataclass
import math

from pose_uncertainty import PlanarPose, project_microphone_origin


@dataclass(frozen=True)
class StampedMicrophoneOrigin:
    stamp_ns: int
    frame_id: str
    origin: object


def _time_ns(stamp):
    try:
        sec, nsec = stamp.sec, stamp.nanosec
        if (type(sec) is not int or type(nsec) is not int
                or sec < 0 or not 0 <= nsec < 1_000_000_000):
            return None
        value = sec * 1_000_000_000 + nsec
        return value if value > 0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def _yaw_if_planar(q, *, maximum_tilt_rad=0.05):
    try:
        x, y, z, w = q.x, q.y, q.z, q.w
        values = (x, y, z, w)
        if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
            return None
        norm = math.sqrt(sum(v * v for v in values))
        if not .99 <= norm <= 1.01:
            return None
        x, y, z, w = (v / norm for v in values)
        # Reject significant roll/pitch: XY + yaw alone is then insufficient.
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x*x + y*y))
        sin_pitch = max(-1., min(1., 2 * (w*y - z*x)))
        pitch = math.asin(sin_pitch)
        if abs(roll) > maximum_tilt_rad or abs(pitch) > maximum_tilt_rad:
            return None
        return math.atan2(2 * (w*z + x*y), 1 - 2*(y*y + z*z))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def from_ros_pose_message(message, *, world_frame, lever_arm_xy,
                          now_ns, maximum_age_ns=500_000_000,
                          maximum_future_ns=50_000_000):
    """Return stamped microphone-origin covariance or None.

    Pose coordinates and covariance must already refer to the specified
    world frame; no frame conversion or interpolation is attempted. Input
    odometry covariance refers to pose.header.frame_id (not child_frame_id).
    """
    if (not isinstance(world_frame, str) or not world_frame
            or world_frame.startswith('/')
            or type(now_ns) is not int or now_ns <= 0
            or type(maximum_age_ns) is not int or maximum_age_ns <= 0
            or type(maximum_future_ns) is not int or maximum_future_ns < 0):
        return None
    try:
        header = message.header
        if header.frame_id != world_frame:
            return None
        stamp_ns = _time_ns(header.stamp)
        if (stamp_ns is None or stamp_ns > now_ns + maximum_future_ns
                or now_ns - stamp_ns > maximum_age_ns):
            return None
        # PoseWithCovarianceStamped.pose and Odometry.pose have the same
        # PoseWithCovariance shape.
        pose_message = message.pose
        pose = pose_message.pose
        covariance = pose_message.covariance
        if len(covariance) != 36:
            return None
        # x/y/yaw entries in ROS REP-103 row-major 6x6 pose covariance.
        indices = (0, 1, 5)
        joint = tuple(float(covariance[6*r+c]) for r in indices for c in indices)
        yaw = _yaw_if_planar(pose.orientation)
        if yaw is None:
            return None
        robot_pose = PlanarPose(
            float(pose.position.x), float(pose.position.y), float(yaw), joint)
        arm = tuple(float(v) for v in lever_arm_xy)
        origin = project_microphone_origin(robot_pose, arm)
        return (StampedMicrophoneOrigin(stamp_ns, world_frame, origin)
                if origin is not None else None)
    except (AttributeError, TypeError, IndexError, ValueError, OverflowError):
        return None
