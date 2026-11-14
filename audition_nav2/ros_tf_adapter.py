"""ROS 2 Humble TF2 adapter for M5 metric-only acoustic navigation evidence.

This adapter does not subscribe to unvalidated acoustic bearings or publish a
Nav2 costmap. A separately authorized producer must supply MetricSource with
independently established metric range and calibrated uncertainty.

TF is looked up at observation time with a bounded timeout. No latest-TF
fallback, no extrapolation and no audio-derived velocity commands.
"""
import math

from nav_admission import eligible
from stamped_tf_gate import StampedTransformGate


def stamped_tf_lookup(buffer, *, timeout_sec=0.05):
    """Build a lookup(target_frame, source_frame, stamp_ns, xyz_m) callable.

    Imports are delayed so the pure-Python M5 contract tests can run without
    ROS. The supplied buffer is an initialized tf2_ros.Buffer.
    """
    if type(timeout_sec) not in (float, int) or not math.isfinite(timeout_sec) or not 0 < timeout_sec <= 1.0:
        raise ValueError('invalid TF timeout')

    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import TransformException

    def lookup(target_frame, source_frame, stamp_ns, xyz_m):
        try:
            transform = buffer.lookup_transform(
                target_frame, source_frame,
                Time(nanoseconds=stamp_ns),
                timeout=Duration(seconds=float(timeout_sec)),
            )
        except TransformException as exc:
            raise LookupError('stamped TF unavailable') from exc

        # Reject mismatched TF stamps and frames rather than using an
        # accidentally returned latest transform.
        if (transform.header.frame_id != target_frame
                or transform.child_frame_id != source_frame):
            raise LookupError('TF frame mismatch')
        header = transform.header.stamp
        tf_stamp_ns = header.sec * 1_000_000_000 + header.nanosec
        # Static TF edges may legally carry a zero timestamp in ROS 2.
        # lookup_transform was explicitly requested at stamp_ns above.

        t = transform.transform.translation
        q = transform.transform.rotation
        values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite TF')
        norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not 0.99 <= norm2 <= 1.01:
            raise ValueError('invalid TF quaternion')

        # Rotate metric xyz by unit quaternion and translate. This does not
        # turn a bearing-only source into metric evidence.
        x, y, z = xyz_m
        ux, uy, uz = q.x, q.y, q.z
        w = q.w
        dot = ux*x + uy*y + uz*z
        cross_x = uy*z - uz*y
        cross_y = uz*x - ux*z
        rx = 2*dot*ux + (w*w - (ux*ux + uy*uy + uz*uz))*x + 2*w*cross_x
        ry = 2*dot*uy + (w*w - (ux*ux + uy*uy + uz*uz))*y + 2*w*cross_y
        return (rx + t.x, ry + t.y)

    return lookup


class RosMetricTfGate:
    """Callable bridge usable as SoftCostCells.reconcile(transform=...)."""

    def __init__(self, target_frame, buffer, now_ns, *, timeout_sec=0.05):
        self.now_ns = now_ns
        self.gate = StampedTransformGate(
            target_frame, stamped_tf_lookup(buffer, timeout_sec=timeout_sec))

    def __call__(self, source):
        if not eligible(source, self.now_ns):
            raise ValueError('invalid metric evidence')
        return self.gate(source, self.now_ns)
