#!/usr/bin/env python3
"""Opt-in M4 ROS pose/DOA timing diagnostics, not source localization."""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from audition_msgs.msg import AcousticSourceArray, AcousticSource

from pose_sample_cache import PoseSampleCache
from ros_pose_adapter import from_ros_pose_message, _yaw_if_planar
from pose_uncertainty import PlanarPose, project_microphone_origin


def ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class PoseEvidenceNode(Node):
    def __init__(self):
        super().__init__('pose_evidence_node')
        self.frame = self.declare_parameter('world_frame', 'odom').value
        self.mode = self.declare_parameter('pose_type', 'odometry').value
        arm = self.declare_parameter('microphone_lever_arm_xy_m', [0.0, 0.0]).value
        self.age_ns = int(self.declare_parameter('maximum_pose_age_ms', 500).value) * 1_000_000
        skew_ns = int(self.declare_parameter('maximum_pose_skew_ms', 50).value) * 1_000_000
        capacity = int(self.declare_parameter('pose_buffer_capacity', 32).value)
        if (self.mode not in ('odometry', 'pose_stamped')
                or not isinstance(self.frame, str) or not self.frame
                or self.frame.startswith('/')
                or len(arm) != 2
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in arm)):
            raise ValueError('invalid M4 pose evidence configuration')
        self.arm = tuple(arm)
        self.cache = PoseSampleCache(capacity=capacity,
                                     maximum_skew_ns=skew_ns,
                                     maximum_age_ns=self.age_ns)
        self.accepted = 0
        self.rejected = 0
        self.adapter_rejected = 0
        self.cache_rejected = 0
        self.matches = 0
        self.unmatched = 0
        self.last_clock = 0
        self.publisher = self.create_publisher(DiagnosticArray, 'fusion/pose_evidence_diagnostics', 10)
        pose_cls = Odometry if self.mode == 'odometry' else PoseWithCovarianceStamped
        self.pose_sub = self.create_subscription(pose_cls, 'fusion/input_pose',
                                                 self.on_pose, 10)
        self.sources_sub = self.create_subscription(AcousticSourceArray, 'sources',
                                                    self.on_sources, 10)
        self.timer = self.create_timer(0.25, self.on_timer)

    def clock(self):
        value = self.get_clock().now().nanoseconds
        if self.last_clock and value < self.last_clock:
            self.cache.reset()
        self.last_clock = value
        self.cache.tick(value)
        return value

    def on_pose(self, msg):
        now = self.clock()
        parsed = from_ros_pose_message(
            msg, world_frame=self.frame, lever_arm_xy=self.arm,
            now_ns=now, maximum_age_ns=self.age_ns,
            maximum_future_ns=0)
        if parsed is None:
            # Bounded, non-sensitive numeric rejection diagnostics for CI
            # and misconfigured robot covariance/clock investigations.
            q = msg.pose.pose.orientation
            yaw = _yaw_if_planar(q)
            indices = (0, 1, 5)
            joint = tuple(msg.pose.covariance[6*r+c]
                          for r in indices for c in indices)
            debug_pose = None if yaw is None else project_microphone_origin(
                PlanarPose(msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw, joint), self.arm)
            self.get_logger().warning(
                'pose rejected: frame=%s expected=%s stamp=%d now=%d '
                'yaw=%s covariance=%s projected=%s' % (
                    msg.header.frame_id, self.frame, ns(msg.header.stamp), now,
                    str(yaw), str(joint), str(debug_pose)))
            self.adapter_rejected += 1
            self.rejected += 1
        elif self.cache.add(parsed, now):
            self.accepted += 1
        else:
            self.cache_rejected += 1
            self.rejected += 1

    def on_sources(self, array):
        now = self.clock()
        scene_stamp = ns(array.header.stamp)
        if (not scene_stamp or scene_stamp > now
                or now - scene_stamp > self.age_ns):
            self.unmatched += 1
            return
        for source in array.sources:
            if (source.state != AcousticSource.ACTIVE
                    or not source.has_direction or not source.source_id):
                continue
            stamp = ns(source.last_seen)
            if (stamp <= 0 or stamp > scene_stamp
                    or now - stamp > self.age_ns):
                self.unmatched += 1
                continue
            if self.cache.match(stamp, now) is None:
                self.unmatched += 1
            else:
                # This only establishes same-clock timestamp proximity.
                # It is not source correspondence or metric position.
                self.matches += 1

    def on_timer(self):
        self.clock()
        result = DiagnosticArray()
        result.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'audition_fusion/pose_evidence'
        status.hardware_id = 'm4-diagnostic-only'
        status.level = DiagnosticStatus.WARN
        status.message = 'Timestamp matching only; localization disabled'
        for key, value in (
            ('accepted_pose_samples', self.accepted),
            ('rejected_pose_samples', self.rejected),
            ('adapter_rejected_pose_samples', self.adapter_rejected),
            ('cache_rejected_pose_samples', self.cache_rejected),
            ('matching_bearings', self.matches),
            ('unmatched_bearings', self.unmatched),
            ('buffered_pose_samples', len(self.cache.samples)),
            ('world_frame', self.frame)):
            status.values.append(KeyValue(key=str(key), value=str(value)))
        result.status.append(status)
        self.publisher.publish(result)


def main():
    rclpy.init()
    node = PoseEvidenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
