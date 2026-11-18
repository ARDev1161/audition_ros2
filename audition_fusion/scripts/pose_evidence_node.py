#!/usr/bin/env python3
"""Opt-in M4 ROS pose/DOA timing diagnostics, not source localization."""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from audition_msgs.msg import AcousticSourceArray, AcousticSource

from pose_sample_cache import PoseSampleCache
from ros_pose_adapter import from_ros_pose_message
from scene_admission import SceneAdmission
from tf_pose_consistency import TransformOrigin, consistent_origin
from ray_evidence import planar_ray_from_observation
from evidence_buffer import DiagnosticEvidenceBuffer


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
        self.require_tf = self.declare_parameter('require_tf_consistency', False).value
        self.microphone_frame = self.declare_parameter(
            'microphone_frame', 'microphone_array').value
        self.maximum_tf_difference_m = float(self.declare_parameter(
            'maximum_tf_pose_disagreement_m', 0.25).value)
        if (type(self.require_tf) is not bool or
                not isinstance(self.microphone_frame, str)
                or not self.microphone_frame or self.microphone_frame.startswith('/')
                or not math.isfinite(self.maximum_tf_difference_m)
                or not 0 < self.maximum_tf_difference_m <= 2.0):
            raise ValueError('invalid M4 TF consistency limits')
        self.enable_evidence = self.declare_parameter(
            'enable_conditional_evidence', False).value
        self.extrinsics_calibrated = self.declare_parameter(
            'microphone_extrinsics_calibrated', False).value
        if (type(self.enable_evidence) is not bool
                or type(self.extrinsics_calibrated) is not bool
                or (self.enable_evidence and
                    (not self.require_tf or not self.extrinsics_calibrated))):
            raise ValueError('conditional rays require TF and explicit calibration')
        self.arm = tuple(arm)
        if self.require_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.evidence = DiagnosticEvidenceBuffer()
        self.conditional_evaluated = 0
        self.conditional_admitted = 0
        self.conditional_rejected = 0
        self.cache = PoseSampleCache(capacity=capacity,
                                     maximum_skew_ns=skew_ns,
                                     maximum_age_ns=self.age_ns)
        self.accepted = 0
        self.rejected = 0
        self.adapter_rejected = 0
        self.cache_rejected = 0
        self.matches = 0
        self.unmatched = 0
        self.tf_rejected = 0
        self.last_clock = 0
        self.scene_guard = SceneAdmission()
        self.replayed_scenes = 0
        self.session_restarts = 0
        self.clock_resets = 0
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
            self.scene_guard.reset()
            self.evidence.reset()
            self.clock_resets += 1
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
        admission = self.scene_guard.begin(
            array.scene_session_id, int(array.sequence_number), scene_stamp)
        if not admission.accepted:
            self.replayed_scenes += 1
            return
        if admission.restarted:
            # Pose samples from the previous capture incarnation are not
            # reassigned to a new stream by timestamp coincidence alone.
            self.cache.reset()
            self.evidence.reset()
            self.session_restarts += 1
        active_evidence = set()
        for source in array.sources:
            if (source.state != AcousticSource.ACTIVE
                    or not source.has_direction or not source.source_id):
                continue
            stamp = ns(source.last_seen)
            if (stamp <= 0 or stamp > scene_stamp
                    or now - stamp > self.age_ns):
                self.unmatched += 1
                continue
            pose_match = self.cache.match(stamp, now)
            if pose_match is None:
                self.unmatched += 1
                continue
            if self.require_tf:
                # Zero timeout: never block the executor or extrapolate TF.
                # A missing/mismatched transform fails closed.
                if array.header.frame_id != self.microphone_frame:
                    self.tf_rejected += 1
                    continue
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.frame, self.microphone_frame,
                        Time.from_msg(source.last_seen),
                        timeout=Duration(seconds=0.0))
                    origin = TransformOrigin(
                        transform.header.frame_id, transform.child_frame_id,
                        ns(transform.header.stamp),
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y))
                except (TransformException, ValueError, TypeError):
                    self.tf_rejected += 1
                    continue
                if not consistent_origin(
                        pose_match.sample, origin,
                        expected_world=self.frame,
                        expected_child=self.microphone_frame,
                        maximum_skew_ns=self.cache.maximum_skew_ns,
                        maximum_disagreement_m=self.maximum_tf_difference_m):
                    self.tf_rejected += 1
                    continue
            # A match is diagnostic timestamp/TF agreement only.
            self.matches += 1
            if self.enable_evidence:
                if (not source.has_angular_variance
                        or source.header.frame_id not in ('', self.microphone_frame)
                        or source.source_generation <= 0):
                    self.conditional_rejected += 1
                    continue
                ray = planar_ray_from_observation(
                    source, transform, pose_match, expected_stamp_ns=stamp)
                if ray is None:
                    self.conditional_rejected += 1
                    continue
                identity = (array.scene_session_id, source.source_id,
                            int(source.source_generation))
                active_evidence.add(identity)
                outcome = self.evidence.add(*identity, ray)
                if outcome.evaluated:
                    self.conditional_evaluated += 1
                    if outcome.admitted:
                        self.conditional_admitted += 1
                    else:
                        self.conditional_rejected += 1
        if self.enable_evidence:
            self.evidence.retire_missing(active_evidence)

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
            ('tf_rejected_bearings', self.tf_rejected),
            ('require_tf_consistency', self.require_tf),
            ('enable_conditional_evidence', self.enable_evidence),
            ('conditional_evaluated', self.conditional_evaluated),
            ('conditional_admitted', self.conditional_admitted),
            ('conditional_rejected', self.conditional_rejected),
            ('replayed_scenes', self.replayed_scenes),
            ('session_restarts', self.session_restarts),
            ('clock_resets', self.clock_resets),
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
