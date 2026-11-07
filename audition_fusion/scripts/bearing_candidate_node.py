#!/usr/bin/env python3
"""M4 diagnostic world-XY candidates from canonical sources and exact-time TF.

No source distance, metric height, motion, or Nav2 costmap output is inferred.
"""
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from audition_msgs.msg import (AcousticSource, AcousticSourceArray,
                               BearingIntersectionCandidate,
                               BearingIntersectionCandidateArray)
from association import CandidateTracker, Observation


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def rotate_xyz(v, q):
    """Rotate a vector by a unit quaternion; refuse unknown/bad orientation."""
    values = (v.x, v.y, v.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values):
        return None
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if norm < 1e-10 or abs(norm - 1.0) > 0.02:
        return None
    qx,qy,qz,qw = q.x/math.sqrt(norm),q.y/math.sqrt(norm),q.z/math.sqrt(norm),q.w/math.sqrt(norm)
    # v + 2*w*(qvec cross v) + 2*(qvec cross (qvec cross v))
    tx,ty,tz = 2*(qy*v.z-qz*v.y),2*(qz*v.x-qx*v.z),2*(qx*v.y-qy*v.x)
    return (v.x+qw*tx+(qy*tz-qz*ty),
            v.y+qw*ty+(qz*tx-qx*tz),
            v.z+qw*tz+(qx*ty-qy*tx))


class BearingCandidateNode(Node):
    def __init__(self):
        super().__init__('bearing_candidate_node')
        self.frame = self.declare_parameter('world_frame', 'odom').value
        self.max_age_ns = int(self.declare_parameter('maximum_observation_age_ms', 750).value) * 1_000_000
        self.max_elevation = float(self.declare_parameter('maximum_elevation_rad', 0.2).value)
        self.max_variance = float(self.declare_parameter('maximum_angular_variance_rad2', 0.1).value)
        minimum_baseline = float(self.declare_parameter('minimum_baseline_m', 0.5).value)
        minimum_sine = float(self.declare_parameter('minimum_parallax_sine', 0.15).value)
        maximum_residual = float(self.declare_parameter('maximum_ray_residual_m', 0.25).value)
        if (not isinstance(self.frame, str) or not self.frame or self.frame.startswith('/')
                or not 50_000_000 <= self.max_age_ns <= 5_000_000_000
                or not math.isfinite(self.max_elevation) or not 0 < self.max_elevation <= 0.6
                or not math.isfinite(self.max_variance) or not 0 < self.max_variance <= 1):
            raise ValueError('invalid M4 TF / observation configuration')
        self.tracker = CandidateTracker(minimum_baseline=minimum_baseline,
                                        minimum_sine=minimum_sine,
                                        maximum_residual=maximum_residual)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            BearingIntersectionCandidateArray, 'fusion/bearing_candidates', 10)
        self.subscriber = self.create_subscription(
            AcousticSourceArray, 'sources', self.on_sources, 10)
        self.seq_out = 0
        self.last_arrival = None
        self.last_ros_now_ns = None
        self.had_candidates = False
        self.timer = self.create_timer(0.1, self.on_timer)

    def emit(self, stamp, candidates):
        output = BearingIntersectionCandidateArray()
        output.header.frame_id = self.frame
        output.header.stamp = stamp
        self.seq_out += 1
        output.sequence_number = self.seq_out
        output.scene_session_id = self.tracker.session or ''
        for c in candidates:
            msg = BearingIntersectionCandidate()
            msg.header.frame_id = self.frame
            msg.header.stamp.sec = c.last_stamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = c.last_stamp_ns % 1_000_000_000
            msg.scene_session_id = output.scene_session_id
            msg.source_id = c.source_id
            msg.source_generation = c.source_generation
            msg.first_observed.sec = c.first_stamp_ns // 1_000_000_000
            msg.first_observed.nanosec = c.first_stamp_ns % 1_000_000_000
            msg.last_observed.sec = c.last_stamp_ns // 1_000_000_000
            msg.last_observed.nanosec = c.last_stamp_ns % 1_000_000_000
            msg.x_m, msg.y_m = c.x, c.y
            msg.baseline_m = c.baseline_m
            msg.parallax_rad = c.parallax_rad
            msg.maximum_ray_residual_m = c.worst_residual_m
            msg.supporting_observations = c.observations
            output.candidates.append(msg)
        self.publisher.publish(output)
        self.had_candidates = bool(candidates)

    def on_sources(self, array):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_ros_now_ns is not None and now_ns < self.last_ros_now_ns:
            self.tracker = CandidateTracker(
                minimum_baseline=self.tracker.minimum_baseline,
                minimum_sine=self.tracker.minimum_sine,
                maximum_residual=self.tracker.maximum_residual)
            self.had_candidates = False
        self.last_ros_now_ns = now_ns
        scene_ns = stamp_ns(array.header.stamp)
        if (scene_ns == 0 or now_ns == 0
                or scene_ns > now_ns + 100_000_000
                or now_ns - scene_ns > self.max_age_ns
                or not array.header.frame_id or array.header.frame_id.startswith('/')):
            return
        if not self.tracker.begin_scene(array.scene_session_id, array.sequence_number, scene_ns):
            return
        candidates = []
        active = set()
        seen = set()
        for src in array.sources:
            if (src.source_id, src.source_generation) in seen:
                continue
            seen.add((src.source_id, src.source_generation))
            if (src.state != AcousticSource.ACTIVE or not src.has_direction
                    or not src.source_id or src.source_generation <= 0
                    or len(src.source_id.encode('utf-8')) > 256
                    or (src.header.frame_id and src.header.frame_id != array.header.frame_id)):
                continue
            sample_ns = stamp_ns(src.last_seen)
            if (sample_ns == 0 or sample_ns > scene_ns or
                    scene_ns - sample_ns > self.max_age_ns):
                continue
            v = src.direction_unit
            if not all(math.isfinite(x) for x in (v.x,v.y,v.z)):
                continue
            if abs(v.x*v.x+v.y*v.y+v.z*v.z-1.0) > 0.02:
                continue
            if src.has_angular_variance and (
                    not math.isfinite(src.angular_variance_rad2)
                    or src.angular_variance_rad2 < 0
                    or src.angular_variance_rad2 > self.max_variance):
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame, array.header.frame_id,
                    Time.from_msg(src.last_seen), timeout=Duration(seconds=0.0))
            except TransformException:
                continue
            direction = rotate_xyz(v, tf.transform.rotation)
            if direction is None:
                continue
            dxy = math.hypot(direction[0],direction[1])
            origin = (tf.transform.translation.x,tf.transform.translation.y)
            if (not all(math.isfinite(x) for x in origin)
                    or dxy < 0.1 or abs(math.atan2(direction[2],dxy)) > self.max_elevation):
                continue
            active.add((src.source_id,src.source_generation))
            result = self.tracker.observe(Observation(
                src.source_id,src.source_generation,sample_ns,
                origin,(direction[0]/dxy,direction[1]/dxy)))
            if result is not None:
                candidates.append(result)
        self.tracker.retire_missing(active)
        self.last_arrival = time.monotonic()
        self.emit(array.header.stamp, candidates)

    def on_timer(self):
        if self.last_arrival is None or not self.had_candidates:
            return
        if time.monotonic() - self.last_arrival > self.max_age_ns / 1e9:
            self.had_candidates = False
            self.tracker.tracks.clear()
            self.emit(self.get_clock().now().to_msg(), [])


def main():
    rclpy.init()
    node = BearingCandidateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
