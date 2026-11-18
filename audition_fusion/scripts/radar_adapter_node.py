#!/usr/bin/env python3
"""Optional M4 radar-track TF2 adapter. Diagnostic observations only.

No acoustic source identity, costmap obstacle or robot command is produced.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from audition_msgs.msg import RadarTrackArray, RadarObservation, RadarObservationArray
from av_matching import StreamFence
from radar_geometry import transform_track


def ns(stamp):
    return (stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000 else 0)


class RadarAdapterNode(Node):
    def __init__(self):
        super().__init__('radar_adapter_node')
        self.world = self.declare_parameter('world_frame', 'odom').value
        self.age_ns = int(self.declare_parameter(
            'maximum_observation_age_ms', 800).value) * 1_000_000
        self.max_tracks = int(self.declare_parameter('maximum_tracks', 16).value)
        if (not isinstance(self.world, str) or not self.world
                or self.world.startswith('/')
                or not 50_000_000 <= self.age_ns <= 5_000_000_000
                or not 1 <= self.max_tracks <= 64):
            raise ValueError('invalid radar adapter configuration')
        self.guard = StreamFence()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_ros_ns = None
        self.last_arrival = None
        self.had_observations = False
        self.sequence = 0
        self.publisher = self.create_publisher(
            RadarObservationArray, 'fusion/radar_observations', 10)
        self.subscriber = self.create_subscription(
            RadarTrackArray, 'radar/tracks', self.on_tracks, 10)
        self.timer = self.create_timer(.1, self.on_timer)

    def clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.guard.reset()
            self.last_arrival = None
            self.had_observations = False
            self.emit([])
        self.last_ros_ns = now
        return now

    def emit(self, observations, stamp=None):
        output = RadarObservationArray()
        output.header.frame_id = self.world
        output.header.stamp = stamp or self.get_clock().now().to_msg()
        self.sequence += 1
        output.sequence_number = self.sequence
        output.radar_session_id = self.guard.session or ''
        output.observations = observations
        self.publisher.publish(output)
        self.had_observations = bool(observations)

    def on_tracks(self, msg):
        now = self.clock()
        stamp = ns(msg.header.stamp)
        if (stamp <= 0 or stamp > now + 100_000_000
                or now - stamp > self.age_ns
                or not msg.header.frame_id
                or msg.header.frame_id.startswith('/')):
            return
        if not self.guard.accept(msg.radar_session_id, int(msg.sequence_number), stamp):
            return
        self.last_arrival = time.monotonic()
        if len(msg.tracks) > self.max_tracks:
            self.emit([], msg.header.stamp)
            return
        seen = set()
        result = []
        for track in msg.tracks:
            # Duplicate IDs (even with different generation) invalidate the
            # entire snapshot. Missing TF or bad covariance also fails closed.
            if track.radar_track_id in seen:
                self.emit([], msg.header.stamp)
                return
            seen.add(track.radar_track_id)
            if not track.active:
                continue
            observed = ns(track.observed_at)
            if (not track.radar_track_id
                    or len(track.radar_track_id.encode('utf-8')) > 256
                    or track.radar_track_generation <= 0
                    or observed <= 0 or observed > stamp
                    or now - observed > self.age_ns):
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world, msg.header.frame_id,
                    Time.from_msg(track.observed_at),
                    timeout=Duration(seconds=0.0))
            except TransformException:
                continue
            q = tf.transform.rotation
            t = tf.transform.translation
            p = track.position_m
            converted = transform_track(
                (p.x,p.y,p.z),
                tuple(track.position_covariance_m2),
                (q.x,q.y,q.z,q.w),
                (t.x,t.y,t.z))
            if converted is None:
                continue
            xyz,cov = converted
            observation = RadarObservation()
            observation.radar_session_id = msg.radar_session_id
            observation.radar_track_id = track.radar_track_id
            observation.radar_track_generation = track.radar_track_generation
            observation.observed_at = track.observed_at
            observation.position_m.x = xyz[0]
            observation.position_m.y = xyz[1]
            observation.position_m.z = xyz[2]
            observation.conditional_position_covariance_m2 = cov
            result.append(observation)
        self.emit(result, msg.header.stamp)

    def on_timer(self):
        self.clock()
        if (self.had_observations and self.last_arrival is not None
                and time.monotonic() - self.last_arrival > self.age_ns/1e9):
            self.emit([])


def main():
    rclpy.init()
    node = RadarAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
