#!/usr/bin/env python3
"""Opt-in M4-D radar/acoustic hypothesis graph; diagnostic only.

No speaker identity, acoustic distance, LocalizedSource, costmap or motion.
"""
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from audition_msgs.msg import (
    AcousticSource, AcousticSourceArray, RadarObservationArray,
    RadarAudioCandidate, RadarAudioCandidateArray)
from av_matching import Ray, StreamFence
from radar_bearing_matching import RadarPoint, RadarBearingMatcher
from radar_temporal import RadarPairTemporalGate
from radar_geometry import quaternion_rotation


def stamp_ns(stamp):
    return (stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000 else 0)


class RadarAudioAssociationNode(Node):
    def __init__(self):
        super().__init__('radar_audio_association_node')
        self.world = self.declare_parameter('world_frame', 'odom').value
        self.age_ns = int(self.declare_parameter(
            'maximum_observation_age_ms', 800).value) * 1_000_000
        max_tracks = int(self.declare_parameter('maximum_tracks', 16).value)
        self.matcher = RadarBearingMatcher(maximum_tracks=max_tracks)
        self.temporal = RadarPairTemporalGate(maximum_pairs=max_tracks)
        if (not isinstance(self.world, str) or not self.world
                or self.world.startswith('/')
                or not 50_000_000 <= self.age_ns <= 5_000_000_000):
            raise ValueError('invalid radar/audio diagnostics configuration')
        self.audio_fence = StreamFence()
        self.radar_fence = StreamFence()
        self.audio = []
        self.radar = []
        self.audio_stamp = 0
        self.radar_stamp = 0
        self.audio_received = None
        self.radar_received = None
        self.had_candidates = False
        self.last_ros_ns = None
        self.sequence = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            RadarAudioCandidateArray, 'fusion/radar_audio_candidates', 10)
        self.audio_sub = self.create_subscription(
            AcousticSourceArray, 'sources', self.on_audio, 10)
        self.radar_sub = self.create_subscription(
            RadarObservationArray, 'fusion/radar_observations',
            self.on_radar, 10)
        self.timer = self.create_timer(.1, self.on_timer)

    def clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_ros_ns is not None and now < self.last_ros_ns:
            self.audio_fence.reset()
            self.radar_fence.reset()
            self.audio = []
            self.radar = []
            self.audio_stamp = self.radar_stamp = 0
            self.audio_received = self.radar_received = None
            self.temporal.reset()
            self.emit([])
        self.last_ros_ns = now
        return now

    def fresh(self, stamp, now):
        return stamp > 0 and stamp <= now + 100_000_000 and (
            now - stamp <= self.age_ns)

    def on_audio(self, msg):
        now = self.clock()
        stamp = stamp_ns(msg.header.stamp)
        if (not self.fresh(stamp, now)
                or not msg.header.frame_id
                or msg.header.frame_id.startswith('/')
                or not self.audio_fence.accept(
                    msg.scene_session_id, int(msg.sequence_number), stamp)):
            return
        self.audio_stamp = stamp
        self.audio_received = time.monotonic()
        if len(msg.sources) > self.matcher.max_tracks:
            self.audio = []
            self.publish_candidates()
            return
        seen = set()
        incoming = []
        for source in msg.sources:
            if source.source_id in seen:
                incoming = []
                break
            seen.add(source.source_id)
            ts = stamp_ns(source.last_seen)
            if (source.state != AcousticSource.ACTIVE
                    or not source.has_direction
                    or not source.has_angular_variance
                    or source.source_generation <= 0
                    or (source.header.frame_id
                        and source.header.frame_id != msg.header.frame_id)
                    or not self.fresh(ts, now) or ts > stamp):
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world, msg.header.frame_id,
                    Time.from_msg(source.last_seen),
                    timeout=Duration(seconds=0.0))
            except TransformException:
                continue
            q = tf.transform.rotation
            rotation = quaternion_rotation((q.x, q.y, q.z, q.w))
            origin = tf.transform.translation
            d = source.direction_unit
            if rotation is None:
                continue
            direction = tuple(sum(rotation[i][j]*v for j,v in enumerate(
                (d.x,d.y,d.z))) for i in range(3))
            xyz = (origin.x, origin.y, origin.z)
            if not all(math.isfinite(v) for v in (*xyz,*direction)):
                continue
            incoming.append(Ray(
                source.source_id,int(source.source_generation),ts,
                xyz,direction,float(source.angular_variance_rad2)))
        self.audio = incoming
        self.publish_candidates()

    def on_radar(self, msg):
        now = self.clock()
        stamp = stamp_ns(msg.header.stamp)
        if (msg.header.frame_id != self.world
                or not self.fresh(stamp, now)
                or not self.radar_fence.accept(
                    msg.radar_session_id,int(msg.sequence_number),stamp)):
            return
        self.radar_stamp = stamp
        self.radar_received = time.monotonic()
        if len(msg.observations) > self.matcher.max_tracks:
            self.radar = []
            self.publish_candidates()
            return
        incoming = []
        seen = set()
        for observation in msg.observations:
            if observation.radar_track_id in seen:
                incoming = []
                break
            seen.add(observation.radar_track_id)
            ts = stamp_ns(observation.observed_at)
            if (observation.radar_session_id != msg.radar_session_id
                    or observation.radar_track_generation <= 0
                    or not self.fresh(ts, now)
                    or ts > stamp):
                continue
            p = observation.position_m
            incoming.append(RadarPoint(
                observation.radar_track_id,
                int(observation.radar_track_generation),ts,
                (p.x,p.y,p.z),
                tuple(observation.conditional_position_covariance_m2)))
        self.radar = incoming
        self.publish_candidates()

    def publish_candidates(self):
        if (self.audio_received is None or self.radar_received is None
                or time.monotonic()-self.audio_received > self.age_ns/1e9
                or time.monotonic()-self.radar_received > self.age_ns/1e9):
            pairs = []
        else:
            pairs = self.matcher.propose(self.audio,self.radar)
        # Asynchronous DDS can deliver a newer audio frame before matching
        # radar evidence. Preserve bounded history across a skew-only gap,
        # but publish [] until the corresponding radar frame arrives.
        pending = (not pairs and self.audio and self.radar
                   and abs(self.audio_stamp-self.radar_stamp)
                   > self.matcher.skew_ns)
        self.emit(pairs, pending=pending)

    def emit(self, pairs, pending=False):
        evidence = ({} if pending else self.temporal.observe(
            pairs,
            audio_session=self.audio_fence.session,
            radar_session=self.radar_fence.session,
            audio_stamp_ns=self.audio_stamp,
            radar_stamp_ns=self.radar_stamp,
            clock_ns=self.get_clock().now().nanoseconds))
        result = RadarAudioCandidateArray()
        result.header.frame_id = self.world
        result.header.stamp = self.get_clock().now().to_msg()
        self.sequence += 1
        result.sequence_number = self.sequence
        result.audio_scene_session_id = self.audio_fence.session or ''
        result.radar_session_id = self.radar_fence.session or ''
        for pair in pairs:
            item = RadarAudioCandidate()
            item.audio_source_id = pair.audio_id
            item.audio_source_generation = pair.audio_generation
            item.radar_track_id = pair.radar_id
            item.radar_track_generation = pair.radar_generation
            item.radar_range_along_audio_ray_m = pair.range_along_ray_m
            item.lateral_residual_m = pair.lateral_residual_m
            item.normalized_residual = pair.normalized_residual
            item.unique_match = pair.unique_match
            source = next(x for x in self.audio
                          if x.identity == pair.audio_id)
            track = next(x for x in self.radar
                         if x.identity == pair.radar_id)
            item.audio_observed_at.sec = source.stamp_ns//1_000_000_000
            item.audio_observed_at.nanosec = source.stamp_ns%1_000_000_000
            item.radar_observed_at.sec = track.stamp_ns//1_000_000_000
            item.radar_observed_at.nanosec = track.stamp_ns%1_000_000_000
            correlation = evidence.get(self.temporal.key(pair))
            item.supporting_synchronized_observations = (
                correlation.supporting_synchronized_observations
                if correlation else 0)
            item.temporally_correlated = bool(
                correlation and correlation.temporally_correlated)
            result.candidates.append(item)
        self.publisher.publish(result)
        self.had_candidates = bool(pairs)

    def on_timer(self):
        self.clock()
        if (self.had_candidates
                and (self.audio_received is None or self.radar_received is None
                     or time.monotonic()-self.audio_received > self.age_ns/1e9
                     or time.monotonic()-self.radar_received > self.age_ns/1e9)):
            self.audio = []
            self.radar = []
            self.temporal.reset()
            self.emit([])


def main():
    rclpy.init()
    node = RadarAudioAssociationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
