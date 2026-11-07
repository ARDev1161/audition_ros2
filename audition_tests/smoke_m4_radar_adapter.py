#!/usr/bin/env python3
"""M4-C radar TF2 adaptor DDS smoke without a radar device."""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from audition_msgs.msg import RadarTrack, RadarTrackArray, RadarObservationArray

NS = '/robot_ci/radar/'
SESSION = 'e' * 32


def main():
    rclpy.init()
    node = Node('m4_radar_probe')
    pub = node.create_publisher(RadarTrackArray, NS+'radar/tracks', 10)
    tfpub = TransformBroadcaster(node)
    observed = []
    sub = node.create_subscription(
        RadarObservationArray, NS+'fusion/radar_observations',
        observed.append, 10)

    def until(predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False

    def snapshot(seq, stamp, *, session=SESSION, tracks=1):
        msg = RadarTrackArray()
        msg.header.frame_id = 'robot_ci_radar_frame'
        msg.header.stamp = stamp
        msg.radar_session_id = session
        msg.sequence_number = seq
        if tracks:
            t = RadarTrack()
            t.radar_track_id = 'range-1'
            t.radar_track_generation = 1
            t.active = True
            t.position_m.x = 2.
            t.position_covariance_m2 = [
                .04, 0., 0., 0., .09, 0., 0., 0., .16]
            t.observed_at = stamp
            msg.tracks = [t]
        return msg

    try:
        assert until(lambda: pub.get_subscription_count() >= 1
                     and tfpub.pub_tf.get_subscription_count() >= 1), (
                         'radar adapter not discovered')
        # Prime TF subscriber before its exact-time observation is issued.
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id = 'odom'
        tf.header.stamp = stamp
        tf.child_frame_id = 'robot_ci_radar_frame'
        tf.transform.translation.x = 1.
        tf.transform.rotation.w = 1.
        tfpub.sendTransform(tf)
        until(lambda: False, .2)
        stamp = node.get_clock().now().to_msg()
        tf.header.stamp = stamp
        tfpub.sendTransform(tf)
        until(lambda: False, .08)
        pub.publish(snapshot(1, stamp))
        assert until(lambda: any(m.observations for m in observed)), (
            'radar observation not propagated')
        positive = next(m for m in reversed(observed) if m.observations)
        assert positive.header.frame_id == 'odom'
        o = positive.observations[0]
        assert abs(o.position_m.x - 3.) < 1e-7
        assert o.radar_session_id == SESSION
        assert abs(o.conditional_position_covariance_m2[0] - .04) < 1e-8

        # Silence must clear even without an explicit empty radar snapshot.
        assert until(lambda: any(
            m.sequence_number > positive.sequence_number and not m.observations
            for m in observed), timeout=3.), 'radar TTL failed to clear'

        old_count = len(observed)
        stamp = node.get_clock().now().to_msg()
        pub.publish(snapshot(1, stamp))  # duplicate sequence
        until(lambda: False, .12)
        assert len(observed) == old_count, 'duplicate radar snapshot admitted'

        # An authoritative empty snapshot clears without waiting for TTL.
        pub.publish(snapshot(2, node.get_clock().now().to_msg(), tracks=0))
        assert until(lambda: observed[-1].sequence_number >
                     positive.sequence_number+1
                     and not observed[-1].observations), (
                         'empty radar snapshot was not admitted')
        topics = dict(node.get_topic_names_and_types())
        assert NS+'localized_sources' not in topics
        assert NS+'cmd_vel' not in topics
        print('PASS: radar TF2 metric observation, conditional covariance, '
              'TTL, replay and empty snapshot; no Nav2')
        return 0
    except Exception as exc:
        print('FAIL: M4 radar ROS graph:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
