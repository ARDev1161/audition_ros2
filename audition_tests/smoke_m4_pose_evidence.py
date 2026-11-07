#!/usr/bin/env python3
"""DDS-level M4 pose evidence regression; diagnostic only, no localization."""
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from audition_msgs.msg import AcousticSource, AcousticSourceArray

NAMESPACE = '/robot_ci/pose/'
SESSION = 'ab' * 16


def main():
    rclpy.init()
    node = Node('pose_evidence_graph_probe')
    poses = node.create_publisher(Odometry, NAMESPACE + 'fusion/input_pose', 10)
    sources = node.create_publisher(AcousticSourceArray, NAMESPACE + 'sources', 10)
    diagnostics = []
    node.create_subscription(DiagnosticArray,
                             NAMESPACE + 'fusion/pose_evidence_diagnostics',
                             lambda msg: diagnostics.extend(msg.status), 10)

    def spin_until(predicate, timeout=7.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def counters():
        for status in reversed(diagnostics):
            if status.name == 'audition_fusion/pose_evidence':
                return {value.key: value.value for value in status.values}
        return {}

    def wait_counter(key, value):
        assert spin_until(lambda: int(counters().get(key, 0)) >= value), (
            'counter did not advance', key, value, counters())

    def pose(stamp, *, valid=True):
        m = Odometry()
        m.header.frame_id = 'odom' if valid else 'map'
        m.header.stamp = stamp
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = 1.0
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.04, 0.09, 0.01
        m.pose.covariance = cov
        poses.publish(m)

    def scene(stamp, *, sequence=1, session=SESSION):
        m = AcousticSourceArray()
        m.header.frame_id = 'microphone_array'
        m.header.stamp = stamp
        m.scene_session_id = session
        m.sequence_number = sequence
        s = AcousticSource()
        s.header = m.header
        s.source_id = 'sound'
        s.source_generation = 1
        s.state = AcousticSource.ACTIVE
        s.has_direction = True
        s.last_seen = stamp
        s.direction_unit.x = 1.0
        m.sources = [s]
        sources.publish(m)

    try:
        assert spin_until(lambda: poses.get_subscription_count() > 0
                          and sources.get_subscription_count() > 0
                          and bool(counters())), 'pose evidence node did not start'
        assert counters()['world_frame'] == 'odom'
        assert int(counters()['matching_bearings']) == 0

        # A bearing without any pose sample must not be called a match.
        before = int(counters()['unmatched_bearings'])
        scene(node.get_clock().now().to_msg())
        wait_counter('unmatched_bearings', before + 1)

        # An invalid frame must be rejected rather than admitted.
        rejected = int(counters()['rejected_pose_samples'])
        pose(node.get_clock().now().to_msg(), valid=False)
        wait_counter('rejected_pose_samples', rejected + 1)

        # Send the same timestamp to both publishers: nearest pose match.
        stamp = node.get_clock().now().to_msg()
        accepted = int(counters()['accepted_pose_samples'])
        pose(stamp)
        wait_counter('accepted_pose_samples', accepted + 1)
        matched = int(counters()['matching_bearings'])
        scene(stamp, sequence=2)
        wait_counter('matching_bearings', matched + 1)

        # A replay cannot inflate the matches, even with a good pose.
        replays = int(counters()['replayed_scenes'])
        scene(node.get_clock().now().to_msg(), sequence=2)
        wait_counter('replayed_scenes', replays + 1)
        assert int(counters()['matching_bearings']) == matched + 1

        # New scene incarnation must revoke the previous pose buffer.
        restarts = int(counters()['session_restarts'])
        missing = int(counters()['unmatched_bearings'])
        scene(node.get_clock().now().to_msg(), sequence=1, session='cd' * 16)
        wait_counter('session_restarts', restarts + 1)
        wait_counter('unmatched_bearings', missing + 1)
        assert int(counters()['buffered_pose_samples']) == 0

        # Replayed former incarnation remains permanently inadmissible
        # within bounded retirement history.
        replays = int(counters()['replayed_scenes'])
        scene(node.get_clock().now().to_msg(), sequence=3)
        wait_counter('replayed_scenes', replays + 1)

        # Check the diagnostic contract: nothing is a localized source.
        assert counters()['world_frame'] == 'odom'
        assert int(counters()['buffered_pose_samples']) == 0
        print('PASS: DDS pose admission, replay/session fencing and revoked pose evidence')
        return 0
    except Exception as exc:
        print('FAIL: pose evidence DDS smoke:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
