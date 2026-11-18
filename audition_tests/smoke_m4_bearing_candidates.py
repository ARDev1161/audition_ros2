#!/usr/bin/env python3
"""Live M4 DDS + timestamped TF2 acceptance without hardware or navigation."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from audition_msgs.msg import (AcousticSource, AcousticSourceArray,
                               BearingIntersectionCandidateArray)

NS = '/robot_ci/m4/'
SESSION = '0123456789abcdef0123456789abcdef'


def main():
    rclpy.init()
    node = Node('fusion_graph_probe')
    tf_publisher = TransformBroadcaster(node)
    publisher = node.create_publisher(AcousticSourceArray, NS+'sources', 10)
    observed = []
    node.create_subscription(
        BearingIntersectionCandidateArray, NS+'fusion/bearing_candidates',
        lambda m: observed.append(m), 10)

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.04)

    def wait_count(n):
        until = time.monotonic() + 5.0
        while len(observed) < n and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(observed) >= n, ('missing candidate snapshot', n, len(observed))

    def send(seq, origin_x, *, active=True):
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id = 'odom'
        tf.header.stamp = stamp
        tf.child_frame_id = 'robot_ci_mic'
        tf.transform.translation.x = float(origin_x)
        tf.transform.rotation.w = 1.0
        tf_publisher.sendTransform(tf)
        # Allow the subscriber's TF buffer to consume the exact-stamp pose.
        spin_for(0.28)
        scene = AcousticSourceArray()
        scene.header.frame_id = 'robot_ci_mic'
        scene.header.stamp = stamp
        scene.scene_session_id = SESSION
        scene.spatial_session_id = 'a'*32
        scene.sequence_number = seq
        scene.generation = 1
        source = AcousticSource()
        source.source_id = 'static-speaker'
        source.source_generation = 1
        source.header.frame_id = 'robot_ci_mic'
        source.header.stamp = stamp
        source.last_seen = stamp
        source.state = AcousticSource.ACTIVE if active else AcousticSource.ENDED
        source.has_direction = active
        if active:
            dx,dy = 3.0-origin_x,2.0
            hyp = math.hypot(dx,dy)
            source.direction_unit.x = dx/hyp
            source.direction_unit.y = dy/hyp
        scene.sources = [source]
        publisher.publish(scene)

    try:
        deadline = time.monotonic() + 15
        while (publisher.get_subscription_count() == 0 or
               tf_publisher.pub_tf.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publisher.get_subscription_count() > 0, 'fusion subscriber not discovered'
        assert tf_publisher.pub_tf.get_subscription_count() > 0, 'TF2 listener not discovered'

        send(1, 0)
        wait_count(1)
        assert not observed[-1].candidates, 'single bearing produced metric candidate'
        send(2, 1)
        wait_count(2)
        assert not observed[-1].candidates, 'only two bearings produced candidate'
        send(3, 2)
        wait_count(3)
        assert len(observed[-1].candidates) == 1, 'missing three-view candidate'
        point = observed[-1].candidates[0]
        assert point.header.frame_id == 'odom'
        assert point.scene_session_id == SESSION
        assert abs(point.x_m-3) < 0.1 and abs(point.y_m-2) < 0.1, (point.x_m,point.y_m)
        assert point.supporting_observations >= 3

        # Retirement must clear the diagnostic snapshot, not retain a stale XY.
        send(4, 2, active=False)
        wait_count(4)
        assert not observed[-1].candidates, 'retired source retained candidate'
        assert observed[-1].sequence_number > observed[-2].sequence_number
        print('PASS: live TF2 stamped three-view candidate, no one/two-view position, retirement clears')
        return 0
    except Exception as exc:
        print('FAIL: M4 candidate ROS graph:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
