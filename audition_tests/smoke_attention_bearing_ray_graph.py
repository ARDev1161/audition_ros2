#!/usr/bin/env python3
"""Live ROS DDS verification: arrow, retirement, replay fence and expiry."""
import sys
import time
import rclpy
from rclpy.node import Node
from audition_msgs.msg import AttentionTarget
from visualization_msgs.msg import Marker

NS = '/robot_ci/ray'


def main():
    rclpy.init()
    node = Node('ray_graph_probe')
    pub = node.create_publisher(AttentionTarget, NS + '/attention/target_base', 10)
    received = []
    sub = node.create_subscription(Marker, NS + '/attention/bearing_marker',
                                   lambda m: received.append(m), 10)
    try:
        deadline = time.monotonic() + 15
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert pub.get_subscription_count(), 'marker node missing'

        def send(stamp, valid=True, source='ci'):
            msg = AttentionTarget()
            msg.valid = valid
            msg.source_id = source
            msg.source_generation = 1
            msg.scene_generation = 1
            msg.header.frame_id = 'base_link'
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            pub.publish(msg)

        def wait_action(action, count, timeout=3):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
                if sum(m.action == action for m in received) >= count:
                    return
            raise AssertionError('missing marker action %s count %s' % (action, count))

        send(100)
        wait_action(Marker.ADD, 1)
        assert received[-1].header.frame_id == 'base_link'
        assert abs(received[-1].points[1].x - 1.5) < 0.01
        send(101, valid=False)
        wait_action(Marker.DELETE, 1)
        send(100)
        until = time.monotonic() + 0.15
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert sum(m.action == Marker.ADD for m in received) == 1
        send(102)
        wait_action(Marker.ADD, 2)
        wait_action(Marker.DELETE, 2, timeout=3)
        print('PASS: RViz ray ADD/DELETE, replay fencing, expiration')
        return 0
    except Exception as exc:
        print('FAIL: RViz ray graph:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
