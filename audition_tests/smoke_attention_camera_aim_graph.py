#!/usr/bin/env python3
"""Live DDS camera proposal lifecycle, independent of any actuator."""
import math
import sys
import time
import rclpy
from rclpy.node import Node
from audition_msgs.msg import AttentionTarget, CameraAimProposal


def main():
    rclpy.init()
    node = Node('camera_aim_graph_probe')
    seen = []
    node.create_subscription(CameraAimProposal, '/robot_ci/camera/attention/camera_aim_proposal',
                             lambda m: seen.append(m), 10)
    pub = node.create_publisher(AttentionTarget, '/robot_ci/camera/attention/target_base', 10)
    try:
        end = time.monotonic() + 20
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert pub.get_subscription_count(), 'camera aim subscriber not discovered'
        def send(stamp, valid=True, x=1.0, y=0.0, source='a'):
            msg = AttentionTarget()
            msg.valid = valid
            msg.source_id = source
            msg.source_generation = 1
            msg.scene_generation = 1
            msg.header.frame_id = 'base_link'
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = x
            msg.direction_unit.y = y
            pub.publish(msg)
        def wait_for(count):
            end = time.monotonic() + 4
            while len(seen) < count and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert len(seen) >= count, ('missing proposal', count, len(seen))
        # Oversized identities must not consume the watermark.
        send(999, source='oversized' * 40)
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not seen, 'oversized source identity accepted'
        send(10, x=math.sqrt(0.5), y=math.sqrt(0.5))
        wait_for(1)
        assert seen[-1].valid and abs(seen[-1].pan_rad - math.pi / 4) < 0.01
        send(9)
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(seen) == 1, 'replay accepted'
        send(11, valid=False)
        wait_for(2)
        assert not seen[-1].valid and seen[-1].reason == 'source_retired'
        send(12)
        wait_for(3)
        assert seen[-1].valid
        wait_for(4)
        assert not seen[-1].valid and seen[-1].reason == 'bearing_expired'
        print('PASS: camera aim DDS, geometry, replay, retirement and expiry')
        return 0
    except Exception as exc:
        print('FAIL: camera aim graph:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
