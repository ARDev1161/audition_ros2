#!/usr/bin/env python3
"""Live ROS graph acceptance for opt-in camera admission and revocation."""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from audition_msgs.msg import CameraAimProposal

NS = '/robot_ci/admission/attention/'


def proposal(stamp):
    msg = CameraAimProposal()
    msg.valid = True
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.scene_generation = 1
    msg.source_generation = 1
    msg.source_id = 'source-a'
    msg.pan_rad = 0.2
    msg.tilt_rad = 0.1
    return msg


def main():
    rclpy.init()
    node = Node('camera_admission_graph_probe')
    seen = []
    node.create_subscription(CameraAimProposal, NS + 'camera_aim_admitted',
                             lambda msg: seen.append(msg), 10)
    proposals = node.create_publisher(CameraAimProposal, NS + 'camera_aim_proposal', 10)
    enable = node.create_publisher(Bool, NS + 'camera_aim_enable', 10)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)

    def await_count(n):
        deadline = time.monotonic() + 3
        while len(seen) < n and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
        assert len(seen) >= n, ('missing admission transition', n, len(seen))

    try:
        deadline = time.monotonic() + 15
        while (proposals.get_subscription_count() < 1 or enable.get_subscription_count() < 1) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert proposals.get_subscription_count() and enable.get_subscription_count(), 'node not discovered'

        # Never emit a valid advisory before authorization.
        proposals.publish(proposal(1))
        spin_for(0.15)
        assert not any(x.valid for x in seen), 'default-deny violated'

        enable.publish(Bool(data=True))
        spin_for(0.08)
        proposals.publish(proposal(2))
        await_count(1)
        assert seen[-1].valid and seen[-1].source_id == 'source-a'

        # Explicit false immediately withdraws active authorization.
        enable.publish(Bool(data=False))
        await_count(2)
        assert not seen[-1].valid and seen[-1].reason == 'authorization_revoked'

        # Stale proposal cannot pass even after authorization returns.
        enable.publish(Bool(data=True))
        spin_for(0.08)
        proposals.publish(proposal(2))
        spin_for(0.12)
        assert len(seen) == 2, 'replay admitted'
        proposals.publish(proposal(3))
        await_count(3)
        assert seen[-1].valid

        # No heartbeat => fail closed, with a single withdrawal.
        await_count(4)
        assert not seen[-1].valid and seen[-1].reason == 'authorization_or_proposal_expired'
        spin_for(0.2)
        assert len(seen) == 4, 'repeated withdrawal'
        print('PASS: default deny, authorized proposal, explicit revoke, replay, heartbeat expiry')
        return 0
    except Exception as exc:
        print('FAIL: camera admission DDS:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
