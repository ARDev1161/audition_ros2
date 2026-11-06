#!/usr/bin/env python3
"""ROS graph acceptance of multi-observation LookAtSound confirmations."""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from audition_msgs.action import LookAtSound
from audition_msgs.msg import AttentionTarget
from rclpy.action import ActionClient
from rclpy.node import Node

NS = "/robot_ci/confirm"


def wait(node, future, seconds=7):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.04)
    assert future.done(), "action future timed out"
    return future.result()


def target(stamp, source_generation=1, angle=0.0):
    msg = AttentionTarget()
    msg.valid = True
    msg.source_id = "tracked"
    msg.source_generation = source_generation
    msg.scene_generation = 7
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = stamp
    msg.direction_unit.x = math.cos(angle)
    msg.direction_unit.y = math.sin(angle)
    return msg


def run():
    rclpy.init()
    node = Node("confirmation_graph_probe")
    pub = node.create_publisher(AttentionTarget, NS + "/attention/target_base", 10)
    client = ActionClient(node, LookAtSound, NS + "/look_at_sound")
    try:
        assert client.wait_for_server(timeout_sec=25), "action server unavailable"
        until = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert pub.get_subscription_count() > 0

        def goal(timeout_ms):
            request = LookAtSound.Goal()
            request.source_id = "tracked"
            request.timeout_ms = timeout_ms
            handle = wait(node, client.send_goal_async(request))
            assert handle.accepted
            return handle

        def send(observation, repeats=3):
            for _ in range(repeats):
                pub.publish(observation)
                rclpy.spin_once(node, timeout_sec=0.09)

        # Repeated DDS publications of one stamped observation count once.
        handle = goal(900)
        future = handle.get_result_async()
        send(target(100), 6)
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_ABORTED, "replay counted as confirmation"

        # Advancing stamps are necessary, but a changed source generation
        # restarts the count even when the source ID remains identical.
        handle = goal(3500)
        future = handle.get_result_async()
        send(target(200, 1))
        send(target(201, 1))
        send(target(202, 2))
        assert not future.done(), "source generation transition reused prior evidence"
        send(target(203, 2))
        assert not future.done(), "only two observations of new generation"
        send(target(204, 2))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.acquired
        assert outcome.result.target.source_generation == 2

        # A large angular discontinuity resets confirmation evidence.
        handle = goal(3500)
        future = handle.get_result_async()
        send(target(300, 2, 0.0))
        send(target(301, 2, 0.6))
        send(target(302, 2, 0.61))
        assert not future.done(), "jump did not reset evidence"
        send(target(303, 2, 0.62))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.acquired
        # An invalidation newer than the current bearing must revoke a
        # partially collected streak. A stale invalidation must not do so.
        handle = goal(3500)
        future = handle.get_result_async()
        send(target(400, 2))
        send(target(401, 2))
        stale = target(399, 2)
        stale.valid = False
        send(stale)
        send(target(402, 2))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, "stale invalidation revoked evidence"

        handle = goal(3500)
        future = handle.get_result_async()
        send(target(500, 2))
        send(target(501, 2))
        revoked = target(502, 2)
        revoked.valid = False
        send(revoked)
        send(target(503, 2))
        assert not future.done(), "revoked evidence survived invalidation"
        send(target(504, 2))
        assert not future.done(), "two post-revocation observations are insufficient"
        send(target(505, 2))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.acquired
        assert outcome.result.target.header.stamp.sec == 505
        print("PASS: replay rejection, generation reset, angular continuity, revocation fencing")
        return 0
    except Exception as exc:
        print("FAIL: LookAtSound confirmation graph:", repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
