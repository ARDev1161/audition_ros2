#!/usr/bin/env python3
"""Live ROS graph regression for LookAtSound angular sector and recovery."""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from audition_msgs.action import LookAtSound
from audition_msgs.msg import AttentionTarget

NS = "/robot_ci/sector"


def wait(node, future, seconds=8):
    until = time.monotonic() + seconds
    while time.monotonic() < until and not future.done():
        rclpy.spin_once(node, timeout_sec=0.04)
    assert future.done(), "action future timed out"
    return future.result()


def observation(node, stamp, x, y=0.0, z=0.0):
    msg = AttentionTarget()
    msg.valid = True
    msg.source_id = "sector-source"
    msg.source_generation = 1
    msg.scene_generation = 1
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = stamp
    msg.direction_unit.x = x
    msg.direction_unit.y = y
    msg.direction_unit.z = z
    return msg


def run():
    rclpy.init()
    node = Node("sector_graph_probe")
    publisher = node.create_publisher(AttentionTarget, NS + "/attention/target_base", 10)
    client = ActionClient(node, LookAtSound, NS + "/look_at_sound")
    try:
        assert client.wait_for_server(timeout_sec=25)
        until = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() > 0

        # 0.5 rad azimuth/elevation limits: valid unit vector outside
        # either limit must not satisfy a goal, even when stamped later.
        goal = LookAtSound.Goal()
        goal.source_id = "sector-source"
        goal.timeout_ms = 1200
        handle = wait(node, client.send_goal_async(goal))
        assert handle.accepted
        future = handle.get_result_async()
        for stamp in range(100, 106):
            publisher.publish(observation(node, stamp, math.cos(0.8), math.sin(0.8)))
            rclpy.spin_once(node, timeout_sec=0.09)
        for stamp in range(106, 112):
            publisher.publish(observation(node, stamp, math.cos(0.8), 0.0, math.sin(0.8)))
            rclpy.spin_once(node, timeout_sec=0.09)
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_ABORTED, "out-of-sector bearing acquired"
        assert outcome.result.reason == "timeout"

        # A later in-sector direction must acquire, without a stale
        # outside-sector observation consuming the valid watermark.
        handle = wait(node, client.send_goal_async(goal))
        assert handle.accepted
        future = handle.get_result_async()
        valid = observation(node, 200, 1.0)
        until = time.monotonic() + 4
        while time.monotonic() < until and not future.done():
            publisher.publish(valid)
            rclpy.spin_once(node, timeout_sec=0.06)
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.acquired and outcome.result.target.source_id == "sector-source"
        print("PASS: azimuth/elevation exclusion and in-sector acquisition")
        return 0
    except Exception as exc:
        print("FAIL: sector graph:", repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
