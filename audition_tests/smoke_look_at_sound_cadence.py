#!/usr/bin/env python3
"""Exercise launch-wired LookAtSound cadence gates through ROS 2 DDS/action."""
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from audition_msgs.action import LookAtSound
from audition_msgs.msg import AttentionTarget
from rclpy.action import ActionClient
from rclpy.node import Node

NS = "/robot_ci/cadence"


def wait(node, future, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
    assert future.done(), "action future timed out"
    return future.result()


def run():
    rclpy.init()
    node = Node("cadence_graph_probe")
    pub = node.create_publisher(AttentionTarget, NS + "/attention/target_base", 10)
    client = ActionClient(node, LookAtSound, NS + "/look_at_sound")
    stamp = 100
    try:
        assert client.wait_for_server(timeout_sec=25), "LookAtSound action unavailable"
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
            return handle.get_result_async()

        def send(delay=0.0, repeats=2):
            nonlocal stamp
            if delay:
                time.sleep(delay)
            stamp += 1
            msg = AttentionTarget()
            msg.valid = True
            msg.source_id = "tracked"
            msg.source_generation = 1
            msg.scene_generation = 7
            msg.header.frame_id = "base_link"
            msg.header.stamp.sec = stamp
            msg.direction_unit.x = 1.0
            # Repeated publication of one stamp helps the worker consume the
            # observation without adding evidence. Keep the send cadence
            # well below the 180ms minimum spacing in the burst case.
            for _ in range(repeats):
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.03)
            time.sleep(0.04)

        # Three updates in a burst: at most two should meet 180ms spacing.
        result = goal(900)
        send()
        send()
        send()
        assert wait(node, result).status == GoalStatus.STATUS_ABORTED, "burst counted"

        # Gap > 750 ms breaks the run, even though each sample is fresh.
        result = goal(1900)
        send()
        send(delay=0.85)
        send()
        assert wait(node, result).status == GoalStatus.STATUS_ABORTED, "gap bridged"

        # Three samples spaced >180 ms but spanning >900 ms restart the run.
        result = goal(2500)
        send()
        send(delay=0.55)
        send(delay=0.55)
        assert wait(node, result).status == GoalStatus.STATUS_ABORTED, "window bridged"

        # A fresh bounded sequence satisfies all three constraints.
        result = goal(2500)
        # Republish each identical stamped observation across several
        # executor spins: DDS scheduling may miss a single short burst under
        # CI load, but duplicates must NOT count as independent evidence.
        # Avoid spending most of the 900ms window repeating the same
        # observation. Under CI load four copies per sample could push the
        # third arrival beyond the window even without any policy defect.
        # Two copies per stamp preserve duplicate-rejection coverage, and
        # 250ms inter-sample delay clears the 180ms minimum spacing while
        # leaving substantially more room for executor scheduling jitter.
        send(repeats=2)
        send(delay=0.25, repeats=2)
        send(delay=0.25, repeats=2)
        outcome = wait(node, result)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, ("bounded recovery", outcome.status, outcome.result.reason)
        assert outcome.result.acquired
        print("PASS: launch-wired spacing, gap, window and bounded recovery")
        return 0
    except Exception as exc:
        print("FAIL: LookAtSound cadence graph:", repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
