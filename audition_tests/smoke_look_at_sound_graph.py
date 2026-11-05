#!/usr/bin/env python3
"""End-to-end LookAtSound action tests against a separate ROS process."""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from audition_msgs.action import LookAtSound
from audition_msgs.msg import AttentionTarget

NS = "/robot_ci/look"


class Probe(Node):
    def __init__(self):
        super().__init__("look_at_sound_graph_probe")
        self.client = ActionClient(self, LookAtSound, NS + "/look_at_sound")
        self.publisher = self.create_publisher(AttentionTarget, NS + "/attention/target_base", 10)

    def await_future(self, future, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        assert future.done(), "action future timed out"
        return future.result()

    def target(self, source="ci-source", valid=True):
        msg = AttentionTarget()
        msg.valid = valid
        msg.source_id = source
        msg.source_generation = 2
        msg.scene_generation = 3
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.direction_unit.x = 1.0
        msg.reason = "tf_transformed"
        return msg

    def start(self, source, timeout_ms):
        goal = LookAtSound.Goal()
        goal.source_id = source
        goal.timeout_ms = timeout_ms
        return self.await_future(self.client.send_goal_async(goal))

    def publish_until(self, handle, expected_status, timeout=5, source="ci-source"):
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not result_future.done():
            self.publisher.publish(self.target(source))
            rclpy.spin_once(self, timeout_sec=0.05)
        assert result_future.done(), "result never arrived"
        outcome = result_future.result()
        assert outcome.status == expected_status, (outcome.status, expected_status)
        return outcome.result


def main():
    rclpy.init()
    node = Probe()
    try:
        assert node.client.wait_for_server(timeout_sec=25), "action server not discovered"
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and node.publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.publisher.get_subscription_count() > 0, "target subscriber missing"

        # Start with no valid cached target; ensure source filtering.
        first = node.start("ci-source", 4000)
        assert first.accepted, "first goal rejected"
        second = node.start("", 1000)
        assert not second.accepted, "concurrent goal accepted"
        wrong = node.target(source="other-source")
        for _ in range(3):
            node.publisher.publish(wrong)
            rclpy.spin_once(node, timeout_sec=0.06)
        result = node.publish_until(first, GoalStatus.STATUS_SUCCEEDED)
        assert result.acquired and result.reason == "bearing_acquired"
        assert result.target.source_id == "ci-source"
        assert result.target.scene_generation == 3 and result.target.header.frame_id == "base_link"

        # A second goal cannot reuse the valid target cached before acceptance.
        cached_goal = node.start("ci-source", 450)
        assert cached_goal.accepted
        cached_outcome = node.await_future(cached_goal.get_result_async(), 5)
        assert cached_outcome.status == GoalStatus.STATUS_ABORTED, "cached pre-goal target incorrectly accepted"
        assert cached_outcome.result.reason == "timeout"
        # Replaying a previously seen observation with a new DDS arrival
        # must not satisfy a new goal.
        replay = node.target()
        for _ in range(3):
            node.publisher.publish(replay)
            rclpy.spin_once(node, timeout_sec=0.05)
        replay_goal = node.start("ci-source", 550)
        assert replay_goal.accepted
        for _ in range(6):
            node.publisher.publish(replay)
            rclpy.spin_once(node, timeout_sec=0.06)
        replay_result = node.await_future(replay_goal.get_result_async(), 5)
        assert replay_result.status == GoalStatus.STATUS_ABORTED, "replayed stamp accepted"
        assert replay_result.result.reason == "timeout"

        # A genuinely advancing observation must still acquire.
        advancing_goal = node.start("ci-source", 3000)
        assert advancing_goal.accepted
        advanced = node.publish_until(advancing_goal, GoalStatus.STATUS_SUCCEEDED)
        assert advanced.acquired and advanced.reason == "bearing_acquired"

        timeout_goal = node.start("missing-source", 300)
        assert timeout_goal.accepted
        timeout_result = node.await_future(timeout_goal.get_result_async(), 5)
        assert timeout_result.status == GoalStatus.STATUS_ABORTED
        assert not timeout_result.result.acquired and timeout_result.result.reason == "timeout"

        cancel_goal = node.start("missing-source", 3000)
        assert cancel_goal.accepted
        cancel_response = node.await_future(cancel_goal.cancel_goal_async(), 5)
        assert cancel_response.goals_canceling, "cancellation not accepted"
        canceled = node.await_future(cancel_goal.get_result_async(), 5)
        assert canceled.status == GoalStatus.STATUS_CANCELED
        assert canceled.result.reason == "canceled"

        invalid = node.start("", 0)
        assert not invalid.accepted, "zero timeout accepted"
        print("PASS: acquisition, source filtering, concurrency, timeout, cancellation, invalid goal")
        return 0
    except (AssertionError, Exception) as exc:
        print("FAIL: LookAtSound graph:", repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
