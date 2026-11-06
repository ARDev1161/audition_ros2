#!/usr/bin/env python3
"""Live DDS/action acceptance for the opt-in TrackSound advisory server."""
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from audition_msgs.action import TrackSound
from audition_msgs.msg import AttentionTarget
from rclpy.action import ActionClient
from rclpy.node import Node

NS = "/robot_ci/track"


def await_future(node, future, timeout=10.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.03)
    assert future.done(), "ROS action future timed out"
    return future.result()


def main():
    rclpy.init()
    node = Node("track_sound_graph_probe")
    pub = node.create_publisher(AttentionTarget, NS + "/attention/target_base", 10)
    client = ActionClient(node, TrackSound, NS + "/track_sound")
    feedback = []
    stamp = 100
    try:
        assert client.wait_for_server(timeout_sec=30), "TrackSound action not discovered"
        until = time.monotonic() + 10
        while pub.get_subscription_count() == 0 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert pub.get_subscription_count(), "TrackSound subscription not discovered"

        def target(source="tracked", generation=1, valid=True, frame="base_link", stamp_override=None):
            nonlocal stamp
            stamp += 1
            msg = AttentionTarget()
            msg.valid = valid
            msg.source_id = source
            msg.source_generation = generation
            msg.scene_generation = 7
            msg.header.frame_id = frame
            msg.header.stamp.sec = stamp if stamp_override is None else stamp_override
            msg.direction_unit.x = 1.0
            return msg

        def publish(msg):
            for _ in range(3):
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
            # Give the separate action server executor time to consume DDS.
            time.sleep(0.12)

        def start(source, timeout_ms):
            request = TrackSound.Goal()
            request.source_id = source
            request.timeout_ms = timeout_ms
            def on_feedback(event):
                feedback.append(event.feedback)
            handle = await_future(node, client.send_goal_async(request, feedback_callback=on_feedback))
            return handle

        # No pre-goal cache replay; source filtering and single-goal admission.
        cached = target()
        publish(cached)
        first = start("tracked", 1600)
        assert first.accepted
        assert not start("", 1000).accepted, "overlapping goal accepted"
        publish(target(source="unrelated"))
        publish(cached)
        assert not feedback, "wrong-source or cached target produced feedback"
        fresh = target()
        publish(fresh)
        publish(fresh)
        publish(target(generation=0))
        publish(target(frame="map"))
        publish(target(valid=False))
        publish(target(stamp_override=101))
        publish(target(generation=2))
        outcome = await_future(node, first.get_result_async(), 6)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, ("tracking outcome", outcome.status, outcome.result.reason)
        assert outcome.result.tracked and outcome.result.updates == 2, ("unexpected evidence", outcome.result.updates)
        assert len(feedback) == 2, ("unexpected feedback", len(feedback))
        assert [f.updates for f in feedback] == [1, 2]
        assert all(f.state == "tracking" and f.target.source_id == "tracked" for f in feedback)
        assert feedback[1].target.source_generation == 2

        # A new goal cannot replay the last observation, even if republished.
        feedback.clear()
        second = start("tracked", 450)
        assert second.accepted
        publish(fresh)
        publish(target(stamp_override=101))
        empty = await_future(node, second.get_result_async(), 5)
        assert empty.status == GoalStatus.STATUS_ABORTED, ("stale replay accepted", empty.status)
        assert not empty.result.tracked and empty.result.updates == 0
        assert empty.result.reason == "no_fresh_bearing"
        assert not feedback

        # Explicit cancellation ends the action without inventing success.
        cancel = start("tracked", 3000)
        assert cancel.accepted
        response = await_future(node, cancel.cancel_goal_async(), 5)
        assert response.goals_canceling, "cancel rejected"
        canceled = await_future(node, cancel.get_result_async(), 5)
        assert canceled.status == GoalStatus.STATUS_CANCELED
        assert canceled.result.reason == "canceled"
        assert not start("", 0).accepted, "zero timeout accepted"
        print("PASS: TrackSound DDS source filter, replay, revocation, feedback, timeout and cancellation")
        return 0
    except Exception as exc:
        print("FAIL: TrackSound ROS graph:", repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
