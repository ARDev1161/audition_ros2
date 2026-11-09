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

        feedback_states = []

        def feedback_callback(msg):
            feedback_states.append(msg.feedback.state)

        def goal(timeout_ms):
            request = LookAtSound.Goal()
            request.source_id = "tracked"
            request.timeout_ms = timeout_ms
            handle = wait(node, client.send_goal_async(request, feedback_callback=feedback_callback))
            assert handle.accepted
            return handle

        def send(observation, repeats=3):
            # Keep each observation visible to the action worker's 50 ms
            # polling loop. DDS publish completion is not an acknowledgement
            # that the action worker has counted that observation.
            for _ in range(repeats):
                pub.publish(observation)
                rclpy.spin_once(node, timeout_sec=0.12)
            time.sleep(0.12)

        # Repeated DDS publications of one stamped observation count once.
        handle = goal(900)
        future = handle.get_result_async()
        send(target(100), 6)
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_ABORTED, "replay counted as confirmation"

        # Advancing stamps are necessary, but a changed source generation
        # restarts the count even when the source ID remains identical.
        handle = goal(6000)
        future = handle.get_result_async()
        # Synchronize on the server's actual evidence acknowledgement before
        # advancing the stamp. DDS delivery alone is not worker consumption.
        def await_progress(expected, since, timeout=3.0):
            # Only a feedback callback emitted after this goal/observation
            # proves that the action worker actually consumed the evidence.
            # Old feedback from previous goals is not an acknowledgement.
            deadline = time.monotonic() + timeout
            while (expected not in feedback_states[since:]
                   and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.05)
            assert expected in feedback_states[since:], (
                "no fresh confirmation progress", expected, feedback_states[since:])

        generation_start = len(feedback_states)
        send(target(200, 1))
        await_progress("confirming_bearing:1/3", generation_start)
        send(target(201, 1))
        await_progress("confirming_bearing:2/3", generation_start)
        before_generation = len(feedback_states)
        send(target(202, 2))
        # The action worker consumes DDS asynchronously. Wait for a *new*
        # acknowledgement rather than testing the most recent callback
        # immediately after publishing.
        deadline = time.monotonic() + 3.0
        while (not any(state == "confirming_bearing:1/3"
                       for state in feedback_states[before_generation:])
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not future.done(), "source generation transition reused prior evidence"
        assert any(state == "confirming_bearing:1/3"
                   for state in feedback_states[before_generation:]), (
                       "generation reset progress", feedback_states[before_generation:])
        send(target(203, 2))
        assert not future.done(), "only two observations of new generation"
        send(target(204, 2))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.acquired
        assert outcome.result.target.source_generation == 2

        # A large angular discontinuity resets confirmation evidence.
        handle = goal(6000)
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
        handle = goal(6000)
        future = handle.get_result_async()
        goal_feedback_start = len(feedback_states)
        send(target(400, 2))
        await_progress("confirming_bearing:1/3", goal_feedback_start)
        send(target(401, 2))
        await_progress("confirming_bearing:2/3", goal_feedback_start)
        # At this point the server has *acknowledged* two independent
        # observations. A stale invalidation can no longer appear to cause
        # an unrelated missed DDS/worker observation.
        stale = target(399, 2)
        stale.valid = False
        send(stale)
        send(target(402, 2))
        outcome = wait(node, future)
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, (
            "stale invalidation revoked evidence",
            outcome.status, outcome.result.reason, feedback_states[goal_feedback_start:])

        handle = goal(6000)
        future = handle.get_result_async()
        goal_feedback_start = len(feedback_states)
        send(target(500, 2))
        await_progress("confirming_bearing:1/3", goal_feedback_start)
        send(target(501, 2))
        await_progress("confirming_bearing:2/3", goal_feedback_start)
        revoked = target(502, 2)
        revoked.valid = False
        before_revocation = len(feedback_states)
        send(revoked)
        await_progress("waiting_for_fresh_bearing", before_revocation)
        # Delayed valid data older than the invalidation must not be
        # admitted or count as post-revocation evidence.
        send(target(501, 2))
        send(target(502, 2))
        assert not future.done(), "revoked observation replay was admitted"
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
