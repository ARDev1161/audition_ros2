#!/usr/bin/env python3
"""LookAtSound: cancellable acoustic target acquisition, NEVER motion control."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from audition_msgs.action import LookAtSound
from audition_msgs.msg import AttentionTarget


def admissible_observation(target, frame):
    """Only valid bearings in our frame may advance the acquisition watermark."""
    return fresh_target(target, 0.0, 0.0, float('inf'), '', frame)


def newer_stamp(target, baseline):
    if target is None:
        return False
    stamp = target.header.stamp
    if stamp.sec < 0 or stamp.nanosec >= 1000000000 or (stamp.sec == 0 and stamp.nanosec == 0):
        return False
    key = (target.scene_generation, stamp.sec, stamp.nanosec)
    return baseline is None or key > baseline


def fresh_target(target, arrival, now, max_age, source, frame, sequence=1, minimum_sequence=0):
    if sequence <= minimum_sequence or target is None or not target.valid or now - arrival > max_age:
        return False
    if target.header.frame_id != frame:
        return False
    if source and target.source_id != source:
        return False
    v = target.direction_unit
    norm = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    return math.isfinite(norm) and abs(norm - 1.0) <= 0.01


class LookAtSoundServer(Node):
    def __init__(self):
        super().__init__("look_at_sound")
        self.frame = self.declare_parameter("target_frame", "base_link").value
        self.max_age = self.declare_parameter("maximum_target_age_ms", 500).value / 1000.0
        self.max_timeout = self.declare_parameter("maximum_goal_timeout_ms", 10000).value
        if not self.frame or self.frame.startswith("/") or self.max_age <= 0 or self.max_timeout < 1:
            raise ValueError("invalid LookAtSound parameters")
        self.lock = threading.Lock()
        self.latest = None
        self.arrival = 0.0
        self.reserved = False
        self.sequence = 0
        self.goal_start_sequence = 0
        self.last_stamp = None
        self.goal_stamp = None
        self.goal_deadline = 0.0
        group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            AttentionTarget, "attention/target_base", self.on_target, 10,
            callback_group=group)
        self.server = ActionServer(
            self, LookAtSound, "look_at_sound", self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=group)

    def on_target(self, msg):
        with self.lock:
            # A delayed DDS replay must not replace a newer accepted bearing.
            # Invalid advisories revoke the cached bearing without consuming
            # the watermark, allowing subsequent valid observations to recover.
            if not admissible_observation(msg, self.frame):
                # Ignore stale invalidations arriving after newer valid DDS data.
                if newer_stamp(msg, self.last_stamp):
                    self.latest = None
                return
            if not newer_stamp(msg, self.last_stamp):
                return
            self.last_stamp = (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.latest = msg
            self.sequence += 1
            self.arrival = time.monotonic()

    def on_goal(self, request):
        with self.lock:
            if self.reserved or request.timeout_ms < 1 or request.timeout_ms > self.max_timeout:
                return GoalResponse.REJECT
            self.reserved = True
            self.goal_start_sequence = self.sequence
            self.goal_stamp = self.last_stamp
            self.goal_deadline = time.monotonic() + request.timeout_ms / 1000.0
            return GoalResponse.ACCEPT

    def on_cancel(self, _goal):
        return CancelResponse.ACCEPT

    def execute(self, goal):
        result = LookAtSound.Result()
        with self.lock:
            deadline = self.goal_deadline
        try:
            while rclpy.ok():
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.reason = "canceled"
                    return result
                now = time.monotonic()
                with self.lock:
                    candidate = self.latest
                    arrival = self.arrival
                    sequence = self.sequence
                    start_sequence = self.goal_start_sequence
                    start_stamp = self.goal_stamp
                if now >= deadline:
                    result.reason = "timeout"
                    goal.abort()
                    return result
                if fresh_target(candidate, arrival, now, self.max_age,
                                goal.request.source_id, self.frame, sequence, start_sequence) and newer_stamp(candidate, start_stamp):
                    result.acquired = True
                    result.target = candidate
                    result.reason = "bearing_acquired"
                    goal.succeed()
                    return result
                feedback = LookAtSound.Feedback()
                feedback.state = "waiting_for_fresh_bearing"
                goal.publish_feedback(feedback)
                time.sleep(min(0.05, max(0.0, deadline - now)))
            result.reason = "shutdown"
            goal.abort()
            return result
        finally:
            with self.lock:
                self.reserved = False


def main():
    rclpy.init()
    node = LookAtSoundServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
