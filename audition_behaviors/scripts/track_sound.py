#!/usr/bin/env python3
"""TrackSound: cancellable, bounded advisory tracking; NO actuator commands."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from audition_msgs.action import TrackSound
from audition_msgs.msg import AttentionTarget


MAX_SOURCE_ID_BYTES = 256


def valid_identity(source):
    return bool(source) and len(source.encode('utf-8')) <= MAX_SOURCE_ID_BYTES


def valid_stamp(msg):
    return (msg.header.stamp.sec >= 0
            and 0 <= msg.header.stamp.nanosec < 1000000000
            and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0))


def valid_bearing(msg, frame):
    if not msg.valid or msg.header.frame_id != frame or not valid_identity(msg.source_id):
        return False
    if not valid_stamp(msg):
        return False
    v = msg.direction_unit
    return (all(math.isfinite(x) for x in (v.x, v.y, v.z))
            and abs(math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) - 1.0) <= 0.01)


def stamp_key(msg):
    return (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)


def admit(previous, msg, frame, source):
    """One monotonically advancing source incarnation; explicit retirement fences replay."""
    if msg.header.frame_id != frame or (source and msg.source_id != source):
        return False
    if previous is not None:
        if stamp_key(msg) <= stamp_key(previous):
            return False
        if (msg.scene_generation == previous.scene_generation
                and msg.source_id == previous.source_id
                and msg.source_generation < previous.source_generation):
            return False
    return valid_bearing(msg, frame)


class TrackSoundServer(Node):
    def __init__(self):
        super().__init__('track_sound')
        self.frame = self.declare_parameter('target_frame', 'base_link').value
        self.max_age = self.declare_parameter('maximum_target_age_ms', 500).value / 1000.0
        self.max_timeout = self.declare_parameter('maximum_goal_timeout_ms', 30000).value
        if (not self.frame or self.frame.startswith('/') or not 0 < self.max_age <= 10
                or not 1 <= self.max_timeout <= 300000):
            raise ValueError('invalid TrackSound configuration')
        self.lock = threading.Lock()
        self.latest = None
        self.arrival = 0.0
        self.sequence = 0
        self.reserved = False
        self.watermark = None
        self.retired = None
        self.retirement_sequence = 0
        group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            AttentionTarget, 'attention/target_base', self.on_target, 10, callback_group=group)
        self.server = ActionServer(self, TrackSound, 'track_sound', self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=group)

    def on_target(self, msg):
        with self.lock:
            if msg.header.frame_id != self.frame or not valid_stamp(msg):
                return
            if msg.source_id and not valid_identity(msg.source_id):
                return
            if self.watermark is not None and stamp_key(msg) <= stamp_key(self.watermark):
                return
            if self.watermark is not None and msg.scene_generation == self.watermark.scene_generation:
                if (msg.source_id == self.watermark.source_id
                        and msg.source_generation < self.watermark.source_generation):
                    return
            if not msg.valid:
                if (self.watermark is not None and msg.scene_generation == self.watermark.scene_generation
                        and msg.source_id and msg.source_id != self.watermark.source_id):
                    return
                self.watermark = msg
                self.latest = None
                self.retired = msg
                self.retirement_sequence += 1
                return
            if not valid_bearing(msg, self.frame):
                return
            self.watermark = msg
            self.latest = msg
            self.arrival = time.monotonic()
            self.sequence += 1

    def on_goal(self, request):
        with self.lock:
            if (self.reserved or not 1 <= request.timeout_ms <= self.max_timeout
                    or (request.source_id and not valid_identity(request.source_id))):
                return GoalResponse.REJECT
            self.reserved = True
            self.start_sequence = self.sequence
            self.start_time = time.monotonic()
            self.start_retirement_sequence = self.retirement_sequence
            return GoalResponse.ACCEPT

    def on_cancel(self, _goal):
        return CancelResponse.ACCEPT

    def execute(self, goal):
        result = TrackSound.Result()
        with self.lock:
            start = self.start_time
            last_sequence = self.start_sequence
            retirement_sequence = self.start_retirement_sequence
            active_identity = None
        deadline = start + goal.request.timeout_ms / 1000.0
        try:
            while rclpy.ok():
                if goal.is_cancel_requested:
                    result.reason = 'canceled'
                    goal.canceled()
                    return result
                now = time.monotonic()
                if now >= deadline:
                    result.tracked = result.updates > 0
                    result.reason = 'tracking_window_complete' if result.tracked else 'no_fresh_bearing'
                    goal.succeed() if result.tracked else goal.abort()
                    return result
                with self.lock:
                    target = self.latest
                    seq = self.sequence
                    arrival = self.arrival
                    retired = self.retired
                    retirement_changed = self.retirement_sequence != retirement_sequence
                # A source retirement is a hard advisory boundary, not a
                # successful tracking window. Never keep reporting a previously
                # tracked source after an explicit invalidation.
                if retirement_changed and retired is not None:
                    retired_identity = (retired.scene_generation, retired.source_id,
                                        retired.source_generation)
                    if (active_identity is not None
                            and retired_identity[0] >= active_identity[0]
                            and (not retired_identity[1] or
                                 retired_identity[1] == active_identity[1])):
                        result.reason = 'source_retired'
                        goal.abort()
                        return result
                    retirement_sequence = self.retirement_sequence
                # A silent source expiration must not be reported as ongoing
                # tracking. End the action when the last admitted bearing has
                # exceeded the configured freshness window.
                if (active_identity is not None and
                        (target is None or now < arrival or now - arrival > self.max_age)):
                    result.reason = 'source_expired'
                    goal.abort()
                    return result
                if (target is not None and seq > last_sequence
                        and now >= arrival and now - arrival <= self.max_age
                        and (not goal.request.source_id or target.source_id == goal.request.source_id)):
                    identity = (target.scene_generation, target.source_id,
                                target.source_generation)
                    if active_identity is not None and identity != active_identity:
                        result.reason = 'source_changed'
                        goal.abort()
                        return result
                    active_identity = identity
                    last_sequence = seq
                    result.updates += 1
                    feedback = TrackSound.Feedback()
                    feedback.target = target
                    feedback.updates = result.updates
                    feedback.state = 'tracking'
                    goal.publish_feedback(feedback)
                time.sleep(min(0.05, max(0.0, deadline - now)))
            result.reason = 'shutdown'
            goal.abort()
            return result
        finally:
            with self.lock:
                self.reserved = False


def main():
    rclpy.init()
    node = TrackSoundServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
