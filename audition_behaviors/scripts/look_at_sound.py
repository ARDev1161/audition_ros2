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


def admissible_observation(target, frame, max_azimuth=0.0, max_elevation=0.0):
    """Only valid bearings in our frame may advance the acquisition watermark."""
    return fresh_target(target, 0.0, 0.0, float('inf'), '', frame, max_azimuth=max_azimuth, max_elevation=max_elevation)


def newer_stamp(target, baseline):
    if target is None:
        return False
    stamp = target.header.stamp
    if stamp.sec < 0 or stamp.nanosec >= 1000000000 or (stamp.sec == 0 and stamp.nanosec == 0):
        return False
    key = (target.scene_generation, stamp.sec, stamp.nanosec)
    return baseline is None or key > baseline


def fresh_target(target, arrival, now, max_age, source, frame, sequence=1, minimum_sequence=0, max_azimuth=0.0, max_elevation=0.0):
    if sequence <= minimum_sequence or target is None or not target.valid or not math.isfinite(now) or not math.isfinite(arrival) or now < arrival or now - arrival > max_age:
        return False
    if target.header.frame_id != frame:
        return False
    if source and target.source_id != source:
        return False
    v = target.direction_unit
    if not all(math.isfinite(component) for component in (v.x, v.y, v.z)):
        return False
    norm = math.hypot(v.x, v.y, v.z)
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
        return False
    if max_azimuth > 0.0 and abs(math.atan2(v.y, v.x)) > max_azimuth:
        return False
    if max_elevation > 0.0 and abs(math.atan2(v.z, math.hypot(v.x, v.y))) > max_elevation:
        return False
    return True


def accumulate_bearing_evidence(count, previous, target, maximum_jump):
    """Count independent advancing observations from the same source and scene.

    A changed identity, generation, frame, or discontinuous bearing restarts
    confirmation. The first observation is an anchor, not a rolling reference.
    """
    if previous is None:
        return 1, target
    same = (previous.source_id == target.source_id
            and previous.source_generation == target.source_generation
            and previous.scene_generation == target.scene_generation
            and previous.header.frame_id == target.header.frame_id)
    if same and maximum_jump > 0:
        a, b = previous.direction_unit, target.direction_unit
        dot = max(-1.0, min(1.0, a.x*b.x + a.y*b.y + a.z*b.z))
        same = math.acos(dot) <= maximum_jump
    return (min(100, count + 1), previous) if same else (1, target)


class LookAtSoundServer(Node):
    def __init__(self):
        super().__init__("look_at_sound")
        self.frame = self.declare_parameter("target_frame", "base_link").value
        self.max_age = self.declare_parameter("maximum_target_age_ms", 500).value / 1000.0
        self.max_timeout = self.declare_parameter("maximum_goal_timeout_ms", 10000).value
        self.max_azimuth = self.declare_parameter("maximum_abs_azimuth_rad", 0.0).value
        self.max_elevation = self.declare_parameter("maximum_abs_elevation_rad", 0.0).value
        self.confirmations = self.declare_parameter("bearing_confirmations", 1).value
        self.confirmation_angle = self.declare_parameter("maximum_confirmation_jump_rad", 0.0).value
        self.confirmation_gap = self.declare_parameter("maximum_confirmation_gap_ms", 0).value / 1000.0
        self.confirmation_window = self.declare_parameter("maximum_confirmation_window_ms", 0).value / 1000.0
        if (not self.frame or self.frame.startswith("/") or self.max_age <= 0 or self.max_timeout < 1
                or not isinstance(self.max_azimuth, (int, float)) or not math.isfinite(self.max_azimuth)
                or not 0.0 <= self.max_azimuth <= math.pi
                or not isinstance(self.max_elevation, (int, float)) or not math.isfinite(self.max_elevation)
                or not 0.0 <= self.max_elevation <= math.pi / 2
                or type(self.confirmations) is not int or not 1 <= self.confirmations <= 100
                or not isinstance(self.confirmation_angle, (int, float))
                or not math.isfinite(self.confirmation_angle)
                or not 0.0 <= self.confirmation_angle <= math.pi
                or not math.isfinite(self.confirmation_gap)
                or not 0.0 <= self.confirmation_gap <= 10.0
                or not math.isfinite(self.confirmation_window)
                or not 0.0 <= self.confirmation_window <= 10.0):
            raise ValueError("invalid LookAtSound parameters")
        self.lock = threading.Lock()
        self.latest = None
        self.arrival = 0.0
        self.reserved = False
        self.sequence = 0
        self.goal_start_sequence = 0
        self.last_stamp = None
        self.revoked_stamp = None
        self.goal_stamp = None
        self.goal_deadline = 0.0
        self.goal_evidence = None
        self.goal_evidence_count = 0
        self.goal_evidence_sequence = 0
        self.goal_evidence_arrival = None
        self.goal_evidence_start_arrival = None
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
            # Foreign-frame DDS advisories cannot invalidate a base-frame bearing
            # or advance its revocation watermark (even with a future epoch).
            if msg.header.frame_id != self.frame:
                return
            # A valid but unusable bearing (outside the configured sector,
            # non-unit, or non-finite) is not a source-retirement signal.
            # Only an explicit valid=false advisory may revoke evidence.
            # Otherwise a high-stamped rejected bearing could permanently
            # poison the revocation watermark and block later good targets.
            if msg.valid and not admissible_observation(
                    msg, self.frame, self.max_azimuth, self.max_elevation):
                return
            if not msg.valid:
                # Ignore stale invalidations arriving after newer valid DDS data.
                if newer_stamp(msg, self.last_stamp) and newer_stamp(msg, getattr(self, 'revoked_stamp', None)):
                    self.revoked_stamp = (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)
                    self.latest = None
                    # A newer revocation breaks the consecutive evidence run.
                    # Do not let observations on either side of it combine.
                    self.goal_evidence = None
                    self.goal_evidence_count = 0
                    self.goal_evidence_sequence = self.sequence
                    self.goal_evidence_arrival = None
                    self.goal_evidence_start_arrival = None
                return
            if not newer_stamp(msg, self.last_stamp) or not newer_stamp(msg, getattr(self, 'revoked_stamp', None)):
                return
            self.last_stamp = (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)
            self.latest = msg
            self.sequence += 1
            self.arrival = time.monotonic()

    def evaluate_candidate_locked(self, now, source):
        """Atomically evaluate the live cache and update confirmation evidence.

        Caller holds self.lock. Invalidation and ingress use the same lock,
        so an old worker snapshot cannot resurrect revoked evidence.
        """
        candidate = self.latest
        sequence = self.sequence
        eligible = (now < self.goal_deadline and
                    fresh_target(candidate, self.arrival, now, self.max_age,
                                 source, self.frame, sequence, self.goal_start_sequence,
                                 self.max_azimuth, self.max_elevation) and
                    newer_stamp(candidate, self.goal_stamp))
        if eligible and sequence > self.goal_evidence_sequence:
            # Long gaps must not bridge two separate confirmation streaks.
            if ((getattr(self, "confirmation_gap", 0.0) > 0.0
                    and getattr(self, "goal_evidence_arrival", None) is not None
                    and self.arrival - self.goal_evidence_arrival > self.confirmation_gap)
                    or (getattr(self, "confirmation_window", 0.0) > 0.0
                        and getattr(self, "goal_evidence_start_arrival", None) is not None
                        and self.arrival - self.goal_evidence_start_arrival > self.confirmation_window)):
                self.goal_evidence_count = 0
                self.goal_evidence = None
                self.goal_evidence_start_arrival = None
            if self.goal_evidence_count == 0:
                self.goal_evidence_start_arrival = self.arrival
            self.goal_evidence_count, self.goal_evidence = accumulate_bearing_evidence(
                self.goal_evidence_count, self.goal_evidence, candidate, self.confirmation_angle)
            # Identity/generation/angle discontinuities also start a new window.
            if self.goal_evidence_count == 1:
                self.goal_evidence_start_arrival = self.arrival
            self.goal_evidence_sequence = sequence
            self.goal_evidence_arrival = self.arrival
        confirmed = eligible and self.goal_evidence_count >= self.confirmations
        return candidate if confirmed else None

    def on_goal(self, request):
        with self.lock:
            if self.reserved or request.timeout_ms < 1 or request.timeout_ms > self.max_timeout:
                return GoalResponse.REJECT
            self.reserved = True
            self.goal_start_sequence = self.sequence
            self.goal_stamp = self.last_stamp
            self.goal_deadline = time.monotonic() + request.timeout_ms / 1000.0
            self.goal_evidence = None
            self.goal_evidence_count = 0
            self.goal_evidence_sequence = self.sequence
            self.goal_evidence_arrival = None
            self.goal_evidence_start_arrival = None
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
                    candidate = self.evaluate_candidate_locked(now, goal.request.source_id)
                if now >= deadline:
                    result.reason = "timeout"
                    goal.abort()
                    return result
                if candidate is not None:
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
