#!/usr/bin/env python3
"""Opt-in, fail-closed camera aim admission; publishes advisories, never actuator commands."""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from audition_msgs.msg import CameraAimProposal


class Admission:
    """Pure monotonic-time gate; authorization is a short-lived external heartbeat."""
    def __init__(self, frame, pan, tilt, age):
        if (not frame or frame.startswith('/') or not all(math.isfinite(x) for x in (pan, tilt, age))
                or not 0 < pan <= math.pi / 2 or not 0 < tilt <= math.pi / 2
                or not 0.05 <= age <= 5):
            raise ValueError('invalid camera admission configuration')
        self.frame, self.pan, self.tilt, self.age = frame, pan, tilt, age
        self.enabled_at = None
        self.last_key = None
        self.active = False
        self.active_at = None

    def authorize(self, enabled, now):
        self.enabled_at = now if enabled else None
        if not enabled:
            self.active = False

    def allowed(self, now):
        return self.enabled_at is not None and 0 <= now - self.enabled_at <= self.age

    def receive(self, msg, now):
        stamp = msg.header.stamp
        if (msg.header.frame_id != self.frame or stamp.sec < 0
                or not 0 <= stamp.nanosec < 1000000000
                or (stamp.sec == 0 and stamp.nanosec == 0)
                or len(msg.source_id.encode('utf-8')) > 256):
            return False
        key = (msg.scene_generation, stamp.sec, stamp.nanosec)
        if self.last_key is not None and key <= self.last_key:
            return False
        if not msg.valid:
            self.last_key = key
            self.active = False
            return False
        if (not self.allowed(now) or not msg.source_id
                or not math.isfinite(msg.pan_rad) or not math.isfinite(msg.tilt_rad)
                or abs(msg.pan_rad) > self.pan or abs(msg.tilt_rad) > self.tilt):
            self.active = False
            return False
        self.last_key = key
        self.active = True
        self.active_at = now
        return True

    def expired(self, now):
        if self.active and (not self.allowed(now) or now - self.active_at > self.age):
            self.active = False
            return True
        return False


class CameraAimAdmission(Node):
    def __init__(self):
        super().__init__('attention_camera_admission')
        frame = self.declare_parameter('target_frame', 'base_link').value
        pan = float(self.declare_parameter('maximum_pan_rad', 1.2).value)
        tilt = float(self.declare_parameter('maximum_tilt_rad', 0.7).value)
        age = float(self.declare_parameter('authorization_timeout_ms', 300).value) / 1000.0
        self.policy = Admission(frame, pan, tilt, age)
        self.publisher = self.create_publisher(CameraAimProposal, 'attention/camera_aim_admitted', 10)
        self.create_subscription(CameraAimProposal, 'attention/camera_aim_proposal', self.on_proposal, 10)
        self.create_subscription(Bool, 'attention/camera_aim_enable', self.on_enable, 10)
        self.create_timer(0.05, self.on_timer)

    def revoke(self, reason):
        msg = CameraAimProposal()
        msg.header.frame_id = self.policy.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.valid = False
        msg.reason = reason
        self.publisher.publish(msg)

    def on_enable(self, msg):
        was_active = self.policy.active
        self.policy.authorize(msg.data, time.monotonic())
        if was_active and not self.policy.active:
            self.revoke('authorization_revoked')

    def on_proposal(self, msg):
        now = time.monotonic()
        was_active = self.policy.active
        if self.policy.receive(msg, now):
            self.publisher.publish(msg)
        elif was_active and not self.policy.active:
            self.revoke('proposal_rejected_or_retired')

    def on_timer(self):
        if self.policy.expired(time.monotonic()):
            self.revoke('authorization_or_proposal_expired')


def main():
    rclpy.init()
    node = CameraAimAdmission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
