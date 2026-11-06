#!/usr/bin/env python3
"""Bearing-to-camera aim proposals. No camera, joint or robot commands."""
import math
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AttentionTarget, CameraAimProposal


def key(msg):
    return (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)


def valid_stamp(msg):
    return (msg.header.stamp.sec >= 0 and
            0 <= msg.header.stamp.nanosec < 1000000000 and
            (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0))


def angles(msg, frame, max_pan, max_tilt):
    """Return pan/tilt only for finite, normalized forward-facing bearings."""
    if not msg.valid or msg.header.frame_id != frame or not msg.source_id or not valid_stamp(msg):
        return None
    v = msg.direction_unit
    if not all(math.isfinite(x) for x in (v.x, v.y, v.z)):
        return None
    norm = v.x * v.x + v.y * v.y + v.z * v.z
    if abs(norm - 1.0) > 0.02:
        return None
    # base_link REP-103: +X forward, +Y left, +Z up.
    # A backward bearing has ambiguous pan for a limited camera mount.
    if v.x <= 0:
        return None
    pan = math.atan2(v.y, v.x)
    tilt = math.atan2(v.z, math.hypot(v.x, v.y))
    if abs(pan) > max_pan or abs(tilt) > max_tilt:
        return None
    return pan, tilt


class CameraAim(Node):
    def __init__(self):
        super().__init__('attention_camera_aim')
        self.frame = self.declare_parameter('target_frame', 'base_link').value
        self.max_pan = float(self.declare_parameter('maximum_pan_rad', 1.2).value)
        self.max_tilt = float(self.declare_parameter('maximum_tilt_rad', 0.7).value)
        self.max_age = float(self.declare_parameter('maximum_age_ms', 500).value) / 1000.0
        if (not self.frame or self.frame.startswith('/') or
                not all(math.isfinite(x) for x in (self.max_pan, self.max_tilt, self.max_age)) or
                not 0 < self.max_pan <= math.pi / 2 or
                not 0 < self.max_tilt <= math.pi / 2 or
                not 0.05 <= self.max_age <= 10):
            raise ValueError('invalid camera aim proposal configuration')
        self.watermark = None
        self.active = False
        self.arrival = 0.0
        self.publisher = self.create_publisher(CameraAimProposal, 'attention/camera_aim_proposal', 10)
        self.subscription = self.create_subscription(
            AttentionTarget, 'attention/target_base', self.on_target, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def emit(self, valid, reason, msg=None, aim=None):
        proposal = CameraAimProposal()
        proposal.header.frame_id = self.frame
        proposal.header.stamp = self.get_clock().now().to_msg()
        proposal.valid = valid
        proposal.reason = reason
        if msg is not None:
            proposal.source_id = msg.source_id
            proposal.source_generation = msg.source_generation
            proposal.scene_generation = msg.scene_generation
        if aim is not None:
            proposal.pan_rad, proposal.tilt_rad = aim
        self.publisher.publish(proposal)

    def revoke(self, reason):
        if self.active:
            self.active = False
            self.emit(False, reason)

    def on_target(self, msg):
        if msg.header.frame_id != self.frame or not valid_stamp(msg):
            return
        if self.watermark is not None:
            if key(msg) <= key(self.watermark):
                return
            if (msg.scene_generation == self.watermark.scene_generation and
                    msg.source_id == self.watermark.source_id and
                    msg.source_generation < self.watermark.source_generation):
                return
        if not msg.valid:
            if (self.watermark is not None and
                    msg.scene_generation == self.watermark.scene_generation and
                    msg.source_id and msg.source_id != self.watermark.source_id):
                return
            self.watermark = msg
            self.revoke('source_retired')
            return
        aim = angles(msg, self.frame, self.max_pan, self.max_tilt)
        if aim is None:
            # Invalid geometry cannot poison the replay watermark; if the
            # previously active source is no longer admissible, revoke it.
            self.revoke('bearing_outside_camera_limits')
            return
        self.watermark = msg
        self.arrival = time.monotonic()
        self.active = True
        self.emit(True, 'bearing_proposal', msg, aim)

    def on_timer(self):
        if self.active and time.monotonic() - self.arrival > self.max_age:
            self.revoke('bearing_expired')


def main():
    rclpy.init()
    node = CameraAim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
