#!/usr/bin/env python3
"""Optional RViz bearing ray: visual-only, no inferred range or actuation."""
import math
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AttentionTarget
from visualization_msgs.msg import Marker


def stamp_key(msg):
    return (msg.scene_generation, msg.header.stamp.sec, msg.header.stamp.nanosec)


def valid_target(msg, frame):
    v = msg.direction_unit
    norm = v.x * v.x + v.y * v.y + v.z * v.z
    return (msg.valid and msg.header.frame_id == frame and bool(msg.source_id)
            and 0 <= msg.header.stamp.sec and 0 <= msg.header.stamp.nanosec < 1000000000
            and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0)
            and all(math.isfinite(x) for x in (v.x, v.y, v.z))
            and abs(norm - 1.0) <= 0.02)


class BearingRay(Node):
    def __init__(self):
        super().__init__('attention_bearing_ray')
        self.frame = self.declare_parameter('target_frame', 'base_link').value
        self.length = float(self.declare_parameter('ray_length_m', 1.5).value)
        self.max_age = int(self.declare_parameter('maximum_age_ms', 500).value) / 1000.0
        if (not self.frame or self.frame.startswith('/') or not math.isfinite(self.length)
                or not 0.05 <= self.length <= 20.0 or not 0.05 <= self.max_age <= 10.0):
            raise ValueError('invalid RViz bearing configuration')
        self.watermark = None
        self.active = False
        self.last_arrival = 0.0
        self.publisher = self.create_publisher(Marker, 'attention/bearing_marker', 10)
        self.subscription = self.create_subscription(
            AttentionTarget, 'attention/target_base', self.on_target, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def marker(self, action, msg=None):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'audition_bearing'
        marker.id = 0
        marker.action = action
        if action == Marker.ADD:
            marker.type = Marker.ARROW
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.scale.y = 0.05
            marker.scale.z = 0.07
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 1.0
            marker.color.a = 0.9
            from geometry_msgs.msg import Point
            start = Point()
            end = Point()
            end.x = msg.direction_unit.x * self.length
            end.y = msg.direction_unit.y * self.length
            end.z = msg.direction_unit.z * self.length
            marker.points = [start, end]
        return marker

    def clear(self):
        if self.active:
            self.active = False
            self.publisher.publish(self.marker(Marker.DELETE))

    def on_target(self, msg):
        if msg.header.frame_id != self.frame:
            return
        if msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 1000000000:
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        if self.watermark is not None:
            if stamp_key(msg) <= stamp_key(self.watermark):
                return
            if (msg.scene_generation == self.watermark.scene_generation
                    and msg.source_id == self.watermark.source_id
                    and msg.source_generation < self.watermark.source_generation):
                return
        if not msg.valid:
            if (self.watermark is not None
                    and msg.scene_generation == self.watermark.scene_generation
                    and msg.source_id and msg.source_id != self.watermark.source_id):
                return
            self.watermark = msg
            self.clear()
            return
        if not valid_target(msg, self.frame):
            return
        self.watermark = msg
        self.last_arrival = time.monotonic()
        self.active = True
        self.publisher.publish(self.marker(Marker.ADD, msg))

    def on_timer(self):
        if self.active and time.monotonic() - self.last_arrival > self.max_age:
            self.clear()


def main():
    rclpy.init()
    node = BearingRay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
