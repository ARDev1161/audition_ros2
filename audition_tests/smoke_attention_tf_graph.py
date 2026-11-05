#!/usr/bin/env python3
"""Real DDS/TF2 graph regression for the advisory bearing bridge."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AttentionTarget
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

NS = "/robot_ci/tf"


class Probe(Node):
    def __init__(self):
        super().__init__("attention_tf_graph_probe")
        self.results = []
        self.publisher = self.create_publisher(AttentionTarget, NS + "/attention/target", 10)
        self.subscription = self.create_subscription(
            AttentionTarget, NS + "/attention/target_base", self.results.append, 10)
        self.tf = StaticTransformBroadcaster(self)

    def send(self, source="mic_ci", valid=True, age_ns=0):
        msg = AttentionTarget()
        msg.valid = valid
        msg.header.frame_id = source
        stamp = self.get_clock().now().nanoseconds - age_ns
        msg.header.stamp.sec = stamp // 1000000000
        msg.header.stamp.nanosec = stamp % 1000000000
        msg.source_id = "source-ci"
        msg.scene_generation = 7
        msg.direction_unit.x = 1.0
        self.publisher.publish(msg)

    def wait_for(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if any(predicate(x) for x in self.results):
                return True
        return False


def main():
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.publisher.get_subscription_count() > 0 and
                    node.count_publishers(NS + "/attention/target_base") > 0):
                break
        else:
            raise AssertionError("graph discovery")
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "mic_ci"
        tf.transform.translation.x = 5.0  # Direction must ignore translation.
        tf.transform.rotation.z = math.sqrt(0.5)
        tf.transform.rotation.w = math.sqrt(0.5)
        for _ in range(8):
            node.tf.sendTransform(tf)
            rclpy.spin_once(node, timeout_sec=0.1)
        # Retry while the static TF listener discovers its transient-local sample.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(x.valid for x in node.results):
            node.send()
            rclpy.spin_once(node, timeout_sec=0.15)
        good = [x for x in node.results if x.valid]
        assert good, "timestamped TF did not resolve"
        out = good[-1]
        assert out.header.frame_id == "base_link"
        assert out.source_id == "source-ci" and out.scene_generation == 7
        assert abs(out.direction_unit.x) < 0.001 and abs(out.direction_unit.y - 1.0) < 0.001
        assert abs(out.direction_unit.z) < 0.001, "translation leaked into bearing"
        node.results.clear()
        node.send(source="unknown_mic")
        assert node.wait_for(lambda x: not x.valid and x.reason == "tf_unavailable"), "missing TF accepted"
        node.results.clear()
        node.send(age_ns=3_000_000_000)
        assert node.wait_for(lambda x: not x.valid and x.reason != "tf_transformed"), "stale observation accepted"
        node.results.clear()
        node.send(valid=False)
        assert node.wait_for(lambda x: not x.valid), "invalid upstream target accepted"
        print("PASS: TF rotation, translation isolation, provenance, missing TF, stale and invalid input")
        return 0
    except AssertionError as error:
        print("FAIL:", error, [(x.valid, x.reason) for x in node.results[-10:]], file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
