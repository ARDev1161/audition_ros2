#!/usr/bin/env python3
"""M3r: actual ROS graph replay fence and challenger confirmation with YAML settings."""
import sys
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AcousticScene, AcousticSource, AttentionTarget

NS = "/robot_ci/attention_strict"


class Probe(Node):
    def __init__(self):
        super().__init__("attention_strict_probe")
        self.received = []
        self.pub = self.create_publisher(AcousticScene, NS + "/scene", 10)
        self.sub = self.create_subscription(
            AttentionTarget, NS + "/attention/target", self.received.append, 10)

    def scene(self, stamp, a=0.7, b=0.5, generation=1):
        scene = AcousticScene()
        scene.header.frame_id = "strict_mic"
        scene.header.stamp.sec = stamp
        scene.generation = generation
        for name, score in (("a", a), ("b", b)):
            source = AcousticSource()
            source.header = scene.header
            source.source_id = name
            source.source_generation = generation
            source.state = AcousticSource.ACTIVE
            source.has_direction = True
            source.direction_unit.x = 1.0
            source.activity_score = score
            source.last_seen = scene.header.stamp
            scene.sources.append(source)
        return scene


def wait(node, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def send(node, scene, expected_id=None, expected_reason=None):
    count = len(node.received)
    node.pub.publish(scene)
    assert wait(node, lambda: len(node.received) > count), (
        "no attention result for stamp " + str(scene.header.stamp.sec))
    result = node.received[count]
    if expected_id is not None:
        assert result.valid and result.source_id == expected_id, (
            "expected " + expected_id + ", got " + str((result.valid, result.source_id, result.reason)))
    if expected_reason is not None:
        assert not result.valid and result.reason == expected_reason, (
            "expected " + expected_reason + ", got " + str((result.valid, result.reason)))
    return result


def main():
    rclpy.init()
    node = Probe()
    try:
        assert wait(node, lambda: node.pub.get_subscription_count() > 0 and
                    node.count_publishers(NS + "/attention/target") > 0, 35), (
            "strict attention graph not discovered: " + str(node.get_topic_names_and_types()))
        send(node, node.scene(10), expected_id="a")
        send(node, node.scene(11, b=0.95), expected_id="a")
        send(node, node.scene(11, b=0.95), expected_reason="regressed_scene_stamp")
        send(node, node.scene(12, b=0.95), expected_id="a")
        send(node, node.scene(13, b=0.95), expected_id="b")
        send(node, node.scene(12, a=0.99), expected_reason="regressed_scene_stamp")
        send(node, node.scene(14, a=0.99, b=0.4), expected_id="b")
        send(node, node.scene(15, a=0.99, b=0.4), expected_id="b")
        send(node, node.scene(16, a=0.99, b=0.4), expected_id="a")
        send(node, node.scene(0, a=0.99), expected_reason="invalid_scene_stamp")
        send(node, node.scene(1, a=0.99, generation=2), expected_id="a")
        print("PASS: strict ROS replay, bounded confirmation, generation reset")
        return 0
    except AssertionError as exc:
        print("FAIL:", exc, file=sys.stderr)
        print("recent targets:", [(x.valid, x.source_id, x.reason)
              for x in node.received[-15:]], file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
