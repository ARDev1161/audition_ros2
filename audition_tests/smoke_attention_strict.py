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
    # The timer can publish an unrelated no_scene/scene_timeout result
    # while DDS discovery converges. Match the response to this exact
    # published scene, rather than assuming the next callback is its reply.
    count = len(node.received)
    node.pub.publish(scene)
    def matching():
        return [result for result in node.received[count:]
                if result.header.stamp == scene.header.stamp
                and result.header.frame_id == scene.header.frame_id
                and result.scene_generation == scene.generation]
    assert wait(node, lambda: bool(matching())), (
        "no matching attention result for stamp " + str(scene.header.stamp.sec)
        + "; graph=" + str(node.get_topic_names_and_types()))
    result = matching()[0]
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
        # A discovered subscription does not guarantee the first DDS sample
        # has crossed the reader's initialization boundary. Retry the initial
        # snapshot with a stable stamp until its correlated advisory arrives.
        # Strict replay fencing accepts a stamp only once. Re-sending the
        # identical initial stamp can be admitted while the reply is still
        # undiscovered, then every retry is correctly rejected as replay.
        # Advance the stamp on each DDS startup retry and continue the test
        # relative to the stamp whose acknowledgement we actually observed.
        deadline = time.monotonic() + 10
        initial_stamp = 10
        while time.monotonic() < deadline:
            initial = node.scene(initial_stamp)
            before = len(node.received)
            node.pub.publish(initial)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(x.valid and x.source_id == "a" and
                   x.header.stamp == initial.header.stamp and
                   x.scene_generation == 1 for x in node.received[before:]):
                break
            initial_stamp += 1
        else:
            raise AssertionError("initial scene not accepted after graph discovery")
        send(node, node.scene(initial_stamp + 1, b=0.95), expected_id="a")
        send(node, node.scene(initial_stamp + 1, b=0.95), expected_reason="regressed_scene_stamp")
        send(node, node.scene(initial_stamp + 2, b=0.95), expected_id="a")
        send(node, node.scene(initial_stamp + 3, b=0.95), expected_id="b")
        send(node, node.scene(initial_stamp + 2, a=0.99), expected_reason="regressed_scene_stamp")
        send(node, node.scene(initial_stamp + 4, a=0.99, b=0.4), expected_id="b")
        send(node, node.scene(initial_stamp + 5, a=0.99, b=0.4), expected_id="b")
        send(node, node.scene(initial_stamp + 6, a=0.99, b=0.4), expected_id="a")
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
