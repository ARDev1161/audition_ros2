#!/usr/bin/env python3
"""M3 ROS graph contract: actual attention node, canonical scene messages, expiry."""
import sys
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AcousticScene, AcousticSource, AttentionTarget


NS = "/robot_ci/audition"


class Probe(Node):
    def __init__(self):
        super().__init__("attention_graph_probe")
        self.targets = []
        self.publisher = self.create_publisher(AcousticScene, NS + "/scene", 10)
        self.subscription = self.create_subscription(
            AttentionTarget, NS + "/attention/target", self.targets.append, 10)

    def scene(self, generation=1, source_id="ci-a", score=0.8, frame="ci_mic"):
        msg = AcousticScene()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.generation = generation
        if source_id:
            src = AcousticSource()
            src.header = msg.header
            src.source_id = source_id
            src.source_generation = generation
            src.state = AcousticSource.ACTIVE
            src.has_direction = True
            src.direction_unit.x = 1.0
            src.activity_score = score
            src.last_seen = msg.header.stamp
            msg.sources.append(src)
        return msg


def wait(node, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    node = Probe()
    try:
        if not wait(node, lambda: node.publisher.get_subscription_count() >= 1
                    and node.subscription.get_publisher_count() >= 1, 35):
            raise AssertionError("attention graph not discovered in namespaced launch")
        # Re-publish until the real ROS node processes a valid target.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not any(
                t.valid and t.source_id == "ci-a" for t in node.targets):
            node.publisher.publish(node.scene())
            rclpy.spin_once(node, timeout_sec=0.1)
        valid = [t for t in node.targets if t.valid and t.source_id == "ci-a"]
        assert valid, "canonical scene did not produce valid advisory target"
        assert valid[-1].header.frame_id == "ci_mic"
        assert valid[-1].scene_generation == 1
        assert valid[-1].direction_unit.x == 1.0
        # An empty accepted scene must immediately revoke the old bearing.
        count = len(node.targets)
        node.publisher.publish(node.scene(source_id=""))
        assert wait(node, lambda: any(
            not t.valid and t.reason == "no_eligible_bearing"
            for t in node.targets[count:])), "empty scene did not invalidate"
        # New source generation is independently selectable.
        count = len(node.targets)
        node.publisher.publish(node.scene(generation=2, source_id="ci-b"))
        assert wait(node, lambda: any(
            t.valid and t.scene_generation == 2 and t.source_id == "ci-b"
            for t in node.targets[count:])), "new generation was not selected"
        # The timer must revoke an unattended target after the configured 500ms.
        count = len(node.targets)
        assert wait(node, lambda: any(
            not t.valid and t.reason == "scene_timeout"
            for t in node.targets[count:]), 4), "stale target remained actionable"
        assert not any(t.valid for t in node.targets[count:]), (
            "target became valid again without a new scene")
        print("PASS: namespaced canonical scene -> advisory target, revoke, generation, expiry")
        return 0
    except AssertionError as exc:
        print("FAIL:", exc, file=sys.stderr)
        print("targets:", [(t.valid, t.reason, t.source_id, t.scene_generation)
                            for t in node.targets[-15:]], file=sys.stderr)
        print("topics:", node.get_topic_names_and_types(), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
