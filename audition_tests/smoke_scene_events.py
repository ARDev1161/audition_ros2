#!/usr/bin/env python3
"""M2 canonical publication graph: injected internal transitions, no fake classifier."""
import sys
import time
import rclpy
from rclpy.node import Node
from audition_msgs.msg import AcousticEvent, AcousticScene, SoundClassification


class Probe(Node):
    def __init__(self):
        super().__init__("audition_m2_event_smoke")
        self.scene_generation = None
        self.frame_id = None
        self.active = False
        self.ended = False
        self.started = False
        self.classified = False
        self.pub = self.create_publisher(AcousticEvent, "/audition/internal/events", 20)
        self.cpub = self.create_publisher(
            SoundClassification, "/audition/internal/classifications", 20)
        self.create_subscription(AcousticScene, "/audition/scene", self.on_scene, 10)
        self.create_subscription(AcousticEvent, "/audition/events", self.on_event, 20)
        self.create_subscription(
            SoundClassification, "/audition/classifications", self.on_class, 20)

    def on_scene(self, msg):
        self.scene_generation = msg.generation
        self.frame_id = msg.header.frame_id
        self.active = any(e.event_id == "ci-speech" for e in msg.active_events)

    def on_event(self, msg):
        if msg.event_id == "ci-speech" and msg.transition == AcousticEvent.STARTED:
            self.started = True
        if msg.event_id == "ci-speech" and msg.transition == AcousticEvent.ENDED:
            self.ended = True

    def on_class(self, msg):
        if msg.source_id == "ci-source" and msg.class_label == "Speech":
            self.classified = True

    def event(self, transition):
        e = AcousticEvent()
        e.header.frame_id = self.frame_id
        e.header.stamp = self.get_clock().now().to_msg()
        e.source_id = "ci-source"
        e.source_generation = self.scene_generation
        e.event_id = "ci-speech"
        e.class_label = "Speech"
        e.taxonomy_version = "AudioSet-521"
        e.model_id = "integration-test"
        e.score_kind = AcousticEvent.UNCALIBRATED_SCORE
        e.score = 0.9
        e.transition = transition
        e.observed_start = e.header.stamp
        e.observed_end = e.header.stamp
        return e

    def classification(self):
        c = SoundClassification()
        c.header.frame_id = self.frame_id
        c.header.stamp = self.get_clock().now().to_msg()
        c.source_id = "ci-source"
        c.source_generation = self.scene_generation
        c.class_label = "Speech"
        c.taxonomy_version = "AudioSet-521"
        c.model_id = "integration-test"
        c.score_kind = SoundClassification.UNCALIBRATED_SCORE
        c.score = 0.9
        return c


def wait(node, predicate, deadline):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 18
    try:
        if not wait(probe, lambda: probe.scene_generation is not None, deadline):
            print("FAIL: no canonical scene", file=sys.stderr)
            return 1
        if not wait(
            probe, lambda: probe.pub.get_subscription_count() > 0 and
            probe.cpub.get_subscription_count() > 0, deadline
        ):
            print("FAIL: scene does not subscribe to internal events", file=sys.stderr)
            return 1
        probe.cpub.publish(probe.classification())
        probe.pub.publish(probe.event(AcousticEvent.STARTED))
        if not wait(probe, lambda: probe.started and probe.active and probe.classified, deadline):
            print("FAIL: STARTED/classification not canonically published", file=sys.stderr)
            return 1
        probe.pub.publish(probe.event(AcousticEvent.ENDED))
        if not wait(probe, lambda: probe.ended and not probe.active, deadline):
            print("FAIL: ENDED did not remove active event", file=sys.stderr)
            return 1
        print("PASS: class forwarding, event STARTED/ENDED and scene active set")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
