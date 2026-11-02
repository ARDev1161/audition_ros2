#!/usr/bin/env python3
"""M2 canonical publication graph: injected internal transitions, no fake classifier."""
import sys
import time
import rclpy
from rclpy.node import Node
from audition_msgs.msg import AcousticEvent, AcousticScene, AcousticSource, AcousticSourceArray, SoundClassification, Transcript


class Probe(Node):
    def __init__(self):
        super().__init__("audition_m2_event_smoke")
        self.scene_generation = None
        self.frame_id = None
        self.active = False
        self.ended = False
        self.updated = False
        self.cancelled = False
        self.reset_active = False
        self.transcript_seen = 0
        self.stale_seen = False
        self.started = False
        self.classified = False
        self.source_present = False
        self.heartbeat = False
        self.retire_started = False
        self.retire_cancelled = False
        self.late_seen = False
        self.ghost_event = False
        self.ghost_classified = False
        self.ghost_transcript = False
        self.pub = self.create_publisher(AcousticEvent, "/audition/internal/events", 20)
        self.tpub = self.create_publisher(Transcript, "/audition/internal/transcripts", 20)
        self.create_subscription(Transcript, "/audition/transcripts", self.on_transcript, 20)
        self.spatial_pub = self.create_publisher(
            AcousticSourceArray, "/audition/internal/spatial_sources", 10)
        self.cpub = self.create_publisher(
            SoundClassification, "/audition/internal/classifications", 20)
        self.create_subscription(AcousticScene, "/audition/scene", self.on_scene, 10)
        self.create_subscription(AcousticEvent, "/audition/events", self.on_event, 20)
        self.create_subscription(
            SoundClassification, "/audition/classifications", self.on_class, 20)
        self.create_timer(0.1, self.send_source)

    def send_source(self):
        if not self.heartbeat or self.scene_generation is None:
            return
        frame = AcousticSourceArray()
        frame.header.frame_id = self.frame_id
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.generation = self.scene_generation
        src = AcousticSource()
        src.header = frame.header
        src.source_id = "ci-source"
        src.source_generation = frame.generation
        src.state = AcousticSource.ACTIVE
        src.spatial_track_id = "fixture"
        src.has_direction = True
        src.direction_unit.x = 1.0
        src.first_seen = frame.header.stamp
        src.last_seen = frame.header.stamp
        frame.sources.append(src)
        self.spatial_pub.publish(frame)

    def on_scene(self, msg):
        self.scene_generation = msg.generation
        self.frame_id = msg.header.frame_id
        self.active = any(e.event_id == "ci-speech" for e in msg.active_events)
        self.reset_active = any(e.event_id == "ci-reset" for e in msg.active_events)
        self.source_present = any(s.source_id == "ci-source" for s in msg.sources)

    def on_event(self, msg):
        if msg.event_id == "ci-speech" and msg.transition == AcousticEvent.STARTED:
            self.started = True
        if msg.event_id == "ci-speech" and msg.transition == AcousticEvent.ENDED:
            self.ended = True
        if msg.event_id == "ci-speech" and msg.transition == AcousticEvent.UPDATED:
            self.updated = True
        if msg.event_id == "ci-retire" and msg.transition == AcousticEvent.STARTED:
            self.retire_started = True
        if msg.event_id == "ci-retire" and msg.transition == AcousticEvent.CANCELLED:
            self.retire_cancelled = msg.termination_reason == "source_ended"
        if msg.event_id == "ci-ghost":
            self.ghost_event = True
        if msg.event_id == "ci-reset" and msg.transition == AcousticEvent.CANCELLED:
            self.cancelled = msg.termination_reason == "spatial_stream_reset"

    def on_transcript(self, msg):
        if msg.text == "Привет":
            self.transcript_seen += 1
        if msg.text == "stale":
            self.stale_seen = True
        if msg.text == "late":
            self.late_seen = True
        if msg.text == "ghost":
            self.ghost_transcript = True

    def on_class(self, msg):
        if msg.source_id == "ci-source" and msg.class_label == "Speech":
            self.classified = True
        if msg.class_label == "Ghost":
            self.ghost_classified = True

    def event(self, transition, event_id='ci-speech'):
        e = AcousticEvent()
        e.header.frame_id = self.frame_id
        e.header.stamp = self.get_clock().now().to_msg()
        e.source_id = "ci-source"
        e.source_generation = self.scene_generation
        e.event_id = event_id
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
    deadline = time.monotonic() + 27
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
        if not wait(probe, lambda: probe.tpub.get_subscription_count() > 0, deadline):
            print("FAIL: scene does not subscribe to transcripts", file=sys.stderr)
            return 1
        spoken = Transcript()
        spoken.header.frame_id = probe.frame_id
        spoken.header.stamp = probe.get_clock().now().to_msg()
        spoken.source_id = "ci-source"
        spoken.source_generation = probe.scene_generation
        spoken.language = "ru"
        spoken.text = "Привет"
        spoken.model_id = "integration-test"
        spoken.is_final = True
        spoken.observed_start = spoken.header.stamp
        spoken.observed_end = spoken.header.stamp
        probe.tpub.publish(spoken)
        if not wait(probe, lambda: probe.transcript_seen == 1, deadline):
            print("FAIL: valid transcript was not forwarded", file=sys.stderr)
            return 1
        spoken.text = "stale"
        spoken.source_generation += 99
        probe.tpub.publish(spoken)
        # A spin window allows stale messages to be delivered and rejected.
        for _ in range(10):
            rclpy.spin_once(probe, timeout_sec=0.02)
        if probe.stale_seen:
            print("FAIL: stale transcript crossed generation fence", file=sys.stderr)
            return 1
        probe.cpub.publish(probe.classification())
        probe.pub.publish(probe.event(AcousticEvent.STARTED))
        if not wait(probe, lambda: probe.started and probe.active and probe.classified, deadline):
            print("FAIL: STARTED/classification not canonically published", file=sys.stderr)
            return 1
        probe.pub.publish(probe.event(AcousticEvent.UPDATED))
        if not wait(probe, lambda: probe.updated and probe.active, deadline):
            print("FAIL: UPDATED transition missing", file=sys.stderr)
            return 1
        probe.pub.publish(probe.event(AcousticEvent.ENDED))
        if not wait(probe, lambda: probe.ended and not probe.active, deadline):
            print("FAIL: ENDED did not remove active event", file=sys.stderr)
            return 1
        probe.pub.publish(probe.event(AcousticEvent.STARTED, "ci-reset"))
        if not wait(probe, lambda: probe.reset_active, deadline):
            print("FAIL: reset fixture never became active", file=sys.stderr)
            return 1
        new_stream = AcousticSourceArray()
        new_stream.header.frame_id = probe.frame_id
        new_stream.header.stamp = probe.get_clock().now().to_msg()
        new_stream.generation = probe.scene_generation + 1
        probe.spatial_pub.publish(new_stream)
        if not wait(probe, lambda: probe.cancelled and not probe.reset_active, deadline):
            print("FAIL: stream reset did not cancel active event", file=sys.stderr)
            return 1
        print("PASS: class forwarding, STARTED/UPDATED/ENDED, stream-reset CANCELLED")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
