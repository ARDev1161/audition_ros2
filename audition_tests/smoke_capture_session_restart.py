#!/usr/bin/env python3
"""Graph regression: capture restart must fence an old publisher, even with no gap."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AudioFrame, SourceAudioFrame, AcousticSourceArray


class Probe(Node):
    def __init__(self):
        super().__init__("capture_session_restart_probe")
        self.publisher = self.create_publisher(
            AudioFrame, "/audition/audio/raw", qos_profile_sensor_data)
        self.audio = []
        self.generations = []
        self.create_subscription(
            SourceAudioFrame, "/audition/internal/source_audio",
            self.on_audio, qos_profile_sensor_data)
        self.create_subscription(
            AcousticSourceArray, "/audition/sources", self.on_sources, 10)

    def on_audio(self, msg):
        if msg.pcm:
            self.audio.append((msg.source_generation, round(float(msg.pcm[0]), 2)))

    def on_sources(self, msg):
        if msg.sources and any(s.state == s.ACTIVE for s in msg.sources):
            self.generations.append(msg.generation)

    def wait(self, predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def send(self, session, sequence, amplitude, nanos):
        frame = AudioFrame()
        frame.header.frame_id = "microphone_array"
        frame.header.stamp.sec = 42
        frame.header.stamp.nanosec = nanos
        frame.capture_session_id = session
        frame.sequence_number = sequence
        frame.sample_rate_hz = 16000
        frame.channel_count = 1
        frame.frame_count = 160
        frame.layout = AudioFrame.INTERLEAVED
        frame.channel_indices = [0]
        frame.pcm = [amplitude] * 160
        self.publisher.publish(frame)


def main():
    rclpy.init()
    probe = Probe()
    try:
        assert probe.wait(lambda: probe.publisher.get_subscription_count() > 0), (
            "mono source subscription unavailable")
        a, b = "a" * 32, "b" * 32
        probe.send(a, 0, 0.1, 0)
        assert probe.wait(lambda: (1, 0.1) in probe.audio and 1 in probe.generations), (
            "first session not admitted")
        probe.send(b, 1, 0.5, 10000000)
        assert probe.wait(lambda: (2, 0.5) in probe.audio and 2 in probe.generations), (
            "capture incarnation did not force a generation reset")
        before = len(probe.audio)
        probe.send(a, 2, 0.9, 20000000)
        probe.wait(lambda: len(probe.audio) > before, seconds=0.5)
        assert len(probe.audio) == before, "old capture process replay was accepted"
        probe.send("INVALID", 2, 0.8, 20000000)
        probe.wait(lambda: len(probe.audio) > before, seconds=0.5)
        assert len(probe.audio) == before, "malformed capture ID was accepted"
        probe.send(b, 2, 0.6, 20000000)
        assert probe.wait(lambda: (2, 0.6) in probe.audio), (
            "valid new-session PCM not admitted after rejected replay")
        assert max(probe.generations) == 2, probe.generations
        print("PASS: ROS graph capture restart, old-session replay fence and recovery")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
