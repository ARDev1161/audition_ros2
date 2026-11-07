#!/usr/bin/env python3
"""ROS graph smoke: decoded WAV audio reaches a coherent empty AcousticScene."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AudioFrame, AcousticScene


class Probe(Node):
    def __init__(self):
        super().__init__("audition_wav_smoke")
        self.saw_pcm = False
        self.saw_scene = False
        self.create_subscription(
            AudioFrame, "/audition/audio/raw", self.on_audio, qos_profile_sensor_data
        )
        self.create_subscription(
            AcousticScene, "/audition/scene", self.on_scene, 10
        )

    def on_audio(self, msg):
        if (
            msg.sample_rate_hz == 16000
            and msg.channel_count == 1
            and msg.layout == AudioFrame.INTERLEAVED
            and msg.frame_count == len(msg.pcm)
            and msg.header.frame_id == "microphone_array"
            and any(abs(float(sample)) > 0.01 for sample in msg.pcm)
        ):
            self.saw_pcm = True

    def on_scene(self, msg):
        if msg.header.frame_id == "microphone_array" and not msg.sources:
            self.saw_scene = True


def main():
    rclpy.init()
    probe = Probe()
    deadline = time.monotonic() + 18
    try:
        while time.monotonic() < deadline and not (probe.saw_pcm and probe.saw_scene):
            rclpy.spin_once(probe, timeout_sec=0.2)
        if not probe.saw_pcm or not probe.saw_scene:
            print(
                f"Smoke FAILED: pcm={probe.saw_pcm}, scene={probe.saw_scene}",
                file=sys.stderr,
            )
            return 1
        print("Smoke PASS: real decoded WAV PCM and no fabricated acoustic sources")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
