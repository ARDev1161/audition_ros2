#!/usr/bin/env python3
"""M2 no-ODAS route: actual PCM and a non-spatial canonical source over DDS."""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AcousticScene, SourceAudioFrame


class Probe(Node):
    def __init__(self):
        super().__init__("audition_mono_smoke")
        self.sources = set()
        self.audio = set()
        self.error = None
        self.create_subscription(AcousticScene, "/audition/scene", self.on_scene, 10)
        self.create_subscription(
            SourceAudioFrame, "/audition/internal/source_audio",
            self.on_audio, qos_profile_sensor_data
        )

    def on_scene(self, msg):
        for source in msg.sources:
            if (source.has_direction or source.has_range or source.has_position
                    or source.has_angular_variance or source.spatial_track_id
                    or source.speaker_id or msg.header.frame_id != "microphone_array"
                    or source.source_generation != msg.generation):
                self.error = "mono source fabricated geometry or identity"
            else:
                self.sources.add((source.source_id, source.source_generation))

    def on_audio(self, msg):
        if (msg.sample_rate_hz == 16000 and msg.frame_count == len(msg.pcm)
                and msg.frame_count > 0 and msg.header.frame_id == "microphone_array"
                and any(abs(float(v)) > 0.01 for v in msg.pcm)):
            self.audio.add((msg.source_id, msg.source_generation))


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 18
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.error or node.sources.intersection(node.audio):
                break
        if node.error or not node.sources.intersection(node.audio):
            print(
                "Mono graph FAILED: scene=%s audio=%s error=%s" %
                (node.sources, node.audio, node.error), file=sys.stderr
            )
            return 1
        print("Mono graph PASS: real 16 kHz PCM and canonical source without DOA")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
