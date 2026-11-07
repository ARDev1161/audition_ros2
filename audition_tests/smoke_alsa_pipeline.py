#!/usr/bin/env python3
"""Live ALSA (virtual 'null' PCM) ROS integration contract smoke."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AudioFrame, AcousticScene
from audition_msgs.srv import GetCaptureStatus


class Probe(Node):
    def __init__(self):
        super().__init__("audition_alsa_smoke")
        self.frames = 0
        self.scene_count = 0
        self.invalid = False
        self.seq = None
        self.status_future = None
        self.status_client = self.create_client(
            GetCaptureStatus, "/audition/audio_capture/get_capture_status")
        self.create_subscription(
            AudioFrame, "/audition/audio/raw", self.on_audio, qos_profile_sensor_data)
        self.create_subscription(
            AcousticScene, "/audition/scene", self.on_scene, 10)

    def on_audio(self, msg):
        if (
            msg.sample_rate_hz != 16000
            or msg.channel_count != 6
            or msg.frame_count == 0
            or msg.frame_count > 160
            or msg.layout != AudioFrame.INTERLEAVED
            or list(msg.channel_indices) != [0, 1, 2, 3, 4, 5]
            or len(msg.pcm) != msg.channel_count * msg.frame_count
            or msg.header.frame_id != "microphone_array"
            or any(not math.isfinite(x) or abs(x) > 1 for x in msg.pcm)
        ):
            print(
                f"Bad ALSA AudioFrame: rate={msg.sample_rate_hz} channels={msg.channel_count} "
                f"frames={msg.frame_count} frame_id={msg.header.frame_id!r} "
                f"indices={list(msg.channel_indices)} samples={len(msg.pcm)}",
                file=sys.stderr,
            )
            self.invalid = True
        if self.seq is not None and msg.sequence_number <= self.seq:
            self.invalid = True
        self.seq = msg.sequence_number
        self.frames += 1

    def on_scene(self, msg):
        if msg.header.frame_id == "microphone_array" and not msg.sources:
            self.scene_count += 1


def main():
    rclpy.init()
    probe = Probe()
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if probe.invalid:
                print("ALSA smoke FAIL: invalid PCM metadata", file=sys.stderr)
                return 1
            if probe.frames >= 8 and probe.scene_count >= 2:
                if probe.status_future is None and probe.status_client.service_is_ready():
                    probe.status_future = probe.status_client.call_async(
                        GetCaptureStatus.Request())
                if probe.status_future is not None and probe.status_future.done():
                    status = probe.status_future.result()
                    if status is None or not status.connected or status.terminal_failure:
                        print("ALSA capture service reported unhealthy null PCM", file=sys.stderr)
                        return 1
                    if status.published_frames < 8 or status.reconnect_failures:
                        print("ALSA capture counters invalid", file=sys.stderr)
                        return 1
                    print("ALSA smoke PASS: live PCM, canonical scene, capture status service")
                    return 0
        print(
            f"ALSA smoke FAIL: frames={probe.frames} scenes={probe.scene_count}",
            file=sys.stderr,
        )
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
