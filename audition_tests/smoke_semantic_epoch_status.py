#!/usr/bin/env python3
"""Real ROS graph: semantic admission status and stale PCM rejection."""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import SourceAudioFrame, AcousticSourceArray
from audition_msgs.srv import GetSemanticStatus


def status(node, client):
    assert client.wait_for_service(timeout_sec=10), client.srv_name
    future = client.call_async(GetSemanticStatus.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done() and future.exception() is None
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node("semantic_epoch_ci")
    try:
        client = node.create_client(
            GetSemanticStatus, "/audition/semantic_perception/get_semantic_status")
        deadline = time.monotonic() + 18
        while time.monotonic() < deadline:
            value = status(node, client)
            if (value.have_stream and value.source_generation >= 1
                    and value.admitted_sources > 0
                    and value.accepted_audio_frames > 0):
                break
            time.sleep(0.1)
        else:
            raise AssertionError("No canonical semantic admission / PCM: " + str(value))

        assert value.last_source_sequence > 0, value
        initial_rejected = value.rejected_source_snapshots
        # Simulate a second publisher replaying a zero-order canonical snapshot.
        stale_pub = node.create_publisher(
            AcousticSourceArray, "/audition/sources", 10)
        stale = AcousticSourceArray()
        stale.header.frame_id = value.frame_id
        stale.generation = value.source_generation
        stale.sequence_number = 0
        for _ in range(8):
            stale_pub.publish(stale)
            time.sleep(0.05)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            value = status(node, client)
            if value.rejected_source_snapshots > initial_rejected:
                break
            time.sleep(0.1)
        assert value.rejected_source_snapshots > initial_rejected, value
        assert value.admitted_sources > 0, value

        initial_drops = value.dropped_audio_frames
        pub = node.create_publisher(
            SourceAudioFrame, "/audition/internal/source_audio",
            qos_profile_sensor_data)
        bad = SourceAudioFrame()
        bad.header.frame_id = value.frame_id
        bad.source_id = "unknown-injected-source"
        bad.source_generation = value.source_generation
        bad.sample_rate_hz = 16000
        bad.frame_count = 16
        bad.pcm = [0.0] * 16
        for _ in range(8):
            pub.publish(bad)
            time.sleep(0.05)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            value = status(node, client)
            if value.dropped_audio_frames > initial_drops:
                break
            time.sleep(0.1)
        assert value.dropped_audio_frames > initial_drops, value
        assert value.have_stream and value.source_generation >= 1
        print("Semantic epoch graph PASS: admitted PCM, orphan rejection, status service")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
