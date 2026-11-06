#!/usr/bin/env python3
"""Real ROS graph: semantic admission status and stale PCM rejection."""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import SourceAudioFrame, AcousticSourceArray
from audition_msgs.srv import GetSemanticStatus


def status(node, client):
    assert client.service_is_ready(), "semantic status service disappeared: " + client.srv_name
    future = client.call_async(GetSemanticStatus.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done() and future.exception() is None
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node("semantic_epoch_ci")
    captured = []
    capture_sub = node.create_subscription(
        SourceAudioFrame, "/audition/internal/source_audio",
        lambda msg: captured.append(msg) if len(captured) < 1 else None,
        qos_profile_sensor_data)
    try:
        client = node.create_client(
            GetSemanticStatus, "/audition/semantic_perception/get_semantic_status")
        assert client.wait_for_service(timeout_sec=35), "semantic service startup timed out: " + client.srv_name
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
        assert len(value.scene_session_id) == 32, value
        assert value.scene_session_resets == 0, value
        # Disabled workers admit PCM, but cannot claim completed inference.
        assert not value.classifier_active and not value.speech_active, value
        assert value.classifier_completed_windows == 0, value
        assert value.asr_completed_segments == 0, value
        assert value.classifier_last_completion_age_ms == 0, value
        assert value.asr_last_completion_age_ms == 0, value
        assert value.inference_poll_failures == 0, value
        assert value.classifier_active_sources == 0, value
        assert value.classifier_queued_sources == 0, value
        assert value.classifier_inferencing_sources == 0, value
        assert value.classifier_dropped_windows_active == 0, value
        assert not value.classifier_has_inference_timing, value
        assert not value.classifier_has_queue_wait_timing, value
        assert value.asr_queued_segments == 0, value
        assert value.asr_buffered_results == 0, value
        assert value.asr_retired_generations == 0, value
        assert value.asr_completed_segments_native == 0, value
        assert value.asr_dropped_segments_native == 0, value
        assert not value.asr_worker_error, value
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

        # Replay actual mono PCM after the live source has advanced.
        deadline = time.monotonic() + 6
        while not captured and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert captured, "No real source PCM available for replay regression"
        time.sleep(0.3)
        initial_reorders = status(node, client).out_of_order_audio_frames
        replay_pub = node.create_publisher(
            SourceAudioFrame, "/audition/internal/source_audio",
            qos_profile_sensor_data)
        for _ in range(8):
            replay_pub.publish(captured[0])
            time.sleep(0.05)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            value = status(node, client)
            if value.out_of_order_audio_frames > initial_reorders:
                break
            time.sleep(0.1)
        assert value.out_of_order_audio_frames > initial_reorders, value

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
        print("Semantic epoch graph PASS: admitted PCM, replay/orphan rejection, status service")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
