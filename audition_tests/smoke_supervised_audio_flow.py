#!/usr/bin/env python3
"""ROS graph smoke: live virtual ALSA PCM must reach supervisor watchdog."""
import time
import rclpy
from audition_msgs.srv import GetPipelineStatus


def main():
    rclpy.init()
    node = rclpy.create_node("supervised_audio_flow_ci")
    try:
        client = node.create_client(GetPipelineStatus, "/audition/pipeline/get_status")
        assert client.wait_for_service(timeout_sec=30)
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            future = client.call_async(GetPipelineStatus.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.exception() is None
            status = future.result()
            assert status.running and status.graph_healthy, status
            assert status.audio_watchdog_enabled, status
            assert status.source_watchdog_enabled, status
            assert status.semantic_watchdog_enabled, status
            if (status.accepted_audio_frames > 0 and status.audio_healthy and
                    status.accepted_source_frames > 0 and status.source_healthy and
                    status.accepted_semantic_frames > 0 and status.semantic_healthy):
                assert status.audio_miss_count == 0, status
                assert status.source_miss_count == 0, status
                assert status.semantic_miss_count == 0, status
                print("Supervised ALSA capture, spatial and semantic ingress progress PASS")
                return
            time.sleep(0.2)
        raise AssertionError("supervisor did not observe live ALSA capture and mono source PCM")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
