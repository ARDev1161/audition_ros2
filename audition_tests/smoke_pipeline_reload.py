#!/usr/bin/env python3
"""Real Humble graph: M2o dry-run, rejected preflight, and owned restart."""
import time
import rclpy
from audition_msgs.srv import ReloadPipeline, GetCapabilities, GetPipelineStatus


def call(node, client, request, timeout=90):
    assert client.wait_for_service(timeout_sec=40), client.srv_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done() and future.exception() is None, client.srv_name
    return future.result()


def wait_audio(node, client, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = call(node, client, GetPipelineStatus.Request())
        if status.accepted_audio_frames > 0 and status.audio_healthy:
            return status
        time.sleep(0.2)
    raise AssertionError("no observed capture PCM after supervised launch")


def main():
    rclpy.init()
    node = rclpy.create_node("pipeline_reload_ci")
    try:
        reload = node.create_client(ReloadPipeline, "/audition/pipeline/reload")
        status_client = node.create_client(GetPipelineStatus, "/audition/pipeline/get_status")
        capabilities = node.create_client(
            GetCapabilities, "/audition/semantic_perception/get_capabilities")
        assert call(node, capabilities, GetCapabilities.Request()).profile_name == "replay_ci"
        initial = call(node, status_client, GetPipelineStatus.Request())
        assert initial.running and initial.owned_process_present, initial
        assert initial.active_profile == "replay_ci" and initial.reload_epoch == 0
        assert not initial.fault and initial.graph_healthy
        assert initial.graph_miss_count == 0
        assert not initial.audio_watchdog_enabled  # finite replay may reach EOF
        assert wait_audio(node, status_client).accepted_audio_frames > 0
        dry = ReloadPipeline.Request()
        dry.dry_run = True
        result = call(node, reload, dry)
        assert result.success and result.reload_epoch == 0, result
        invalid = ReloadPipeline.Request()
        invalid.profile = "/definitely/not/a/profile.yaml"
        result = call(node, reload, invalid)
        assert not result.success and result.active_profile == "replay_ci", result
        assert result.reload_epoch == 0 and not result.rollback_succeeded
        result = call(node, reload, ReloadPipeline.Request())
        assert result.success and result.reload_epoch == 1, result
        healthy = call(node, status_client, GetPipelineStatus.Request())
        assert healthy.running and healthy.owned_process_present, healthy
        assert healthy.last_good_profile == "replay_ci" and not healthy.fault
        assert healthy.graph_healthy and healthy.graph_miss_count == 0
        assert healthy.audio_miss_count == 0
        assert wait_audio(node, status_client).accepted_audio_frames > 0
        assert call(node, capabilities, GetCapabilities.Request()).profile_name == "replay_ci"
        nodes = node.get_node_names_and_namespaces()
        for name in ("audio_capture", "mono_source", "acoustic_scene", "semantic_perception"):
            assert nodes.count((name, "/audition")) == 1, (name, nodes)
        print("Pipeline reload PASS: preflight, dry-run, owned restart, graph watchdog status")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
