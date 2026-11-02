#!/usr/bin/env python3
"""Real ROS graph: transactional semantic worker restart and re-admission."""
import time
import rclpy
from audition_msgs.srv import ReloadSemanticWorkers, GetSemanticStatus


def call(node, client, request):
    assert client.wait_for_service(timeout_sec=10), client.srv_name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    assert future.done() and future.exception() is None
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node("semantic_reload_ci")
    try:
        status = node.create_client(GetSemanticStatus,
            "/audition/semantic_perception/get_semantic_status")
        reload = node.create_client(ReloadSemanticWorkers,
            "/audition/semantic_perception/reload_workers")
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            before = call(node, status, GetSemanticStatus.Request())
            if before.have_stream and before.admitted_sources and before.accepted_audio_frames:
                break
            time.sleep(0.1)
        else:
            raise AssertionError("No admitted stream before reload")
        dry = ReloadSemanticWorkers.Request()
        dry.dry_run = True
        checked = call(node, reload, dry)
        assert checked.success and checked.reload_epoch == before.reload_epoch, checked
        normal = ReloadSemanticWorkers.Request()
        normal.dry_run = False
        result = call(node, reload, normal)
        assert result.success and result.reload_epoch == before.reload_epoch + 1, result
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            after = call(node, status, GetSemanticStatus.Request())
            if (after.reload_epoch == result.reload_epoch and
                    after.reload_successes >= 1 and not after.reload_pending and
                    after.admitted_sources > 0 and
                    after.accepted_audio_frames > before.accepted_audio_frames):
                break
            time.sleep(0.1)
        else:
            raise AssertionError("No semantic re-admission after reload: " + str(after))
        assert after.source_generation == before.source_generation
        print("Semantic reload graph PASS: dry run, commit, fence, re-admission")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
