#!/usr/bin/env python3
"""ROS 2 M2o service for supervised stop/start profile switching and failback."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from audition_msgs.srv import ReloadPipeline
from profile_config import resolve_profile, validate_namespace, ProfileError
from profile_supervisor_core import PipelineSupervisor, ProfileSpec


class OwnedLaunch:
    def __init__(self, spec):
        self.log = tempfile.NamedTemporaryFile(
            mode="w", prefix="audition-profile-", suffix=".log", delete=False)
        command = ["ros2", "launch", "audition_bringup", "m2_profile.launch.py",
                   "profile:=" + spec.profile,
                   "audition_namespace:=" + spec.namespace]
        command.extend(k + ":=" + v for k, v in spec.overrides().items())
        try:
            self.child = subprocess.Popen(
                command, stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception:
            self.log.close()
            raise

    def poll(self):
        return self.child.poll()

    def stop(self):
        try:
            if self.child.poll() is None:
                os.killpg(self.child.pid, signal.SIGINT)
                try:
                    self.child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    self.child.wait(timeout=5)
        finally:
            self.log.close()


class GraphProbe:
    def __init__(self, node, validate):
        self.node, self.validate = node, validate

    @staticmethod
    def expected(spec, resolved):
        spatial = "mono_source" if resolved.spatial_backend == "mono" else "spatial_perception"
        return {("audio_capture", "/" + spec.namespace),
                (spatial, "/" + spec.namespace),
                ("acoustic_scene", "/" + spec.namespace),
                ("semantic_perception", "/" + spec.namespace)}

    def _nodes(self):
        return set(self.node.get_node_names_and_namespaces())

    def ready(self, spec, process, timeout):
        expected = self.expected(spec, self.validate(spec))
        service = "/" + spec.namespace + "/semantic_perception/get_capabilities"
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                return False
            services = {name for name, _ in self.node.get_service_names_and_types()}
            if expected.issubset(self._nodes()) and service in services:
                return True
            time.sleep(0.1)
        return False

    def gone(self, spec, timeout):
        expected = self.expected(spec, self.validate(spec))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if not (expected & self._nodes()):
                return True
            time.sleep(0.1)
        return False


class SupervisorNode(Node):
    def __init__(self, initial, timeout):
        super().__init__("pipeline_supervisor", namespace=initial.namespace)
        self.namespace_name = initial.namespace
        # Source tree: launch/..; installed: share/audition_bringup/launch/..
        self.share = Path(__file__).resolve().parents[1]
        self.supervisor = PipelineSupervisor(
            self.validate, OwnedLaunch, GraphProbe(self, self.validate), timeout)
        self.supervisor.start(initial)
        self.service = self.create_service(
            ReloadPipeline, "pipeline/reload", self.on_reload)

    def validate(self, spec):
        validate_namespace(spec.namespace)
        if spec.namespace != self.namespace_name:
            raise ProfileError("namespace changes are not supported")
        resolved = resolve_profile(spec.profile, self.share, spec.overrides())
        if spec.namespace != "audition" and resolved.capture["frame_id"] == "microphone_array":
            raise ProfileError("non-default namespace requires a unique frame_id")
        return resolved

    def on_reload(self, request, response):
        old = self.supervisor.active
        if old is None:
            response.success = False
            response.message = "no active supervised pipeline"
            return response
        profile = request.profile or old.profile
        same = profile == old.profile
        spec = ProfileSpec(profile, self.namespace_name,
            request.wav_path or (old.wav_path if same else ""),
            request.device or (old.device if same else ""),
            request.models_dir or (old.models_dir if same else ""),
            request.frame_id or (old.frame_id if same else ""))
        result = self.supervisor.switch(spec, request.dry_run)
        response.success = result.success
        response.rollback_succeeded = result.rollback_succeeded
        response.reload_epoch = result.reload_epoch
        response.active_profile = result.active_profile
        response.message = result.message
        if not result.success:
            self.get_logger().error(result.message)
        return response

    def close(self):
        self.supervisor.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--namespace", default="audition")
    parser.add_argument("--wav-path", default="")
    parser.add_argument("--device", default="")
    parser.add_argument("--models-dir", default="")
    parser.add_argument("--frame-id", default="")
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    initial = ProfileSpec(args.profile, args.namespace, args.wav_path,
                          args.device, args.models_dir, args.frame_id)
    rclpy.init()
    node = None
    try:
        node = SupervisorNode(initial, args.timeout)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
