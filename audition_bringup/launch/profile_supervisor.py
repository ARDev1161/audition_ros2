#!/usr/bin/env python3
"""ROS 2 supervised profile switching with graph and capture PCM watchdogs."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AudioFrame, SourceAudioFrame, SemanticProgress
from audition_msgs.srv import ReloadPipeline, GetPipelineStatus
from profile_config import resolve_profile, validate_namespace, ProfileError
from profile_supervisor_core import PipelineSupervisor, ProfileSpec
from profile_audio_flow import AudioFlowMonitor
from profile_source_flow import SourceFlowMonitor
from profile_semantic_flow import SemanticFlowMonitor


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

    def _group_exists(self):
        try:
            os.killpg(self.child.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def stop(self):
        # The launch parent may exit while its ROS workers survive.
        try:
            if self._group_exists():
                os.killpg(self.child.pid, signal.SIGINT)
                deadline = time.monotonic() + 8
                while self._group_exists() and time.monotonic() < deadline:
                    self.child.poll()  # reap exited group leader promptly
                    time.sleep(0.1)
            if self._group_exists():
                os.killpg(self.child.pid, signal.SIGKILL)
                deadline = time.monotonic() + 5
                while self._group_exists() and time.monotonic() < deadline:
                    self.child.poll()  # reap exited group leader promptly
                    time.sleep(0.1)
            if self._group_exists():
                raise RuntimeError("owned process group survived SIGKILL")
            self.child.wait(timeout=2)
        finally:
            self.log.close()


class GraphProbe:
    def __init__(self, node, validate):
        self.node, self.validate = node, validate
        self._ready_expected = None
        self._ready_service = None

    @staticmethod
    def expected(spec, resolved):
        spatial = "mono_source" if resolved.spatial_backend == "mono" else "spatial_perception"
        return {("audio_capture", "/" + spec.namespace),
                (spatial, "/" + spec.namespace),
                ("acoustic_scene", "/" + spec.namespace),
                ("semantic_perception", "/" + spec.namespace)}

    def _nodes(self):
        return self.node.get_node_names_and_namespaces()

    def vacant(self, spec):
        return not (self.expected(spec, self.validate(spec)) & set(self._nodes()))

    def ready(self, spec, process, timeout):
        expected = self.expected(spec, self.validate(spec))
        service = "/" + spec.namespace + "/semantic_perception/get_capabilities"
        end = time.monotonic() + timeout
        last_nodes, last_services = [], set()
        while time.monotonic() < end:
            if process.poll() is not None:
                self.node.get_logger().error(
                    "owned launch exited before readiness (code=%s, log=%s)" %
                    (process.poll(), process.log.name))
                return False
            services = {name for name, _ in self.node.get_service_names_and_types()}
            nodes = self._nodes()
            last_nodes, last_services = nodes, services
            if all(nodes.count(item) == 1 for item in expected) and service in services:
                self._ready_expected = expected
                self._ready_service = service
                return True
            time.sleep(0.1)
        missing_nodes = sorted(item for item in expected if last_nodes.count(item) != 1)
        self.node.get_logger().error(
            "pipeline readiness timeout: missing/duplicate nodes=%s, "
            "capability_service_present=%s, owned_log=%s" %
            (missing_nodes, service in last_services, process.log.name))
        return False

    def health(self, spec):
        """Read-only graph liveness, using the last successfully started layout."""
        if self._ready_expected is None or self._ready_service is None:
            return False, "no confirmed ready graph"
        nodes = self._nodes()
        missing = [name for name, namespace in sorted(self._ready_expected)
                   if nodes.count((name, namespace)) != 1]
        services = {name for name, _ in self.node.get_service_names_and_types()}
        if self._ready_service not in services:
            missing.append("semantic_perception/get_capabilities")
        return (not missing, "missing/duplicate: " + ", ".join(missing) if missing else "")

    def gone(self, spec, timeout):
        expected = self.expected(spec, self.validate(spec))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if not (expected & set(self._nodes())):
                return True
            time.sleep(0.1)
        return False


class SupervisorNode(Node):
    def __init__(self, initial, timeout):
        super().__init__("pipeline_supervisor", namespace=initial.namespace)
        self.namespace_name = initial.namespace
        # Source tree: launch/..; installed: share/audition_bringup/launch/..
        self.share = Path(__file__).resolve().parents[1]
        self.audio_flow = AudioFlowMonitor(
            lambda spec: self.validate(spec).capture["kind"])
        def source_mode(spec):
            resolved = self.validate(spec)
            return resolved.capture["kind"], resolved.spatial_backend
        self.source_flow = SourceFlowMonitor(source_mode)
        self.semantic_flow = SemanticFlowMonitor(source_mode)
        self.supervisor = PipelineSupervisor(
            self.validate, OwnedLaunch, GraphProbe(self, self.validate),
            timeout, audio_flow=self.audio_flow, source_flow=self.source_flow,
            semantic_flow=self.semantic_flow)
        # Subscribe before starting the owned graph. The flow monitor ignores
        # callbacks outside an active incarnation and fences old session IDs.
        self.audio_subscription = self.create_subscription(
            AudioFrame, "audio/raw", self.on_audio, qos_profile_sensor_data)
        self.source_subscription = self.create_subscription(
            SourceAudioFrame, "internal/source_audio", self.on_source,
            qos_profile_sensor_data)
        self.semantic_subscription = self.create_subscription(
            SemanticProgress, "internal/semantic_progress", self.on_semantic,
            qos_profile_sensor_data)
        self.supervisor.start(initial)
        self.service = self.create_service(
            ReloadPipeline, "pipeline/reload", self.on_reload)
        self.status_service = self.create_service(
            GetPipelineStatus, "pipeline/get_status", self.on_status)
        self.monitor_timer = self.create_timer(1.0, self.on_monitor)

    def validate(self, spec):
        validate_namespace(spec.namespace)
        if spec.namespace != self.namespace_name:
            raise ProfileError("namespace changes are not supported")
        resolved = resolve_profile(spec.profile, self.share, spec.overrides())
        if spec.namespace != "audition" and resolved.capture["frame_id"] == "microphone_array":
            raise ProfileError("non-default namespace requires a unique frame_id")
        return resolved

    def on_audio(self, msg):
        self.audio_flow.observe(msg.capture_session_id, msg.sequence_number,
                                msg.frame_count, msg.channel_count, len(msg.pcm))

    def on_source(self, msg):
        self.source_flow.observe(
            msg.spatial_session_id, msg.source_id, msg.source_generation,
            msg.sequence_number, msg.sample_rate_hz, msg.frame_count,
            len(msg.pcm))

    def on_semantic(self, msg):
        self.semantic_flow.observe(
            msg.spatial_session_id, msg.source_id, msg.source_generation,
            msg.sequence_number, msg.frame_count)

    def on_monitor(self):
        previous = self.supervisor.status().running
        self.supervisor.monitor()
        status = self.supervisor.status()
        if previous and not status.running:
            self.get_logger().error(status.fault)

    def on_status(self, request, response):
        self.supervisor.monitor()
        status = self.supervisor.status()
        response.running = status.running
        response.owned_process_present = status.owned_process_present
        response.active_profile = status.active_profile
        response.last_good_profile = status.last_good_profile
        response.reload_epoch = status.reload_epoch
        response.fault = status.fault
        response.graph_healthy = status.graph_healthy
        response.graph_miss_count = status.graph_miss_count
        response.audio_watchdog_enabled = status.audio_watchdog_enabled
        response.audio_healthy = status.audio_healthy
        response.audio_miss_count = status.audio_miss_count
        response.accepted_audio_frames = status.accepted_audio_frames
        response.last_audio_age_ms = status.last_audio_age_ms
        response.source_watchdog_enabled = status.source_watchdog_enabled
        response.source_healthy = status.source_healthy
        response.source_miss_count = status.source_miss_count
        response.accepted_source_frames = status.accepted_source_frames
        response.last_source_age_ms = status.last_source_age_ms
        response.semantic_watchdog_enabled = status.semantic_watchdog_enabled
        response.semantic_healthy = status.semantic_healthy
        response.semantic_miss_count = status.semantic_miss_count
        response.accepted_semantic_frames = status.accepted_semantic_frames
        response.last_semantic_age_ms = status.last_semantic_age_ms
        return response

    def on_reload(self, request, response):
        self.supervisor.monitor()
        old = self.supervisor.active or self.supervisor.last_good
        if old is None:
            response.success = False
            response.message = "no previously healthy supervised profile"
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
