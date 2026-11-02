#!/usr/bin/env python3
"""Concurrent namespaced ROS graphs: no cross-robot PCM, scene or service leaks."""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audition_msgs.msg import AcousticScene, SourceAudioFrame
from audition_msgs.srv import GetCapabilities


class Probe(Node):
    def __init__(self):
        super().__init__("namespace_isolation_ci")
        self.frames = {"robot_a": "robot_a_mic", "robot_b": "robot_b_mic"}
        self.audio = set()
        self.scenes = set()
        self.errors = []
        self.clients = {}
        for robot in self.frames:
            ns = "/" + robot + "/audition"
            self.create_subscription(SourceAudioFrame, ns + "/internal/source_audio",
                lambda msg, robot=robot: self.on_audio(robot, msg), qos_profile_sensor_data)
            self.create_subscription(AcousticScene, ns + "/scene",
                lambda msg, robot=robot: self.on_scene(robot, msg), 10)
            self.clients[robot] = self.create_client(
                GetCapabilities, ns + "/semantic_perception/get_capabilities")

    def on_audio(self, robot, msg):
        if msg.header.frame_id != self.frames[robot]:
            self.errors.append(robot + " cross-namespace PCM frame")
            return
        if msg.sample_rate_hz != 16000 or msg.frame_count != len(msg.pcm):
            self.errors.append(robot + " invalid PCM")
            return
        if len(msg.pcm) < 160 or max(abs(v) for v in msg.pcm) < 0.1:
            return
        samples = msg.pcm[:160]
        crossings = sum((a < 0) != (b < 0) for a, b in zip(samples, samples[1:]))
        low, high = (6, 11) if robot == "robot_a" else (15, 21)
        if low <= crossings <= high:
            self.audio.add(robot)
        else:
            self.errors.append(robot + " unexpected audio frequency: " + str(crossings))

    def on_scene(self, robot, msg):
        if msg.header.frame_id != self.frames[robot]:
            self.errors.append(robot + " cross-namespace scene frame")
        if msg.sources and all(s.source_generation == msg.generation for s in msg.sources):
            self.scenes.add(robot)


def main():
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.errors:
                break
            if len(node.audio) == len(node.scenes) == 2 and all(
                    c.service_is_ready() for c in node.clients.values()):
                break
        assert not node.errors, node.errors
        assert len(node.audio) == len(node.scenes) == 2, (
            node.audio, node.scenes, node.get_node_names_and_namespaces())
        for robot, client in node.clients.items():
            assert client.wait_for_service(timeout_sec=5.0), robot
            future = client.call_async(GetCapabilities.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            assert future.done() and future.exception() is None, robot
            caps = future.result()
            assert caps.profile_name == "replay_ci" and caps.spatial_backend == "mono"
            assert not caps.classifier_active and not caps.speech_active
        topics = {name for name, _ in node.get_topic_names_and_types()}
        assert "/audition/scene" not in topics
        assert "/audition/internal/source_audio" not in topics
        print("PASS: two isolated namespaces, distinct PCM, scenes and capability services")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, RuntimeError) as error:
        print("Namespace isolation failed:", error, file=sys.stderr)
        sys.exit(1)
