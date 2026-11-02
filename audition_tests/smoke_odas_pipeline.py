#!/usr/bin/env python3
"""Native ODAS ROS integration smoke: distinguish processed snapshots from startup."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from audition_msgs.msg import AcousticScene, AcousticSourceArray
from audition_msgs.srv import GetIngressStatus


class Probe(Node):
    def __init__(self):
        super().__init__("audition_odas_probe")
        self.saw_internal = False
        self.saw_scene = False
        self.internal_generation = None
        self.invalid_spatial_observation = False
        self.status_client = self.create_client(GetIngressStatus,
            "/audition/spatial_perception/get_ingress_status")
        self.status_future = None
        self.create_subscription(
            AcousticSourceArray,
            "/audition/internal/spatial_sources",
            self.on_internal, 10,
        )
        self.create_subscription(
            AcousticScene, "/audition/scene", self.on_scene, 10,
        )

    def on_internal(self, msg):
        if msg.header.frame_id == "microphone_array" and msg.generation > 0:
            self.saw_internal = True
            self.internal_generation = msg.generation
            for src in msg.sources:
                vec = src.direction_unit
                magnitude = math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)
                if (
                    not src.source_id
                    or not src.spatial_track_id
                    or src.source_generation != msg.generation
                    or not src.has_direction
                    or not math.isfinite(magnitude)
                    or abs(magnitude - 1) > 0.05
                    or src.has_range
                    or src.has_position
                    or src.has_angular_variance
                ):
                    self.invalid_spatial_observation = True

    def on_scene(self, msg):
        if msg.header.frame_id == "microphone_array" and msg.generation > 0:
            self.saw_scene = True


def main():
    rclpy.init()
    probe = Probe()
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.25)
            if probe.invalid_spatial_observation:
                print('ODAS violated bearing/source contract', file=sys.stderr)
                return 1
            if probe.saw_internal and probe.saw_scene and probe.status_future is None:
                if probe.status_client.service_is_ready():
                    probe.status_future = probe.status_client.call_async(
                        GetIngressStatus.Request())
            if probe.status_future is not None and probe.status_future.done():
                result = probe.status_future.result()
                if (result is None or result.backend != "odas" or
                        result.generation < 1 or result.accepted_frames == 0 or
                        result.processed_hops == 0 or result.rejected_frames != 0):
                    print("ODAS ingress status invalid", result, file=sys.stderr)
                    return 1
                print("ODAS integration smoke PASS: native spatial hops, scene, ingress counters.")
                print("NOTE: this only verifies ROS integration, not measured DOA accuracy.")
                return 0
        print("ODAS integration smoke FAILED: no processed spatial scene.", file=sys.stderr)
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
