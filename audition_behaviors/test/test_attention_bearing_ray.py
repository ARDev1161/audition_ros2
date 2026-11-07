#!/usr/bin/env python3
"""Deterministic policy tests for the optional RViz bearing ray."""
import importlib.util
import pathlib
import unittest
from audition_msgs.msg import AttentionTarget

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'attention_bearing_ray.py'
spec = importlib.util.spec_from_file_location('attention_bearing_ray', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def target(stamp, source='a', valid=True, generation=1, scene=1):
    msg = AttentionTarget()
    msg.valid = valid
    msg.source_id = source
    msg.source_generation = generation
    msg.scene_generation = scene
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.direction_unit.x = 1.0
    return msg


class RayPolicy(unittest.TestCase):
    def setUp(self):
        class FakePublisher:
            messages = []
            def publish(self, message):
                self.messages.append(message)
        class State:
            frame = 'base_link'
            length = 1.5
            max_age = 0.5
            watermark = None
            active = False
            last_arrival = 0.0
            publisher = FakePublisher()
            def get_clock(self):
                class Clock:
                    def now(self):
                        class Now:
                            def to_msg(self):
                                from builtin_interfaces.msg import Time
                                return Time(sec=1)
                        return Now()
                return Clock()
            marker = module.BearingRay.marker
            clear = module.BearingRay.clear
        self.state = State()

    def send(self, msg):
        module.BearingRay.on_target(self.state, msg)

    def test_ray_and_retirement(self):
        from visualization_msgs.msg import Marker
        self.send(target(10))
        self.assertEqual(self.state.publisher.messages[-1].action, Marker.ADD)
        self.assertAlmostEqual(self.state.publisher.messages[-1].points[1].x, 1.5)
        self.send(target(11, valid=False))
        self.assertEqual(self.state.publisher.messages[-1].action, Marker.DELETE)
        count = len(self.state.publisher.messages)
        self.send(target(10))
        self.assertEqual(len(self.state.publisher.messages), count)
        self.send(target(12))
        self.assertEqual(self.state.publisher.messages[-1].action, Marker.ADD)

    def test_invalid_and_source_regression(self):
        self.send(target(10, generation=3))
        bad = target(11)
        bad.direction_unit.x = float('nan')
        self.send(bad)
        self.send(target(12, generation=2))
        self.send(target(13, source='b', valid=False))
        self.assertEqual(len(self.state.publisher.messages), 1)
        self.send(target(14, generation=4))
        self.assertEqual(len(self.state.publisher.messages), 2)

    def test_expiration_once(self):
        from visualization_msgs.msg import Marker
        self.send(target(10))
        self.state.last_arrival = 0.0
        module.BearingRay.on_timer(self.state)
        self.assertEqual(self.state.publisher.messages[-1].action, Marker.DELETE)
        module.BearingRay.on_timer(self.state)
        self.assertEqual(len(self.state.publisher.messages), 2)


if __name__ == '__main__':
    unittest.main()
