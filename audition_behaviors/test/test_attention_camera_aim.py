#!/usr/bin/env python3
"""Camera aim proposal geometry and fail-closed admission."""
import importlib.util
import pathlib
import unittest
from audition_msgs.msg import AttentionTarget

p = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'attention_camera_aim.py'
spec = importlib.util.spec_from_file_location('attention_camera_aim', p)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def target(stamp, x=1.0, y=0.0, z=0.0, valid=True, source='a', generation=1, scene=1):
    msg = AttentionTarget()
    msg.valid = valid
    msg.source_id = source
    msg.source_generation = generation
    msg.scene_generation = scene
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.direction_unit.x = float(x)
    msg.direction_unit.y = float(y)
    msg.direction_unit.z = float(z)
    return msg


class AimTests(unittest.TestCase):
    def setUp(self):
        class Publisher:
            def __init__(self):
                self.messages = []
            def publish(self, m):
                self.messages.append(m)
        class State:
            frame = 'base_link'
            max_pan = 1.2
            max_tilt = 0.7
            max_age = 0.5
            watermark = None
            active = False
            arrival = 0
            publisher = Publisher()
            emit = mod.CameraAim.emit
            revoke = mod.CameraAim.revoke
            def get_clock(self):
                class Clock:
                    def now(self):
                        class Stamp:
                            def to_msg(self):
                                from builtin_interfaces.msg import Time
                                return Time(sec=1)
                        return Stamp()
                return Clock()
        self.state = State()

    def send(self, msg):
        mod.CameraAim.on_target(self.state, msg)

    def test_axes_and_limits(self):
        self.assertEqual(mod.angles(target(1), 'base_link', 1.2, 0.7), (0, 0))
        self.assertAlmostEqual(mod.angles(target(1, 0.8, 0.6), 'base_link', 1.2, 0.7)[0], 0.64350110879)
        self.assertAlmostEqual(mod.angles(target(1, 0.8, 0, 0.6), 'base_link', 1.2, 0.7)[1], 0.64350110879)
        self.assertIsNone(mod.angles(target(1, -1), 'base_link', 1.2, 0.7))
        self.assertIsNone(mod.angles(target(1, 0.1, 0.994987437), 'base_link', 1.2, 0.7))
        self.assertIsNone(mod.angles(target(1, float('nan')), 'base_link', 1.2, 0.7))

    def test_replay_retirement_and_recovery(self):
        self.send(target(10))
        self.assertTrue(self.state.publisher.messages[-1].valid)
        self.send(target(9))
        self.assertEqual(len(self.state.publisher.messages), 1)
        self.send(target(11, valid=False))
        self.assertFalse(self.state.publisher.messages[-1].valid)
        self.send(target(10))
        self.assertEqual(len(self.state.publisher.messages), 2)
        self.send(target(12))
        self.assertTrue(self.state.publisher.messages[-1].valid)

    def test_malformed_cannot_poison_watermark(self):
        self.send(target(10))
        self.send(target(999, x=float('nan')))
        self.assertFalse(self.state.publisher.messages[-1].valid)
        self.send(target(11))
        self.assertTrue(self.state.publisher.messages[-1].valid)

    def test_bounded_source_id_and_recovery(self):
        self.send(target(10, source='a' * 257))
        self.assertFalse(self.state.active)
        self.assertIsNone(self.state.watermark)
        self.send(target(11, source='a' * 256))
        self.assertTrue(self.state.active)
        self.assertEqual(self.state.publisher.messages[-1].source_id, 'a' * 256)

    def test_foreign_retirement_cannot_revoke_active(self):
        self.send(target(10, source='a'))
        self.send(target(11, valid=False, source='b'))
        self.assertTrue(self.state.active)
        self.assertEqual(len(self.state.publisher.messages), 1)

    def test_expiry_once(self):
        self.send(target(10))
        self.state.arrival = 0
        mod.CameraAim.on_timer(self.state)
        self.assertEqual(self.state.publisher.messages[-1].reason, 'bearing_expired')
        mod.CameraAim.on_timer(self.state)
        self.assertEqual(len(self.state.publisher.messages), 2)


if __name__ == '__main__':
    unittest.main()
