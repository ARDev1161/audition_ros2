#!/usr/bin/env python3
"""TrackSound admission and revocation policy without a DDS graph."""
import importlib.util
import pathlib
import threading
import unittest
from audition_msgs.msg import AttentionTarget

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'track_sound.py'
spec = importlib.util.spec_from_file_location('track_sound', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def target(stamp, source='a', generation=1, scene=1, valid=True):
    msg = AttentionTarget()
    msg.valid = valid
    msg.source_id = source
    msg.source_generation = generation
    msg.scene_generation = scene
    msg.header.frame_id = 'base_link'
    msg.header.stamp.sec = stamp
    msg.direction_unit.x = 1.0
    return msg


class Policy(unittest.TestCase):
    def setUp(self):
        class State:
            lock = threading.Lock()
            frame = 'base_link'
            latest = None
            watermark = None
            retired = None
            sequence = 0
            arrival = 0.0
        self.state = State()

    def send(self, msg):
        module.TrackSoundServer.on_target(self.state, msg)

    def test_replay_and_incarnation(self):
        self.send(target(10, generation=3))
        self.send(target(9, generation=3))
        self.send(target(100, generation=2))
        self.assertEqual(self.state.sequence, 1)
        self.send(target(11, generation=4))
        self.assertEqual(self.state.sequence, 2)
        self.send(target(1, generation=1, scene=2))
        self.assertEqual(self.state.sequence, 3)

    def test_revocation_and_recovery(self):
        self.send(target(10))
        self.send(target(11, valid=False))
        self.assertIsNone(self.state.latest)
        self.send(target(10))
        self.assertIsNone(self.state.latest)
        self.send(target(12))
        self.assertEqual(self.state.sequence, 2)

    def test_foreign_retirement_and_malformed(self):
        self.send(target(10))
        self.send(target(100, source='b', valid=False))
        self.assertIsNotNone(self.state.latest)
        invalid = target(11)
        invalid.direction_unit.x = 4.0
        self.send(invalid)
        self.assertEqual(self.state.sequence, 1)
        self.send(target(12))
        self.assertEqual(self.state.sequence, 2)

    def test_stamp_and_source_filter(self):
        self.assertFalse(module.valid_bearing(target(0), 'base_link'))
        self.assertFalse(module.admit(target(10), target(11, generation=0), 'base_link', 'a'))
        self.assertFalse(module.admit(None, target(11, source='b'), 'base_link', 'a'))
        self.assertTrue(module.admit(target(10), target(11), 'base_link', 'a'))


if __name__ == '__main__':
    unittest.main()
