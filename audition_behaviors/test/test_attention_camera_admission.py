#!/usr/bin/env python3
"""Fail-closed camera admission policy regression tests."""
import importlib.util
import pathlib
import unittest
from audition_msgs.msg import CameraAimProposal

path = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'attention_camera_admission.py'
spec = importlib.util.spec_from_file_location('camera_admission', path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def proposal(stamp=1, source='s', pan=0.1, valid=True):
    m = CameraAimProposal()
    m.header.frame_id = 'base_link'
    m.header.stamp.sec = stamp
    m.scene_generation = 1
    m.source_generation = 1
    m.source_id = source
    m.pan_rad = pan
    m.tilt_rad = 0.1
    m.valid = valid
    return m


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.p = mod.Admission('base_link', 1.2, 0.7, 0.3)

    def test_default_denied_and_heartbeat_expiry(self):
        self.assertFalse(self.p.receive(proposal(), 1.0))
        self.p.authorize(True, 1.0)
        self.assertTrue(self.p.receive(proposal(), 1.1))
        self.assertTrue(self.p.expired(1.31))
        self.assertFalse(self.p.active)
        self.assertFalse(self.p.receive(proposal(2), 1.4))

    def test_revocation_and_reauthorization(self):
        self.p.authorize(True, 1)
        self.assertTrue(self.p.receive(proposal(), 1.1))
        self.p.authorize(False, 1.2)
        self.assertFalse(self.p.active)
        self.p.authorize(True, 1.3)
        self.assertTrue(self.p.receive(proposal(2), 1.4))

    def test_replay_and_retirement(self):
        self.p.authorize(True, 1)
        self.assertTrue(self.p.receive(proposal(10), 1.1))
        self.assertFalse(self.p.receive(proposal(10), 1.12))
        self.assertFalse(self.p.receive(proposal(11, valid=False), 1.14))
        self.assertFalse(self.p.active)
        self.assertFalse(self.p.receive(proposal(10), 1.15))
        self.assertTrue(self.p.receive(proposal(12), 1.16))

    def test_invalid_cannot_poison_watermark(self):
        self.p.authorize(True, 1)
        self.assertFalse(self.p.receive(proposal(999, pan=float('nan')), 1.1))
        self.assertTrue(self.p.receive(proposal(2), 1.2))
        self.assertFalse(self.p.receive(proposal(1000, source='é'*129), 1.21))
        self.assertTrue(self.p.receive(proposal(3), 1.22))

    def test_limits_and_frame(self):
        self.p.authorize(True, 1)
        self.assertFalse(self.p.receive(proposal(1, pan=1.3), 1.1))
        m = proposal(2)
        m.header.frame_id = 'map'
        self.assertFalse(self.p.receive(m, 1.2))
        self.assertTrue(self.p.receive(proposal(3), 1.21))


if __name__ == '__main__':
    unittest.main()
