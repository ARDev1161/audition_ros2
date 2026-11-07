"""M4 scene incarnation and replay admission regressions."""
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from scene_admission import SceneAdmission

A = 'a' * 32
B = 'b' * 32


class SceneAdmissionTests(unittest.TestCase):
    def test_monotonic_scene_and_replay_rejection(self):
        gate = SceneAdmission()
        self.assertTrue(gate.begin(A, 1, 100).accepted)
        self.assertFalse(gate.begin(A, 1, 200).accepted)
        self.assertFalse(gate.begin(A, 2, 100).accepted)
        self.assertTrue(gate.begin(A, 2, 200).accepted)
        self.assertFalse(gate.begin(A, 1, 150).accepted)

    def test_new_incarnation_requires_clearing_evidence(self):
        gate = SceneAdmission()
        self.assertTrue(gate.begin(A, 1, 100).accepted)
        switched = gate.begin(B, 1, 200)
        self.assertTrue(switched.accepted)
        self.assertTrue(switched.restarted)
        self.assertFalse(gate.begin(A, 3, 300).accepted)
        self.assertTrue(gate.begin(B, 2, 400).accepted)

    def test_invalid_identity_and_order(self):
        gate = SceneAdmission()
        for session in (None, '', 'invalid', 'A' * 32):
            self.assertFalse(gate.begin(session, 1, 100).accepted)
        for sequence, stamp in ((0, 100), (1, 0), (1, -10),
                                (True, 200), (1.0, 200)):
            self.assertFalse(gate.begin(A, sequence, stamp).accepted)

    def test_reset_keeps_retired_session_fence(self):
        gate = SceneAdmission()
        gate.begin(A, 1, 100)
        gate.reset()
        self.assertFalse(gate.begin(A, 2, 200).accepted)
        self.assertTrue(gate.begin(B, 1, 300).accepted)

    def test_bounded_retirement(self):
        gate = SceneAdmission(retired_capacity=1)
        gate.begin(A, 1, 100)
        gate.begin(B, 1, 200)
        self.assertFalse(gate.begin(A, 2, 300).accepted)
        gate.reset()
        # Old generations outside the fixed replay memory cannot be fenced
        # indefinitely; the producer must guarantee fresh session UUIDs.
        self.assertTrue(gate.begin(A, 1, 400).accepted)

    def test_invalid_capacity(self):
        for n in (0, 129, 1.5):
            with self.assertRaises(ValueError):
                SceneAdmission(retired_capacity=n)


if __name__ == '__main__':
    unittest.main()
