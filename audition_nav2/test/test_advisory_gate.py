"""M5 advisory gate contract regression tests."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from advisory_gate import Advisory, AdvisoryGate

NOW = 2_000_000_000

def item(name='a', stamp=NOW-1, expires=NOW+100, confidence=.9, authorized=True):
    return Advisory(name, stamp, expires, confidence, authorized)

class AdvisoryGateTests(unittest.TestCase):
    def test_requires_explicit_authorization(self):
        gate = AdvisoryGate()
        self.assertFalse(gate.ingest(item(authorized=False), NOW))
        self.assertEqual(gate.tick(NOW), ((), ()))

    def test_silence_expires_and_revokes(self):
        gate = AdvisoryGate()
        self.assertTrue(gate.ingest(item(), NOW))
        self.assertEqual(gate.tick(NOW+100), ((), ('a',)))

    def test_replay_and_reorder_rejected(self):
        gate = AdvisoryGate()
        self.assertTrue(gate.ingest(item(), NOW))
        self.assertFalse(gate.ingest(item(), NOW))
        self.assertFalse(gate.ingest(item(stamp=NOW-2), NOW))

    def test_clock_rewind_revokes_all(self):
        gate = AdvisoryGate()
        gate.ingest(item(), NOW)
        self.assertEqual(gate.tick(NOW-1)[1], ('a',))

    def test_capacity_and_explicit_cancel(self):
        gate = AdvisoryGate(capacity=1)
        self.assertTrue(gate.ingest(item(), NOW))
        self.assertFalse(gate.ingest(item(name='b'), NOW))
        self.assertTrue(gate.revoke('a'))
        self.assertTrue(gate.ingest(item(name='b'), NOW))

    def test_nan_future_and_expired_rejected(self):
        gate = AdvisoryGate()
        self.assertFalse(gate.ingest(item(confidence=float('nan')), NOW))
        self.assertFalse(gate.ingest(item(stamp=NOW+1), NOW))
        self.assertFalse(gate.ingest(item(expires=NOW), NOW))

    def test_invalid_clock_clears(self):
        gate = AdvisoryGate()
        gate.ingest(item(), NOW)
        self.assertEqual(gate.tick(0), ((), ('a',)))

if __name__ == '__main__':
    unittest.main()
