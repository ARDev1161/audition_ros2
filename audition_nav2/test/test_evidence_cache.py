"""ROS-independent M5 bounded cache and expiration contracts."""
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nav_admission import MetricSource
from evidence_cache import EvidenceCache

T = 2_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)


def source(name='s1', stamp=T-100, expiry=T+100):
    return MetricSource(name, 'map', stamp, expiry, (2., 3., 0.), COV, True)


class EvidenceCacheTests(unittest.TestCase):
    def test_silence_expires_without_new_messages(self):
        cache = EvidenceCache()
        self.assertTrue(cache.ingest(source(), T))
        self.assertEqual(len(cache.tick(T+99)), 1)
        self.assertEqual(cache.tick(T+100), ())

    def test_replay_and_older_update_do_not_replace(self):
        cache = EvidenceCache()
        self.assertTrue(cache.ingest(source(), T))
        self.assertFalse(cache.ingest(source(), T))
        self.assertFalse(cache.ingest(source(stamp=T-101), T))
        self.assertEqual(cache.tick(T)[0].stamp_ns, T-100)

    def test_capacity_and_revoke(self):
        cache = EvidenceCache(capacity=1)
        self.assertTrue(cache.ingest(source(), T))
        self.assertFalse(cache.ingest(source('s2'), T))
        cache.revoke('s1')
        self.assertTrue(cache.ingest(source('s2'), T))

    def test_clock_rewind_clears(self):
        cache = EvidenceCache()
        self.assertTrue(cache.ingest(source(), T))
        self.assertEqual(cache.tick(T-1), ())
        self.assertEqual(cache.tick(T), ())

    def test_unvalidated_never_enters_cache(self):
        cache = EvidenceCache()
        self.assertFalse(cache.ingest(replace(source(), physically_validated=False), T))
        self.assertFalse(cache.ingest(source(stamp=T-3_000_000_000), T))
        self.assertEqual(cache.tick(T), ())

    def test_invalid_time_fails_closed(self):
        cache = EvidenceCache()
        self.assertTrue(cache.ingest(source(), T))
        self.assertEqual(cache.tick(0), ())
        self.assertEqual(cache.tick(T), ())


if __name__ == '__main__':
    unittest.main()
