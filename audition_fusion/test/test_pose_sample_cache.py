"""M4 bounded pose matching regressions."""
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pose_sample_cache import PoseSampleCache


def pose(stamp):
    return NS(stamp_ns=stamp, frame_id="odom", origin=NS(x_m=0.0))


class PoseCacheTests(unittest.TestCase):
    def test_nearest_and_skew(self):
        c = PoseSampleCache(maximum_skew_ns=30)
        self.assertTrue(c.add(pose(100), 150))
        self.assertTrue(c.add(pose(200), 250))
        self.assertEqual(c.match(190, 260).sample.stamp_ns, 200)
        self.assertIsNone(c.match(160, 260))

    def test_replay_and_bounded_memory(self):
        c = PoseSampleCache(capacity=2)
        for stamp in (100, 200, 300):
            self.assertTrue(c.add(pose(stamp), stamp))
        self.assertEqual([s.stamp_ns for s in c.samples], [200, 300])
        self.assertFalse(c.add(pose(300), 300))
        self.assertFalse(c.add(pose(250), 310))

    def test_ttl_and_clock_rollback(self):
        c = PoseSampleCache(maximum_age_ns=100, maximum_skew_ns=30)
        self.assertTrue(c.add(pose(100), 110))
        self.assertIsNone(c.match(100, 250))
        self.assertFalse(c.samples)
        self.assertTrue(c.add(pose(260), 270))
        self.assertIsNone(c.match(260, 200))
        self.assertFalse(c.samples)
        self.assertTrue(c.add(pose(180), 200))

    def test_future_and_missing_pose_rejected(self):
        c = PoseSampleCache()
        self.assertFalse(c.add(pose(200), 100))
        self.assertFalse(c.add(NS(stamp_ns=50, frame_id="odom", origin=None), 100))
        self.assertFalse(c.add(NS(stamp_ns=50, frame_id="", origin=object()), 100))
        self.assertIsNone(c.match(200, 100))

    def test_invalid_settings(self):
        for kw in ({"capacity": 0}, {"capacity": 300},
                   {"maximum_skew_ns": -1},
                   {"maximum_skew_ns": 100, "maximum_age_ns": 10}):
            with self.assertRaises(ValueError):
                PoseSampleCache(**kw)


if __name__ == "__main__":
    unittest.main()
