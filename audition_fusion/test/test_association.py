"""Tests for incarnation, replay, retirement and three-view consistency."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from association import CandidateTracker, Observation

SESSION = '1'*32
SESSION2 = '2'*32


def obs(t, origin, target=(3., 2.), name='src', gen=1):
    dx,dy = target[0]-origin[0], target[1]-origin[1]
    r = math.hypot(dx,dy)
    return Observation(name,gen,t*1_000_000_000, origin, (dx/r,dy/r))


class AssociationTests(unittest.TestCase):
    def setUp(self):
        self.t = CandidateTracker()

    def begin(self, seq, t, session=SESSION):
        return self.t.begin_scene(session, seq, t*1_000_000_000)

    def test_three_translated_views_produce_candidate(self):
        for seq,origin in enumerate([(0.,0.), (1.,0.), (2.,0.)],start=1):
            self.assertTrue(self.begin(seq,seq))
            candidate = self.t.observe(obs(seq,origin))
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate.x,3)
        self.assertAlmostEqual(candidate.y,2)
        self.assertEqual(candidate.observations,3)

    def test_same_position_or_parallel_cannot_generate_position(self):
        for seq in range(1,4):
            self.begin(seq,seq)
            self.assertIsNone(self.t.observe(obs(seq,(0.,0.))))
        self.t = CandidateTracker()
        for seq,x in enumerate([0.,1.,2.],start=1):
            self.begin(seq,seq)
            self.assertIsNone(self.t.observe(
                Observation('src',1,seq*1_000_000_000,(x,0.),(1.,0.))))

    def test_new_session_clears_history(self):
        for seq,x in enumerate([0.,1.],start=1):
            self.begin(seq,seq)
            self.t.observe(obs(seq,(x,0.)))
        self.begin(1,3,SESSION2)
        self.assertIsNone(self.t.observe(obs(3,(2.,0.))))
        self.assertEqual(self.t.session,SESSION2)

    def test_generation_change_and_retirement(self):
        for seq,x in enumerate([0.,1.],start=1):
            self.begin(seq,seq)
            self.t.observe(obs(seq,(x,0.)))
        self.begin(3,3)
        self.assertIsNone(self.t.observe(obs(3,(2.,0.),gen=2)))
        self.assertEqual(list(self.t.tracks),[('src',2)])
        self.t.retire_missing(set())
        self.assertFalse(self.t.tracks)

    def test_scene_and_track_replay(self):
        self.assertFalse(self.begin(0,1))
        self.assertTrue(self.begin(1,1))
        self.assertFalse(self.begin(1,2))
        self.assertFalse(self.begin(2,1))
        self.assertTrue(self.begin(2,2))
        self.assertIsNone(self.t.observe(obs(2,(0,0))))
        self.assertIsNone(self.t.observe(obs(1,(1,0))))
        self.assertEqual(len(self.t.tracks[('src',1)]),1)

    def test_inconsistent_third_ray_rejected(self):
        for seq,x in enumerate([0.,1.],start=1):
            self.begin(seq,seq)
            self.t.observe(obs(seq,(x,0.)))
        self.begin(3,3)
        self.assertIsNone(self.t.observe(obs(3,(2.,0.),target=(3.,-3.))))

    def test_bounded_tracks_and_bad_data(self):
        self.t = CandidateTracker(max_tracks=2,max_observations=3)
        for seq, name in enumerate(['a','b','c'],start=1):
            self.begin(seq,seq)
            self.t.observe(obs(seq,(0.,0.),name=name))
        self.assertEqual(len(self.t.tracks),2)
        self.begin(4,4)
        self.assertIsNone(self.t.observe(Observation('bad',1,4_000_000_000,(0.,0.),(float('nan'),0.))))
        self.assertEqual(len(self.t.tracks),2)

    def test_expiry(self):
        self.begin(1,1)
        self.t.observe(obs(1,(0.,0.)))
        self.t.expire(7_000_000_001)
        self.assertFalse(self.t.tracks)


if __name__ == '__main__':
    unittest.main()
