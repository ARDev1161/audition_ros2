"""Strict AV pair matching: activity, time, geometry and ambiguity."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from av_matching import PairMatcher, Ray, StreamFence


def ray(name, gen=1, stamp=1_000_000_000, origin=(0.,0.,0.),
        direction=(1.,0.,0.), variance=.01, activity=True):
    return Ray(name,gen,stamp,origin,direction,variance,activity)


class TestStreamFence(unittest.TestCase):
    def test_epoch_replay_and_stamp_order(self):
        f=StreamFence()
        a='a'*32
        b='b'*32
        self.assertFalse(f.accept('',1,1))
        self.assertTrue(f.accept(a,1,100))
        self.assertFalse(f.accept(a,2,100))
        self.assertFalse(f.accept(a,1,101))
        self.assertFalse(f.accept(b,1,99))
        self.assertTrue(f.accept(b,1,101))
        self.assertFalse(f.accept(a,3,100))
        f.reset()
        self.assertTrue(f.accept(a,1,1))


    def test_retired_epoch_cannot_return_with_newer_timestamp(self):
        guard = StreamFence()
        a, b, c = ('a' * 32, 'b' * 32, 'c' * 32)
        self.assertTrue(guard.accept(a, 1, 100))
        self.assertTrue(guard.accept(b, 1, 200))
        self.assertFalse(guard.accept(a, 999, 300))
        self.assertEqual((guard.session, guard.sequence, guard.stamp_ns),
                         (b, 1, 200))
        self.assertTrue(guard.accept(c, 1, 301))
        self.assertFalse(guard.accept(b, 2, 302))
        guard.reset()  # Explicit clock epoch reset, not producer restart.
        self.assertTrue(guard.accept(a, 1, 1))

    def test_retirement_capacity_exhaustion_fails_closed(self):
        guard = StreamFence(max_retired=1)
        self.assertTrue(guard.accept('a' * 32, 1, 100))
        self.assertTrue(guard.accept('b' * 32, 1, 200))
        self.assertFalse(guard.accept('c' * 32, 1, 300))
        self.assertFalse(guard.accept('a' * 32, 9, 400))
        self.assertTrue(guard.accept('b' * 32, 2, 500))
        self.assertEqual(len(guard.retired), 1)

    def test_malformed_epoch_parameters_rejected_without_state_mutation(self):
        guard = StreamFence()
        self.assertTrue(guard.accept('a' * 32, 1, 100))
        for session, seq, stamp in [
                ('A' * 32, 2, 200), ('b' * 32, 0, 200),
                ('b' * 32, True, 200), ('b' * 32, 1, 0),
                ('b' * 32, 1, 2.0)]:
            self.assertFalse(guard.accept(session, seq, stamp))
        self.assertEqual((guard.session, guard.sequence, guard.stamp_ns),
                         ('a' * 32, 1, 100))


class TestPairMatcher(unittest.TestCase):
    def setUp(self):
        self.m=PairMatcher()

    def test_valid_unique_but_not_identity_proof(self):
        result=self.m.propose([ray('audio')],[ray('face')])
        self.assertEqual(len(result),1)
        self.assertTrue(result[0].unique_match)
        self.assertEqual(result[0].normalized_residual,0)
        self.assertEqual(result[0].sensor_baseline_m,0)

    def test_time_and_activity_gates(self):
        self.assertEqual(self.m.propose([ray('a')],[ray('v',stamp=1_200_000_000)]),[])
        self.assertEqual(self.m.propose([ray('a')],[ray('v',activity=False)]),[])
        self.assertEqual(self.m.propose([ray('a',activity=False)],[ray('v')]),[])

    def test_angle_and_uncertainty(self):
        self.assertFalse(self.m.propose([ray('a')],[ray('v',direction=(0.,1.,0.))]))
        self.assertFalse(self.m.propose([ray('a',variance=float('nan'))],[ray('v')]))
        self.assertFalse(self.m.propose([ray('a',variance=0.)],[ray('v')]))
        self.assertFalse(self.m.propose([ray('a',direction=(2.,0.,0.))],[ray('v')]))
        angle=.25
        self.assertEqual(len(self.m.propose(
            [ray('a')],[ray('v',direction=(math.cos(angle),math.sin(angle),0.))])),1)

    def test_parallax_baseline_and_units(self):
        self.assertFalse(self.m.propose([ray('a')],[ray('v',origin=(.5,0,0))]))
        self.assertEqual(len(self.m.propose([ray('a')],[ray('v',origin=(.1,0,0))])),1)

    def test_ambiguous_visuals_explicitly_not_unique(self):
        a=[ray('a')]
        v=[ray('alice'),ray('bob')]
        result=self.m.propose(a,v)
        self.assertEqual(len(result),2)
        self.assertFalse(any(e.unique_match for e in result))

    def test_mutual_assignment_and_margin(self):
        d0=(1.,0.,0.)
        d1=(math.cos(.24),math.sin(.24),0.)
        a=[ray('a',direction=d0),ray('b',direction=d1)]
        v=[ray('one',direction=d0),ray('two',direction=d1)]
        matches=self.m.propose(a,v)
        self.assertTrue(any(x.unique_match and x.audio.identity=='a' and x.visual.identity=='one' for x in matches))
        self.assertTrue(any(x.unique_match and x.audio.identity=='b' and x.visual.identity=='two' for x in matches))

    def test_duplicate_identity_and_bounds(self):
        self.assertFalse(self.m.propose([ray('a'),ray('a')],[ray('v')]))
        self.assertFalse(self.m.propose([ray(str(i)) for i in range(17)],[ray('v')]))
        self.assertFalse(self.m.propose([ray('a')],[ray('v'),ray('v')]))


if __name__=='__main__':
    unittest.main()
