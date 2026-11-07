"""Independent-frame temporal radar/audio corroboration, not identity proof."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))

from radar_bearing_matching import RadarBearingCandidate
from radar_temporal import RadarPairTemporalGate

A='a'*32
B='b'*32
C='c'*32
START=1_000_000_000
STEP=150_000_000


def candidate(*, audio='sound', radar='track', unique=True, ag=1, rg=1):
    return RadarBearingCandidate(audio,ag,radar,rg,3.,0.,0.,unique)


def step(gate, candidates, n, *, audio_session=A, radar_session=B,
         audio_stamp=None, radar_stamp=None, clock=None):
    t=START+n*STEP
    return gate.observe(
        candidates,audio_session=audio_session,radar_session=radar_session,
        audio_stamp_ns=t if audio_stamp is None else audio_stamp,
        radar_stamp_ns=t if radar_stamp is None else radar_stamp,
        clock_ns=t if clock is None else clock)


class RadarTemporalTests(unittest.TestCase):
    def setUp(self):
        self.g=RadarPairTemporalGate()
        self.c=candidate()
        self.key=self.g.key(self.c)

    def test_three_independent_pairs_only_then_replay_not_confirmed(self):
        for n in range(3):
            e=step(self.g,[self.c],n)[self.key]
            self.assertEqual(e.supporting_synchronized_observations,n+1)
            self.assertEqual(e.temporally_correlated,n==2)
        replay=step(self.g,[self.c],3,radar_stamp=START+2*STEP)[self.key]
        self.assertEqual(replay.supporting_synchronized_observations,3)
        self.assertFalse(replay.temporally_correlated)
        fresh=step(self.g,[self.c],3)[self.key]
        self.assertTrue(fresh.temporally_correlated)

    def test_one_modal_replay_does_not_count(self):
        step(self.g,[self.c],0)
        e=step(self.g,[self.c],1,audio_stamp=START)[self.key]
        self.assertEqual(e.supporting_synchronized_observations,1)
        self.assertFalse(e.temporally_correlated)
        self.assertEqual(step(self.g,[self.c],1)[self.key].
                         supporting_synchronized_observations,2)

    def test_ambiguity_invalidates_prior_progress(self):
        step(self.g,[self.c],0)
        step(self.g,[self.c],1)
        rival=candidate(radar='other',unique=False)
        result=step(self.g,[self.c,rival],2)
        self.assertEqual(result,{})
        self.assertEqual(step(self.g,[self.c],3)[self.key].
                         supporting_synchronized_observations,1)

    def test_audio_competition_invalidates_prior_progress(self):
        step(self.g,[self.c],0)
        rival=candidate(audio='other')
        self.assertEqual(step(self.g,[self.c,rival],1),{})
        self.assertEqual(step(self.g,[self.c],2)[self.key].
                         supporting_synchronized_observations,1)

    def test_explicit_clear_and_epoch_switch(self):
        step(self.g,[self.c],0)
        step(self.g,[self.c],1)
        self.assertEqual(step(self.g,[],2),{})
        self.assertEqual(step(self.g,[self.c],3)[self.key].
                         supporting_synchronized_observations,1)
        self.assertEqual(step(self.g,[self.c],4,radar_session=C)[self.key].
                         supporting_synchronized_observations,1)

    def test_generation_switch_and_time_regression(self):
        step(self.g,[self.c],0)
        step(self.g,[self.c],1)
        renewed=candidate(rg=2)
        self.assertEqual(step(self.g,[renewed],2)[self.g.key(renewed)].
                         supporting_synchronized_observations,1)
        self.assertEqual(step(self.g,[renewed],3,clock=100)[self.g.key(renewed)].
                         supporting_synchronized_observations,1)

    def test_long_gap_and_short_spacing(self):
        step(self.g,[self.c],0)
        short=step(self.g,[self.c],1,
                   audio_stamp=START+50_000_000,
                   radar_stamp=START+50_000_000)[self.key]
        self.assertEqual(short.supporting_synchronized_observations,1)
        gap=step(self.g,[self.c],10)[self.key]
        self.assertEqual(gap.supporting_synchronized_observations,1)

    def test_duplicate_keys_and_invalid_metadata_fail_closed(self):
        self.assertEqual(step(self.g,[self.c,self.c],0),{})
        self.assertEqual(step(self.g,[self.c],1,audio_session=''),{})
        self.assertEqual(step(self.g,[self.c],2,audio_stamp=0),{})
        self.assertEqual(step(self.g,[self.c],3)[self.key].
                         supporting_synchronized_observations,1)


if __name__=='__main__':
    unittest.main()
