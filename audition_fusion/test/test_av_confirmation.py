"""M4 AV temporally distinct corroboration and fail-closed regressions."""
import pathlib
import sys
import unittest
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from av_matching import Pair, Ray
from av_confirmation import TemporalPairGate


def pair(a=1, v=1, name="audio", visual="face", ag=1, vg=1, unique=True):
    audio = Ray(name, ag, a, (0.,0.,0.), (1.,0.,0.), .01)
    vision = Ray(visual, vg, v, (0.,0.,0.), (1.,0.,0.), .01)
    return Pair(audio, vision, 0., 0., 0., unique)


def state(gate, pairs, clock, ae="a"*32, ve="b"*32):
    return gate.observe(pairs, ae, ve, clock)


def result(gate, p, clock, ae="a"*32, ve="b"*32):
    return state(gate, [p], clock, ae, ve)[gate.key(p)]


class ConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.g = TemporalPairGate()

    def test_requires_three_independently_advanced_frames(self):
        a = 1_000_000_000
        step = 150_000_000
        p = pair(a, a)
        self.assertEqual(result(self.g, p, a).count, 1)
        self.assertFalse(result(self.g, p, a+1).confirmed)
        self.assertEqual(result(self.g, pair(a+step,a), a+step).count, 1)
        self.assertEqual(result(self.g, pair(a+step,a+step), a+step).count, 2)
        self.assertEqual(result(self.g, pair(a+2*step,a+2*step), a+2*step).count, 3)
        self.assertFalse(result(self.g, pair(a+2*step,a+2*step), a+2*step).confirmed)
        self.assertEqual(result(self.g, pair(a+2*step,a+2*step), a+2*step).count, 3)

    def test_one_modality_replay_cannot_retain_confirmation(self):
        a = 1_000_000_000
        step = 150_000_000
        for i in range(3):
            observed = result(self.g,pair(a+i*step,a+i*step),a+i*step)
        self.assertTrue(observed.confirmed)
        replay = result(self.g,pair(a+3*step,a+2*step),a+3*step)
        self.assertEqual(replay.count,3)
        self.assertFalse(replay.confirmed)
        resumed = result(self.g,pair(a+3*step,a+3*step),a+3*step)
        self.assertTrue(resumed.confirmed)

    def test_short_spacing_and_long_gap(self):
        a=1_000_000_000
        self.assertEqual(result(self.g,pair(a,a),a).count,1)
        self.assertEqual(result(self.g,pair(a+50_000_000,a+50_000_000),a+50_000_000).count,1)
        self.assertEqual(result(self.g,pair(a+150_000_000,a+150_000_000),a+150_000_000).count,2)
        self.assertEqual(result(self.g,pair(a+900_000_000,a+900_000_000),a+900_000_000).count,1)

    def test_ambiguity_and_retirement_clear(self):
        a=1_000_000_000
        result(self.g,pair(a,a),a)
        result(self.g,pair(a+150_000_000,a+150_000_000),a+150_000_000)
        self.assertEqual(state(self.g,[pair(a+300_000_000,a+300_000_000,unique=False)],a+300_000_000),{})
        self.assertEqual(result(self.g,pair(a+450_000_000,a+450_000_000),a+450_000_000).count,1)
        self.assertEqual(state(self.g,[],a+500_000_000),{})
        self.assertEqual(result(self.g,pair(a+600_000_000,a+600_000_000),a+600_000_000).count,1)

    def test_competitor_cancels_even_winner(self):
        a=1_000_000_000
        self.assertEqual(state(self.g,[pair(a,a),pair(a,a,visual="other",unique=False)],a),{})
        self.assertEqual(result(self.g,pair(a+200_000_000,a+200_000_000),a+200_000_000).count,1)

    def test_forged_unique_edges_sharing_audio_endpoint_fail_closed(self):
        a=1_000_000_000
        result(self.g,pair(a,a),a)
        conflict=[pair(a+150_000_000,a+150_000_000),
                  pair(a+150_000_000,a+150_000_000,visual="second")]
        self.assertEqual(state(self.g,conflict,a+150_000_000),{})
        self.assertEqual(result(self.g,pair(a+300_000_000,a+300_000_000),
                                a+300_000_000).count,1)

    def test_forged_unique_edges_sharing_visual_endpoint_fail_closed(self):
        a=1_000_000_000
        result(self.g,pair(a,a),a)
        conflict=[pair(a+150_000_000,a+150_000_000),
                  pair(a+150_000_000,a+150_000_000,name="second")]
        self.assertEqual(state(self.g,conflict,a+150_000_000),{})
        self.assertEqual(result(self.g,pair(a+300_000_000,a+300_000_000),
                                a+300_000_000).count,1)

    def test_epoch_generation_and_clock_jump(self):
        a=1_000_000_000
        result(self.g,pair(a,a),a)
        self.assertEqual(result(self.g,pair(a+200_000_000,a+200_000_000),a+200_000_000,ve="c"*32).count,1)
        self.assertEqual(result(self.g,pair(a+400_000_000,a+400_000_000,ag=2),a+400_000_000,ve="c"*32).count,1)
        self.assertEqual(result(self.g,pair(a+600_000_000,a+600_000_000,ag=2),a+100,ve="c"*32).count,1)

    def test_bounds_and_bogus_input(self):
        a=1_000_000_000
        self.assertEqual(state(self.g,[pair(a,a,visual=str(i)) for i in range(17)],a),{})
        self.assertEqual(result(self.g,pair(a,a),a).count,1)
        self.assertEqual(state(self.g,[pair(a,a),pair(a,a)],a+1),{})
        self.assertEqual(result(self.g,pair(a+200_000_000,a+200_000_000),a+200_000_000).count,1)


if __name__ == "__main__":
    unittest.main()
