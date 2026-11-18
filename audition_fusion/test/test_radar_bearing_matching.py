"""M4 radar/audio diagnostic candidate gates; no metric localization output."""
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from av_matching import Ray
from radar_bearing_matching import RadarPoint, RadarBearingMatcher

STAMP=1_000_000_000
COV=(.04,0.,0.,0.,.04,0.,0.,0.,.04)


def audio(name='sound', *, stamp=STAMP, origin=(0.,0.,0.),
          direction=(1.,0.,0.), variance=.01):
    return Ray(name,1,stamp,origin,direction,variance)


def radar(name='radar', *, stamp=STAMP, xyz=(3.,0.,0.),cov=COV):
    return RadarPoint(name,1,stamp,xyz,cov)


class RadarBearingTests(unittest.TestCase):
    def setUp(self):
        self.m=RadarBearingMatcher()

    def test_unique_diagnostic_proposal_not_identity(self):
        items=self.m.propose([audio()],[radar()])
        self.assertEqual(len(items),1)
        self.assertTrue(items[0].unique_match)
        self.assertEqual(items[0].range_along_ray_m,3.)
        self.assertEqual(items[0].lateral_residual_m,0.)

    def test_off_ray_behind_outside_range_and_time_skew_abstain(self):
        self.assertEqual(self.m.propose([audio()],[radar(xyz=(0.,3.,0.))]),[])
        self.assertEqual(self.m.propose([audio()],[radar(xyz=(-3.,0.,0.))]),[])
        self.assertEqual(self.m.propose([audio()],[radar(xyz=(50.,0.,0.))]),[])
        self.assertEqual(self.m.propose([audio()],[radar(stamp=STAMP+200_000_000)]),[])

    def test_invalid_covariance_and_uncertain_ray_abstain(self):
        self.assertEqual(self.m.propose([audio()],[radar(cov=(0.,)*9)]),[])
        self.assertEqual(self.m.propose([audio(variance=0.)],[radar()]),[])
        self.assertEqual(self.m.propose([audio(direction=(2.,0.,0.))],[radar()]),[])

    def test_two_radar_candidates_preserve_ambiguity(self):
        pairs=self.m.propose([audio()],[radar('r1'),radar('r2')])
        self.assertEqual(len(pairs),2)
        self.assertFalse(any(p.unique_match for p in pairs))

    def test_two_audio_candidates_preserve_ambiguity(self):
        pairs=self.m.propose([audio('a'),audio('b')],[radar()])
        self.assertEqual(len(pairs),2)
        self.assertFalse(any(p.unique_match for p in pairs))

    def test_duplicate_track_id_across_generations_fails_closed(self):
        other=RadarPoint('radar',2,STAMP,(3.,0.,0.),COV)
        self.assertEqual(self.m.propose([audio()],[radar(),other]),[])

    def test_radar_position_is_not_synthetic_audio_range(self):
        # Lateral geometry alone is not evidence that the acoustic source
        # is the measured radar object. No localization estimate is emitted.
        c=self.m.propose([audio()],[radar()])
        self.assertEqual(set(vars(c[0])),{
            'audio_id','audio_generation','radar_id','radar_generation',
            'range_along_ray_m','lateral_residual_m','normalized_residual',
            'unique_match'})


if __name__=='__main__':
    unittest.main()
