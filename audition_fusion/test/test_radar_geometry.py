"""Radar validation never creates an acoustic localization result."""
import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from radar_geometry import quaternion_rotation, rotate_covariance, transform_track


class RadarGeometryTests(unittest.TestCase):
    def test_identity_transform(self):
        xyz,cov=transform_track((2.,-1.,3.),
            (4.,0.,0.,0.,1.,0.,0.,0.,9.),
            (0.,0.,0.,1.),(1.,2.,0.))
        self.assertEqual(xyz,(3.,1.,3.))
        self.assertEqual(cov,[4.,0.,0.,0.,1.,0.,0.,0.,9.])

    def test_quarter_turn_rotates_anisotropic_covariance(self):
        q=(0.,0.,math.sqrt(.5),math.sqrt(.5))
        xyz,cov=transform_track((2.,0.,0.),
            (4.,0.,0.,0.,1.,0.,0.,0.,9.),q,(0.,0.,0.))
        self.assertAlmostEqual(xyz[0],0.,places=8)
        self.assertAlmostEqual(xyz[1],2.,places=8)
        self.assertAlmostEqual(cov[0],1.,places=8)
        self.assertAlmostEqual(cov[4],4.,places=8)
        self.assertAlmostEqual(cov[8],9.,places=8)

    def test_rejects_missing_uncertainty_and_malformed_transform(self):
        valid=(1.,0.,0.,0.,1.,0.,0.,0.,1.)
        for cov in ((0.,)*9, (1.,)*8, (float('nan'),)*9,
                    (1.,2.,0.,2.,1.,0.,0.,0.,1.),
                    (1.,.3,0.,.2,1.,0.,0.,0.,1.)):
            # A zero covariance is *not* reported accuracy; a sensor must
            # supply nonzero measurement uncertainty.
            self.assertIsNone(transform_track((1.,0.,0.),cov,
                (0.,0.,0.,1.),(0.,0.,0.)))
        self.assertIsNone(transform_track((1.,0.,0.),valid,
            (0.,0.,0.,0.),(0.,0.,0.)))
        self.assertIsNone(transform_track((float('inf'),0.,0.),valid,
            (0.,0.,0.,1.),(0.,0.,0.)))
        self.assertIsNone(quaternion_rotation((0.,0.,float('nan'),1.)))

    def test_covariance_psd_semidefinite_is_accepted(self):
        out=rotate_covariance((1.,0.,0.,0.,1.,0.,0.,0.,0.),
                              quaternion_rotation((0.,0.,0.,1.)))
        self.assertIsNotNone(out)


if __name__=='__main__':
    unittest.main()
