"""Explicit-uncertainty geometry gates; no implied metric position."""
import math
import pathlib
import sys
import unittest
from dataclasses import replace
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from uncertainty import PlanarRay,conditional_planar_covariance


def rays(target=(3.,2.), origins=((0.,0.),(1.,0.),(2.,0.)),
         angle_var=.001,pose_var=.01):
    result=[]
    for idx,origin in enumerate(origins):
        dx,dy=target[0]-origin[0],target[1]-origin[1]
        scale=math.hypot(dx,dy)
        result.append(PlanarRay(1_000_000_000+idx*200_000_000,origin,
            (dx/scale,dy/scale),angle_var,
            (pose_var,0.,0.,pose_var)))
    return result


class UncertaintyTests(unittest.TestCase):
    def test_well_conditioned_explicit_uncertainty(self):
        value=conditional_planar_covariance((3.,2.),rays())
        self.assertIsNotNone(value)
        self.assertEqual(value.supporting_observations,3)
        self.assertLess(value.major_std_m,.5)
        self.assertGreater(value.covariance_xx_m2,0)
        self.assertGreater(value.covariance_yy_m2,0)
        self.assertGreater(value.covariance_xx_m2*value.covariance_yy_m2,
                           value.covariance_xy_m2**2)

    def test_unknown_or_non_psd_pose_covariance_never_inferred(self):
        samples=rays()
        for cov in [None,(.01,0.,0.),(.1,.2,.2,.01),
                    (.01,.03,0.,.01),(float('nan'),0.,0.,.01)]:
            changed=samples[:]
            changed[0]=replace(samples[0],origin_covariance=cov)
            self.assertIsNone(conditional_planar_covariance((3.,2.),changed))
        changed=samples[:]
        changed[0]=replace(samples[0],angular_variance_rad2=0)
        self.assertIsNone(conditional_planar_covariance((3.,2.),changed))

    def test_missing_baseline_and_parallel_rays(self):
        self.assertIsNone(conditional_planar_covariance((3.,2.),
            rays(origins=((0.,0.),(0.,0.),(0.,0.)))))
        parallel=[PlanarRay(1_000_000_000+i*200_000_000,
            (float(i),0.),(1.,0.),.001,(.01,0.,0.,.01))
            for i in range(3)]
        self.assertIsNone(conditional_planar_covariance((5.,0.),parallel))

    def test_inconsistent_third_bearing_fails_residual_gate(self):
        samples=rays()
        samples[2]=replace(samples[2],direction=(-.6,.8))
        self.assertIsNone(conditional_planar_covariance((3.,2.),samples))

    def test_time_and_backward_intersection_rejected(self):
        samples=rays()
        samples[1]=replace(samples[1],stamp_ns=samples[0].stamp_ns)
        self.assertIsNone(conditional_planar_covariance((3.,2.),samples))
        self.assertIsNone(conditional_planar_covariance((-1.,-1.),rays()))

    def test_uncertainty_grows_with_noise(self):
        low=conditional_planar_covariance((3.,2.),rays(),
            maximum_major_std_m=100)
        high=conditional_planar_covariance((3.,2.),rays(
            angle_var=.08,pose_var=.2),maximum_major_std_m=100)
        self.assertIsNotNone(low)
        self.assertIsNotNone(high)
        self.assertGreater(high.major_std_m,low.major_std_m)
        self.assertIsNone(conditional_planar_covariance((3.,2.),rays(
            angle_var=.08,pose_var=.2),maximum_major_std_m=.5))

    def test_no_two_ray_covariance(self):
        self.assertIsNone(conditional_planar_covariance((3.,2.),rays()[:2]))


if __name__=="__main__":
    unittest.main()
