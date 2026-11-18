"""M4 base-to-microphone covariance propagation regression tests."""
import math
import pathlib
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from pose_uncertainty import PlanarPose, project_microphone_origin


class PoseUncertaintyTests(unittest.TestCase):
    def setUp(self):
        self.pose = PlanarPose(10., 20., 0.,
            (.04, 0., 0.,
             0., .09, 0.,
             0., 0., .01))

    def test_offset_increases_transverse_uncertainty(self):
        o = project_microphone_origin(self.pose, (2., 0.))
        self.assertIsNotNone(o)
        self.assertAlmostEqual(o.x_m, 12.)
        self.assertAlmostEqual(o.y_m, 20.)
        self.assertAlmostEqual(o.covariance[0], .04)
        self.assertAlmostEqual(o.covariance[3], .13)

    def test_yaw_rotation_changes_uncertainty_axis(self):
        o = project_microphone_origin(replace(self.pose, yaw_rad=math.pi/2),
                                      (2., 0.))
        self.assertAlmostEqual(o.x_m, 10.)
        self.assertAlmostEqual(o.y_m, 22.)
        self.assertAlmostEqual(o.covariance[0], .08)
        self.assertAlmostEqual(o.covariance[3], .09)

    def test_zero_lever_arm_preserves_xy_covariance(self):
        o = project_microphone_origin(self.pose, (0., 0.))
        self.assertEqual(o.covariance, (.04, 0., 0., .09))

    def test_correlations_propagate_through_jacobian(self):
        p = replace(self.pose, covariance=(
            .04, 0., .01,
            0., .09, 0.,
            .01, 0., .01))
        o = project_microphone_origin(p, (2., 0.))
        self.assertIsNotNone(o)
        self.assertAlmostEqual(o.covariance[1], .02)
        self.assertAlmostEqual(o.covariance[2], .02)

    def test_unknown_or_non_psd_pose_fails_closed(self):
        for c in (
            None,
            (0.,) * 9,
            (-.01, 0., 0., 0., .09, 0., 0., 0., .01),
            (.04, .3, 0., .3, .09, 0., 0., 0., .01),
            (.04, .01, 0., 0., .09, 0., 0., 0., .01),
            (.04, 0., 0., 0., .09, 0., 0., 0., float('nan')),
        ):
            self.assertIsNone(project_microphone_origin(
                replace(self.pose, covariance=c), (1., 0.)))

    def test_bad_extrinsics_and_pose(self):
        for offset in (None, (1.,), (float('nan'), 0.), (0., float('inf'))):
            self.assertIsNone(project_microphone_origin(self.pose, offset))
        self.assertIsNone(project_microphone_origin(
            replace(self.pose, yaw_rad=float('inf')), (1., 0.)))


if __name__ == '__main__':
    unittest.main()
