"""M4 independently testable TF/pose origin admission."""
import pathlib
import sys
import unittest
from dataclasses import replace
from types import SimpleNamespace as NS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from tf_pose_consistency import TransformOrigin, consistent_origin


class TransformPoseConsistencyTests(unittest.TestCase):
    def setUp(self):
        self.pose = NS(frame_id='odom', stamp_ns=1_000_000_000,
                       origin=NS(x_m=2.0, y_m=3.0))
        self.tf = TransformOrigin('odom', 'microphone_array',
                                  1_000_000_000, 2.1, 3.0)

    def check(self, tf=None, pose=None):
        return consistent_origin(
            self.pose if pose is None else pose,
            self.tf if tf is None else tf,
            expected_world='odom', expected_child='microphone_array')

    def test_valid_metric_and_time_consistency(self):
        self.assertTrue(self.check())
        self.assertTrue(self.check(tf=replace(self.tf, stamp_ns=1_050_000_000)))

    def test_wrong_frames_and_static_tf_rejected(self):
        self.assertFalse(self.check(tf=replace(self.tf, frame_id='map')))
        self.assertFalse(self.check(tf=replace(self.tf, child_frame_id='base_link')))
        self.assertFalse(self.check(tf=replace(self.tf, stamp_ns=0)))
        self.assertFalse(self.check(pose=NS(frame_id='map', stamp_ns=1_000_000_000,
                                            origin=self.pose.origin)))

    def test_time_skew_and_disagreement_rejected(self):
        self.assertFalse(self.check(tf=replace(self.tf, stamp_ns=1_050_000_001)))
        self.assertFalse(self.check(tf=replace(self.tf, x_m=2.251)))
        self.assertFalse(self.check(tf=replace(self.tf, x_m=float('nan'))))
        self.assertFalse(self.check(tf=replace(self.tf, y_m=float('inf'))))

    def test_missing_geometry_and_bad_configuration_fail_closed(self):
        self.assertFalse(self.check(pose=NS(frame_id='odom', stamp_ns=1,
                                             origin=None)))
        self.assertFalse(consistent_origin(
            self.pose, self.tf, expected_world='odom',
            expected_child='microphone_array', maximum_disagreement_m=-1))


if __name__ == '__main__':
    unittest.main()
