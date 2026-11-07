"""M4 calibrated bearing-to-conditional-ray regression coverage."""
import math
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ray_evidence import planar_ray_from_observation


def fixtures():
    source = NS(has_direction=True, has_angular_variance=True,
                state=0, source_id='speaker', source_generation=1,
                angular_variance_rad2=.002,
                direction_unit=NS(x=1., y=0., z=0.))
    tf = NS(transform=NS(rotation=NS(x=0., y=0., z=0., w=1.)))
    sample = NS(stamp_ns=1_000_000_000,
                origin=NS(x_m=2., y_m=3.,
                          covariance=(.04, 0., 0., .09)))
    matched = NS(sample=sample, skew_ns=10_000_000)
    return source, tf, matched


class RayEvidenceTests(unittest.TestCase):
    def test_valid_ray_preserves_measured_uncertainties(self):
        source, tf, matched = fixtures()
        ray = planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000)
        self.assertIsNotNone(ray)
        self.assertEqual(ray.stamp_ns, 1_010_000_000)
        self.assertEqual(ray.origin, (2., 3.))
        self.assertEqual(ray.direction, (1., 0.))
        self.assertEqual(ray.angular_variance_rad2, .002)
        self.assertEqual(ray.origin_covariance, (.04, 0., 0., .09))

    def test_yaw_rotation(self):
        source, tf, matched = fixtures()
        tf.transform.rotation.z = math.sin(math.pi / 4)
        tf.transform.rotation.w = math.cos(math.pi / 4)
        ray = planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=990_000_000)
        self.assertIsNotNone(ray)
        self.assertAlmostEqual(ray.direction[0], 0., places=6)
        self.assertAlmostEqual(ray.direction[1], 1., places=6)

    def test_missing_or_unmeasured_variance_fails_closed(self):
        source, tf, matched = fixtures()
        source.has_angular_variance = False
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))
        source.has_angular_variance = True
        for value in (0., float('nan'), float('inf'), .2):
            source.angular_variance_rad2 = value
            self.assertIsNone(planar_ray_from_observation(
                source, tf, matched, expected_stamp_ns=1_010_000_000))

    def test_tilt_and_out_of_plane_direction_rejected(self):
        source, tf, matched = fixtures()
        source.direction_unit.z = .9
        source.direction_unit.x = math.sqrt(1 - .9 ** 2)
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))
        source.direction_unit.x, source.direction_unit.z = 1., 0.
        tf.transform.rotation.x = math.sin(.2)
        tf.transform.rotation.w = math.cos(.2)
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))

    def test_pose_timestamp_and_covariance_required(self):
        source, tf, matched = fixtures()
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_020_000_000))
        matched.sample.origin.covariance = None
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))

    def test_inactive_and_invalid_direction_rejected(self):
        source, tf, matched = fixtures()
        source.state = 2
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))
        source.state = 0
        source.direction_unit.x = float('nan')
        self.assertIsNone(planar_ray_from_observation(
            source, tf, matched, expected_stamp_ns=1_010_000_000))


if __name__ == '__main__':
    unittest.main()
