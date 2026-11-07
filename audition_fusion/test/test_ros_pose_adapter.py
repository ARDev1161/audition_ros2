"""Tests ROS PoseWithCovarianceStamped and Odometry-like inputs."""
import math
import pathlib
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from ros_pose_adapter import from_ros_pose_message

NOW = 2_000_000_000


def sample(*, frame="odom", stamp=1_900_000_000, yaw=0., odometry=False):
    cov = [0.] * 36
    cov[0], cov[7], cov[35] = .04, .09, .01
    q = NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))
    inner = NS(position=NS(x=10., y=20., z=0.), orientation=q)
    wrapped = NS(pose=inner, covariance=cov)
    result = NS(header=NS(frame_id=frame, stamp=NS(
        sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000)),
        pose=wrapped)
    if odometry:
        result.child_frame_id = "base_link"
    return result


def adapt(msg, **kw):
    return from_ros_pose_message(
        msg, world_frame="odom", lever_arm_xy=(2., 0.),
        now_ns=NOW, **kw)


class RosPoseAdapterTests(unittest.TestCase):
    def test_pose_with_covariance_stamped(self):
        out = adapt(sample())
        self.assertIsNotNone(out)
        self.assertEqual(out.frame_id, "odom")
        self.assertEqual(out.stamp_ns, 1_900_000_000)
        self.assertAlmostEqual(out.origin.x_m, 12.)
        self.assertAlmostEqual(out.origin.covariance[3], .13)

    def test_odometry_uses_parent_frame_not_child_frame(self):
        self.assertIsNotNone(adapt(sample(odometry=True)))
        self.assertIsNone(adapt(sample(frame="base_link", odometry=True)))

    def test_rotation_changes_lever_arm_and_covariance(self):
        out = adapt(sample(yaw=math.pi/2))
        self.assertAlmostEqual(out.origin.y_m, 22.)
        self.assertAlmostEqual(out.origin.covariance[0], .08)

    def test_missing_covariance_and_unknown_pose_fail_closed(self):
        msg = sample()
        msg.pose.covariance = [0.] * 36
        self.assertIsNone(adapt(msg))
        msg.pose.covariance = [-1.] * 36
        self.assertIsNone(adapt(msg))
        msg.pose.covariance = []
        self.assertIsNone(adapt(msg))
        self.assertIsNone(adapt(NS()))

    def test_pose_frame_and_timestamp_fences(self):
        self.assertIsNone(adapt(sample(frame="map")))
        self.assertIsNone(adapt(sample(stamp=1_000_000_000)))
        self.assertIsNone(adapt(sample(stamp=2_100_000_000)))
        self.assertIsNone(adapt(sample(stamp=0)))
        self.assertIsNone(adapt(sample(), maximum_age_ns=0))

    def test_reject_tilt_nonunit_quaternion_and_nan(self):
        msg = sample()
        msg.pose.pose.orientation.x = math.sin(.2)
        msg.pose.pose.orientation.w = math.cos(.2)
        self.assertIsNone(adapt(msg))
        msg = sample()
        msg.pose.pose.orientation.w = 0.
        self.assertIsNone(adapt(msg))
        msg = sample()
        msg.pose.pose.position.x = float("nan")
        self.assertIsNone(adapt(msg))

    def test_correlations_carry_from_ros_covariance(self):
        msg = sample()
        msg.pose.covariance[5] = .01
        msg.pose.covariance[30] = .01
        out = adapt(msg)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.origin.covariance[1], .02)


if __name__ == "__main__":
    unittest.main()
