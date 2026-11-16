"""Real ROS 2 Humble tf2::Buffer admission tests for the optional M5 adapter.

Unlike the deterministic stamped gate tests, these exercise Buffer lookup,
interpolation, static transforms, missing frames and extrapolation. No robot,
audio source, Nav2 costmap, or network graph is required.
"""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
from nav_admission import MetricSource
from ros_tf_adapter import RosMetricTfGate, stamped_tf_lookup

NS = 1_000_000_000
COV = (.04, 0., 0., 0., .04, 0., 0., 0., .04)


def transform(stamp_ns, *, parent='map', child='microphone_array',
              tx=0., ty=0., yaw=0.):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp.sec = stamp_ns // NS
    msg.header.stamp.nanosec = stamp_ns % NS
    msg.transform.translation.x = tx
    msg.transform.translation.y = ty
    msg.transform.rotation.z = math.sin(yaw / 2.)
    msg.transform.rotation.w = math.cos(yaw / 2.)
    return msg


def source(stamp_ns=2 * NS, *, validated=True, frame='microphone_array'):
    return MetricSource('s1', frame, stamp_ns, stamp_ns + NS,
                        (1., 0., 0.), COV, validated)


class RealTf2BufferAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.buffer = Buffer(cache_time=None)
        self.buffer.set_transform(transform(NS, tx=2.), 'm5-test')
        self.buffer.set_transform(transform(3 * NS, tx=4.), 'm5-test')
        self.lookup = stamped_tf_lookup(self.buffer, timeout_sec=0.01)

    def test_interpolation_at_observation_time_not_latest(self):
        # A latest-TF fallback would produce x=5 rather than x=4.
        x, y = self.lookup('map', 'microphone_array', 2 * NS, (1., 0., 0.))
        self.assertAlmostEqual(x, 4.)
        self.assertAlmostEqual(y, 0.)

    def test_future_and_past_extrapolation_fail_closed(self):
        for stamp in (NS - 1, 3 * NS + 1):
            with self.subTest(stamp=stamp), self.assertRaises(LookupError):
                self.lookup('map', 'microphone_array', stamp, (1., 0., 0.))

    def test_missing_frame_and_invalid_source_never_authorized(self):
        gate = RosMetricTfGate('map', self.buffer, 2 * NS, timeout_sec=0.01)
        self.assertEqual(gate(source()), (4., 0.))
        with self.assertRaises(ValueError):
            gate(source(validated=False))
        with self.assertRaises(LookupError):
            gate(source(frame='unknown_microphone'))

    def test_rotation_and_static_transform(self):
        buf = Buffer()
        buf.set_transform_static(transform(0, child='static_mic',
                                           tx=2., ty=3., yaw=math.pi / 2.),
                                 'm5-test')
        lookup = stamped_tf_lookup(buf, timeout_sec=0.01)
        x, y = lookup('map', 'static_mic', 100 * NS, (1., 0., 0.))
        self.assertAlmostEqual(x, 2., places=6)
        self.assertAlmostEqual(y, 4., places=6)

    def test_tf2_normalizes_quaternion_before_lookup(self):
        # tf2 normalizes rotations on ingestion; malformed input cannot be
        # tested by round-tripping through a real Buffer.
        buf = Buffer()
        bad = transform(NS)
        bad.transform.rotation.w = 2.
        self.assertTrue(buf.set_transform(bad, 'm5-test'))
        result = stamped_tf_lookup(buf, timeout_sec=0.01)(
            'map', 'microphone_array', NS, (1., 0., 0.))
        self.assertAlmostEqual(result[0], 1.)
        self.assertAlmostEqual(result[1], 0.)

    def test_adapter_rejects_malformed_buffer_response(self):
        # Inject a deliberately broken Buffer result to cover the adapter's
        # fail-closed boundary, which a real Buffer sanitizes internally.
        class BrokenBuffer:
            def __init__(self, reply):
                self.reply = reply

            def lookup_transform(self, target, source, stamp, timeout):
                return self.reply

        cases = []
        bad_norm = transform(NS)
        bad_norm.transform.rotation.w = 2.
        cases.append(('nonunit quaternion', bad_norm, ValueError))
        bad_nan = transform(NS)
        bad_nan.transform.translation.x = float('nan')
        cases.append(('nonfinite translation', bad_nan, ValueError))
        bad_frame = transform(NS, parent='odom')
        cases.append(('mismatched parent frame', bad_frame, LookupError))
        bad_child = transform(NS, child='wrong_mic')
        cases.append(('mismatched child frame', bad_child, LookupError))
        for name, reply, error in cases:
            with self.subTest(name=name), self.assertRaises(error):
                stamped_tf_lookup(BrokenBuffer(reply), timeout_sec=0.01)(
                    'map', 'microphone_array', NS, (1., 0., 0.))


if __name__ == '__main__':
    unittest.main()
