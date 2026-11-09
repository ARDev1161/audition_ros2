#!/usr/bin/env python3
"""M4 live TF2/pose cross-check: no TF, matching TF, contradictory TF."""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from audition_msgs.msg import AcousticSource, AcousticSourceArray
from tf2_ros import TransformBroadcaster

NS = '/robot_ci/pose_tf/'
SESSION = 'ef' * 16


def main():
    rclpy.init()
    node = Node('m4_tf_pose_graph_probe')
    poses = node.create_publisher(Odometry, NS + 'fusion/input_pose', 10)
    scenes = node.create_publisher(AcousticSourceArray, NS + 'sources', 10)
    broadcaster = TransformBroadcaster(node)
    statuses = []
    node.create_subscription(DiagnosticArray,
                             NS + 'fusion/pose_evidence_diagnostics',
                             lambda m: statuses.extend(m.status), 10)

    def counts():
        for status in reversed(statuses):
            if status.name == 'audition_fusion/pose_evidence':
                return {v.key: v.value for v in status.values}
        return {}

    def wait_for(predicate, timeout=7.):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False

    def expect_counter(key, target):
        assert wait_for(lambda: int(counts().get(key, 0)) >= target), (
            key, target, counts())

    def send(sequence, *, tf_x=None):
        stamp = node.get_clock().now().to_msg()
        pose = Odometry()
        pose.header.frame_id = 'odom'
        pose.header.stamp = stamp
        pose.pose.pose.position.x = 1.
        pose.pose.pose.orientation.w = 1.
        covariance = [0.] * 36
        covariance[0], covariance[7], covariance[35] = .04, .09, .01
        pose.pose.covariance = covariance
        poses.publish(pose)
        accepted = int(counts()['accepted_pose_samples'])
        expect_counter('accepted_pose_samples', accepted + 1)

        if tf_x is not None:
            tf = TransformStamped()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'microphone_array'
            tf.header.stamp = stamp
            tf.transform.translation.x = float(tf_x)
            tf.transform.rotation.w = 1.
            broadcaster.sendTransform(tf)
            # Process DDS discovery / TF listener before issuing the bearing.
            wait_for(lambda: False, .18)

        scene = AcousticSourceArray()
        scene.header.frame_id = 'microphone_array'
        scene.header.stamp = stamp
        scene.scene_session_id = SESSION
        scene.sequence_number = sequence
        source = AcousticSource()
        source.header = scene.header
        source.source_id = 'speaker'
        source.source_generation = 1
        source.state = AcousticSource.ACTIVE
        source.has_direction = True
        source.direction_unit.x = 1.
        source.last_seen = stamp
        scene.sources = [source]
        scenes.publish(scene)

    try:
        assert wait_for(lambda: poses.get_subscription_count() > 0 and
                        scenes.get_subscription_count() > 0 and
                        broadcaster.pub_tf.get_subscription_count() > 0 and
                        counts().get('require_tf_consistency') == 'True' and
                        counts().get('enable_conditional_evidence') == 'True'), (
                            'TF2 diagnostic node unavailable', counts())
        rejected = int(counts()['tf_rejected_bearings'])
        send(1)
        expect_counter('tf_rejected_bearings', rejected + 1)
        matched = int(counts()['matching_bearings'])
        missing_variance = int(counts()['conditional_rejected'])
        send(2, tf_x=1.)
        expect_counter('matching_bearings', matched + 1)
        expect_counter('conditional_rejected', missing_variance + 1)
        assert int(counts()['conditional_admitted']) == 0
        rejected = int(counts()['tf_rejected_bearings'])
        send(3, tf_x=3.)
        expect_counter('tf_rejected_bearings', rejected + 1)
        assert int(counts()['matching_bearings']) == matched + 1
        print('PASS: TF absent / consistent / contradictory; missing angular variance fails closed')
        return 0
    except Exception as exc:
        print('FAIL: M4 TF pose smoke:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
