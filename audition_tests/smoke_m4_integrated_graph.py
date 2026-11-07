#!/usr/bin/env python3
"""M4-A one-launch multi-consumer ROS DDS integration smoke, no robot hardware."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from audition_msgs.msg import (
    AcousticSource, AcousticSourceArray,
    VisualBearingObservation, VisualBearingObservationArray,
    AudioVisualPairHypothesisArray, BearingIntersectionCandidateArray)

NS = '/robot_ci/m4_integrated/'
AUDIO_SESSION = 'a' * 32
VISION_SESSION = 'b' * 32


def main():
    rclpy.init()
    node = Node('m4_integrated_graph_probe')
    audio = node.create_publisher(AcousticSourceArray, NS + 'sources', 10)
    vision = node.create_publisher(VisualBearingObservationArray,
                                   NS + 'vision/bearings', 10)
    pose_pub = node.create_publisher(Odometry, NS + 'fusion/input_pose', 10)
    tf_pub = TransformBroadcaster(node)
    pairs, candidates, statuses = [], [], []
    subs = [
        node.create_subscription(AudioVisualPairHypothesisArray,
            NS + 'fusion/av_pair_hypotheses', pairs.append, 10),
        node.create_subscription(BearingIntersectionCandidateArray,
            NS + 'fusion/bearing_candidates', candidates.append, 10),
        node.create_subscription(DiagnosticArray,
            NS + 'fusion/pose_evidence_diagnostics',
            lambda m: statuses.extend(m.status), 10),
    ]

    def until(predicate, seconds=8.):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False

    def diagnostics():
        for s in reversed(statuses):
            if s.name == 'audition_fusion/pose_evidence':
                return {kv.key: kv.value for kv in s.values}
        return {}

    try:
        assert until(lambda: audio.get_subscription_count() == 3
                    and vision.get_subscription_count() >= 1
                    and pose_pub.get_subscription_count() >= 1
                    and tf_pub.pub_tf.get_subscription_count() >= 2
                    and bool(diagnostics())), (
                        'integrated nodes not discovered',
                        audio.get_subscription_count(),
                        vision.get_subscription_count(),
                        pose_pub.get_subscription_count(), diagnostics())
        assert diagnostics()['world_frame'] == 'odom'
        assert diagnostics()['enable_conditional_evidence'] == 'False'

        # Prime TF2 graph discovery before starting the 500ms pose TTL.
        # The final pose, TF and acoustic sample use a fresh common stamp.
        for frame, x in [('m4_integrated_mic', 1.),
                         ('m4_integrated_camera', 1.05)]:
            tf = TransformStamped()
            tf.header.frame_id = 'odom'
            tf.header.stamp = node.get_clock().now().to_msg()
            tf.child_frame_id = frame
            tf.transform.translation.x = x
            tf.transform.rotation.w = 1.
            tf_pub.sendTransform(tf)
        until(lambda: False, .18)

        stamp = node.get_clock().now().to_msg()
        for frame, x in [('m4_integrated_mic', 1.),
                         ('m4_integrated_camera', 1.05)]:
            tf = TransformStamped()
            tf.header.frame_id = 'odom'
            tf.header.stamp = stamp
            tf.child_frame_id = frame
            tf.transform.translation.x = x
            tf.transform.rotation.w = 1.
            tf_pub.sendTransform(tf)

        pose = Odometry()
        pose.header.frame_id = 'odom'
        pose.header.stamp = stamp
        pose.pose.pose.position.x = 1.
        pose.pose.pose.orientation.w = 1.
        cov = [0.] * 36
        cov[0], cov[7], cov[35] = .04, .04, .01
        pose.pose.covariance = cov
        pose_pub.publish(pose)
        assert until(lambda: int(diagnostics().get('accepted_pose_samples', 0)) >= 1), (
            'pose adapter did not accept good odometry', diagnostics())
        # No fixed post-pose wait: it would consume the limited TTL.
        a = AcousticSourceArray()
        a.header.frame_id = 'm4_integrated_mic'
        a.header.stamp = stamp
        a.scene_session_id = AUDIO_SESSION
        a.spatial_session_id = 'c' * 32
        a.sequence_number = 1
        a.generation = 1
        src = AcousticSource()
        src.header = a.header
        src.source_id = 'sound'
        src.source_generation = 1
        src.state = AcousticSource.ACTIVE
        src.has_direction = True
        src.has_angular_variance = True
        src.angular_variance_rad2 = .01
        src.direction_unit.x = 1.
        src.last_seen = stamp
        a.sources = [src]

        v = VisualBearingObservationArray()
        v.header.frame_id = 'm4_integrated_camera'
        v.header.stamp = stamp
        v.vision_session_id = VISION_SESSION
        v.sequence_number = 1
        visual = VisualBearingObservation()
        visual.visual_track_id = 'face'
        visual.visual_track_generation = 1
        visual.active = True
        visual.activity_evidence = True
        visual.has_angular_variance = True
        visual.angular_variance_rad2 = .01
        visual.direction_unit.x = 1.
        visual.observed_at = stamp
        v.tracks = [visual]

        audio.publish(a)
        vision.publish(v)
        assert until(lambda: candidates and
                     any(len(p.hypotheses) == 1 for p in pairs) and
                     int(diagnostics().get('matching_bearings', 0)) >= 1), (
                        'integrated graph did not converge',
                        len(pairs), len(candidates), diagnostics())
        assert all(not c.candidates for c in candidates), (
            'one-view bearing unexpectedly triangulated')
        assert not any(p.hypotheses[0].temporally_confirmed
                       for p in pairs if len(p.hypotheses) == 1), (
            'one-view AV proposal marked confirmed')
        # An authoritative empty canonical snapshot must revoke the last
        # AV hypothesis in the same combined graph, without awaiting TTL.
        last_pair_sequence = max(p.sequence_number for p in pairs)
        last_candidate_sequence = max(c.sequence_number for c in candidates)
        retired = AcousticSourceArray()
        retired.header.frame_id = a.header.frame_id
        retired.header.stamp = node.get_clock().now().to_msg()
        retired.scene_session_id = AUDIO_SESSION
        retired.spatial_session_id = a.spatial_session_id
        retired.generation = a.generation
        retired.sequence_number = 2
        audio.publish(retired)
        assert until(lambda: any(
            p.sequence_number > last_pair_sequence and not p.hypotheses
            for p in pairs) and any(
            c.sequence_number > last_candidate_sequence and not c.candidates
            for c in candidates)), (
                'retired source still has diagnostic hypotheses',
                [(p.sequence_number, len(p.hypotheses)) for p in pairs[-4:]],
                [(c.sequence_number, len(c.candidates))
                 for c in candidates[-4:]])

        # Simulate scene process restart: a fresh incarnation can start at
        # sequence 1, but it must retire the old pose evidence.
        restart_count = int(diagnostics()['session_restarts'])
        restarted = AcousticSourceArray()
        restarted.header.frame_id = a.header.frame_id
        restarted.header.stamp = node.get_clock().now().to_msg()
        restarted.scene_session_id = 'd' * 32
        restarted.spatial_session_id = a.spatial_session_id
        restarted.generation = a.generation
        restarted.sequence_number = 1
        audio.publish(restarted)
        assert until(lambda: int(diagnostics().get('session_restarts', 0))
                     >= restart_count + 1), (
                         'scene restart was not admitted', diagnostics())
        assert int(diagnostics()['buffered_pose_samples']) == 0

        # A retired incarnation cannot reintroduce its old observations.
        replayed = int(diagnostics()['replayed_scenes'])
        old = AcousticSourceArray()
        old.header.frame_id = a.header.frame_id
        old.header.stamp = node.get_clock().now().to_msg()
        old.scene_session_id = AUDIO_SESSION
        old.spatial_session_id = a.spatial_session_id
        old.generation = a.generation
        old.sequence_number = 3
        audio.publish(old)
        assert until(lambda: int(diagnostics().get('replayed_scenes', 0))
                     >= replayed + 1), (
                         'retired scene incarnation unexpectedly admitted',
                         diagnostics())
        # No M4 consumer is entitled to publish authoritative localization.
        topics = dict(node.get_topic_names_and_types())
        assert NS + 'localized_sources' not in topics
        assert NS + 'cmd_vel' not in topics
        print('PASS: one-launch M4 DDS AV + TF bearing + pose; '
              'retirement, restart and replay fencing; no motion')
        return 0
    except Exception as exc:
        print('FAIL: M4 integrated graph:', repr(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
