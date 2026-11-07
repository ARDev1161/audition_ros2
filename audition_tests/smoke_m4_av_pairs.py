#!/usr/bin/env python3
"""M4 AV diagnostic live DDS/TF2 graph smoke; never movement or identity."""
import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from audition_msgs.msg import (AcousticSource, AcousticSourceArray,
    VisualBearingObservation, VisualBearingObservationArray,
    AudioVisualPairHypothesisArray)

NS='/robot_ci/av/'
AUDIO_EPOCH='a'*32
VISION_EPOCH='b'*32


def main():
    rclpy.init()
    node=Node('m4_av_probe')
    tfpub=TransformBroadcaster(node)
    acoustic=node.create_publisher(AcousticSourceArray,NS+'sources',10)
    visual=node.create_publisher(VisualBearingObservationArray,NS+'vision/bearings',10)
    received=[]
    node.create_subscription(
        AudioVisualPairHypothesisArray,NS+'fusion/av_pair_hypotheses',
        lambda m:received.append(m),10)

    def spin_for(dt):
        deadline=time.monotonic()+dt
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.04)

    def find(predicate,timeout=4.0):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if any(predicate(m) for m in received):
                return next(m for m in reversed(received) if predicate(m))
            rclpy.spin_once(node,timeout_sec=.05)
        raise AssertionError(('missing AV observation',len(received),
              [(len(m.hypotheses),m.sequence_number) for m in received[-12:]]))

    def prepare_tf(stamp,camera_yaw=0.0):
        for frame,x in [('robot_ci_av_mic',0.),('robot_ci_av_camera',.05)]:
            tf=TransformStamped()
            tf.header.stamp=stamp
            tf.header.frame_id='odom'
            tf.child_frame_id=frame
            tf.transform.translation.x=x
            yaw=camera_yaw if frame=='robot_ci_av_camera' else 0.0
            tf.transform.rotation.z=math.sin(yaw/2.0)
            tf.transform.rotation.w=math.cos(yaw/2.0)
            tfpub.sendTransform(tf)
        spin_for(.22)

    def audio_msg(sequence,stamp):
        a=AcousticSourceArray()
        a.header.frame_id='robot_ci_av_mic'
        a.header.stamp=stamp
        a.scene_session_id=AUDIO_EPOCH
        a.sequence_number=sequence
        a.generation=1
        a.spatial_session_id='c'*32
        s=AcousticSource()
        s.header.stamp=stamp
        s.header.frame_id='robot_ci_av_mic'
        s.last_seen=stamp
        s.state=AcousticSource.ACTIVE
        s.source_id='speaker-hyp'
        s.source_generation=1
        s.has_direction=True
        s.direction_unit.x=1.
        s.has_angular_variance=True
        s.angular_variance_rad2=.01
        a.sources=[s]
        return a

    def vision_msg(sequence,stamp,track_ids,local_yaw=0.0):
        v=VisualBearingObservationArray()
        v.header.frame_id='robot_ci_av_camera'
        v.header.stamp=stamp
        v.vision_session_id=VISION_EPOCH
        v.sequence_number=sequence
        for tid in track_ids:
            t=VisualBearingObservation()
            t.visual_track_id=tid
            t.visual_track_generation=1
            t.active=True
            t.activity_evidence=True
            t.has_angular_variance=True
            t.angular_variance_rad2=.01
            t.direction_unit.x=math.cos(local_yaw)
            t.direction_unit.y=math.sin(local_yaw)
            t.observed_at=stamp
            v.tracks.append(t)
        return v

    try:
        deadline=time.monotonic()+16
        while (acoustic.get_subscription_count()==0 or visual.get_subscription_count()==0
               or tfpub.pub_tf.get_subscription_count()==0) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        assert acoustic.get_subscription_count() and visual.get_subscription_count(), 'node not discovered'
        assert tfpub.pub_tf.get_subscription_count(), 'TF2 not discovered'

        stamp=node.get_clock().now().to_msg()
        prepare_tf(stamp)
        acoustic.publish(audio_msg(1,stamp))
        spin_for(.08)
        assert not any(m.hypotheses for m in received), 'paired without vision'
        visual.publish(vision_msg(1,stamp,['face-a']))
        first=find(lambda m:len(m.hypotheses)==1 and
                   m.hypotheses[0].visual_track_id=='face-a')
        assert first.hypotheses[0].unique_match, 'single reciprocal pair not unique'
        assert first.hypotheses[0].supporting_synchronized_observations == 1
        assert not first.hypotheses[0].temporally_confirmed

        for seq in [2, 3]:
            stamp=node.get_clock().now().to_msg()
            prepare_tf(stamp)
            acoustic.publish(audio_msg(seq, stamp))
            visual.publish(vision_msg(seq, stamp, ['face-a']))
            current=find(lambda m: any(
                p.visual_track_id == 'face-a' and
                p.supporting_synchronized_observations == seq for p in m.hypotheses))
            assert current.hypotheses[0].temporally_confirmed == (seq == 3)
        assert first.audio_scene_session_id==AUDIO_EPOCH
        assert first.vision_session_id==VISION_EPOCH
        assert first.hypotheses[0].sensor_baseline_m<.1

        stamp=node.get_clock().now().to_msg()
        # Camera rotates +90 degrees, requiring historical TF2 orientation.
        prepare_tf(stamp,camera_yaw=math.pi/2)
        acoustic.publish(audio_msg(2,stamp))
        visual.publish(vision_msg(2,stamp,['face-a','face-b'],local_yaw=-math.pi/2))
        ambiguous=find(lambda m:len(m.hypotheses)==2)
        assert not any(p.unique_match for p in ambiguous.hypotheses), 'ambiguity concealed'
        assert not any(p.temporally_confirmed for p in ambiguous.hypotheses), 'stale confirmation'

        stamp=node.get_clock().now().to_msg()
        visual.publish(vision_msg(3,stamp,[]))
        cleared=find(lambda m:m.sequence_number>ambiguous.sequence_number
                    and not m.hypotheses)
        assert not cleared.hypotheses
        print('PASS: live AV TF2 rotated-frame pairing, ambiguity, retirement clearing')
        return 0
    except Exception as exc:
        print('FAIL: AV DDS/TF2 graph:',repr(exc),file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
