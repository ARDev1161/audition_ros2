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
    visual_echo=[]
    node.create_subscription(
        AudioVisualPairHypothesisArray,NS+'fusion/av_pair_hypotheses',
        lambda m:received.append(m),10)
    # Independent graph probe: distinguish a lost visual DDS sample from
    # an AV-node admission/TF2/matching failure.
    node.create_subscription(
        VisualBearingObservationArray,NS+'vision/bearings',
        lambda m:visual_echo.append((m.sequence_number,len(m.tracks))),10)

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
              [(len(m.hypotheses),m.sequence_number) for m in received[-12:]],
              'visual_echo',visual_echo[-12:],
              'visual_subscribers',visual.get_subscription_count(),
              'acoustic_subscribers',acoustic.get_subscription_count(),
              'av_publishers',node.count_publishers(NS+'fusion/av_pair_hypotheses')))

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
        # A matched /tf publisher does not guarantee the listener has
        # consumed its first transform. Re-broadcast the SAME stamped TF
        # after discovery settles, before publishing sensor observations.
        spin_for(.25)
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
        spin_for(.20)

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
        # First prove the producer's DDS sample is visible independently of
        # the fusion node. An AV timeout alone cannot distinguish transport
        # failure from TF2/admission/association rejection.
        deadline=time.monotonic()+1.5
        while (1,1) not in visual_echo and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert (1,1) in visual_echo, (
            'visual DDS delivery failed',visual_echo[-12:],
            visual.get_subscription_count())
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

        # A camera that goes silent (without publishing an empty snapshot)
        # must not leave the last confirmed pairing indefinitely visible.
        # The AV node defaults to an 800ms observation-age timeout.
        last_confirmed_sequence=max(m.sequence_number for m in received)
        expired=find(lambda m:m.sequence_number>last_confirmed_sequence
                     and not m.hypotheses,timeout=3.0)
        assert not expired.hypotheses, 'stale camera evidence retained'

        stamp=node.get_clock().now().to_msg()
        # Camera rotates +90 degrees, requiring historical TF2 orientation.
        prepare_tf(stamp,camera_yaw=math.pi/2)
        acoustic.publish(audio_msg(4,stamp))
        visual.publish(vision_msg(4,stamp,['face-a','face-b'],local_yaw=-math.pi/2))
        ambiguous=find(lambda m:len(m.hypotheses)==2)
        assert not any(p.unique_match for p in ambiguous.hypotheses), 'ambiguity concealed'
        assert not any(p.temporally_confirmed for p in ambiguous.hypotheses), 'stale confirmation'

        # A single visual ID cannot denote two generations in one snapshot.
        # The entire snapshot must withdraw the prior hypothesis.
        stamp=node.get_clock().now().to_msg()
        prepare_tf(stamp)
        duplicate=vision_msg(5,stamp,['face-a','face-a'])
        duplicate.tracks[1].visual_track_generation=2
        visual.publish(duplicate)
        withdrawn=find(lambda m:m.sequence_number>ambiguous.sequence_number
                       and not m.hypotheses)
        assert not withdrawn.hypotheses, 'duplicate visual ID was accepted'

        stamp=node.get_clock().now().to_msg()
        visual.publish(vision_msg(6,stamp,[]))
        cleared=find(lambda m:m.sequence_number>ambiguous.sequence_number
                    and not m.hypotheses)
        assert not cleared.hypotheses
        # Reacquisition with a *different visual identity* is an independent
        # hypothesis, even if the bearing is geometrically identical.
        stamp=node.get_clock().now().to_msg()
        prepare_tf(stamp)
        acoustic.publish(audio_msg(5,stamp))
        # Sequence 6 was the authoritative empty snapshot; reacquisition must
        # advance the sequence rather than replay it (StreamFence rejects <=).
        visual.publish(vision_msg(7,stamp,['face-new']))
        fresh=find(lambda m:m.sequence_number>cleared.sequence_number
                   and len(m.hypotheses)==1
                   and m.hypotheses[0].visual_track_id=='face-new')
        assert fresh.hypotheses[0].supporting_synchronized_observations==1
        assert not fresh.hypotheses[0].temporally_confirmed

        # A process restart may begin with sequence 1 on a fresh visual
        # incarnation, but confirmation must reset even with the same ID.
        stamp=node.get_clock().now().to_msg()
        prepare_tf(stamp)
        acoustic.publish(audio_msg(6,stamp))
        new_vision=vision_msg(1,stamp,['face-new'])
        new_vision.vision_session_id='d'*32
        visual.publish(new_vision)
        restarted=find(lambda m:m.sequence_number>fresh.sequence_number
                      and m.vision_session_id=='d'*32
                      and len(m.hypotheses)==1
                      and m.hypotheses[0].visual_track_id=='face-new')
        assert restarted.hypotheses[0].supporting_synchronized_observations==1
        assert not restarted.hypotheses[0].temporally_confirmed

        # A retired vision incarnation must never reopen with a later
        # timestamp, even if its sequence has advanced.
        before=len(received)
        old_vision=vision_msg(99,node.get_clock().now().to_msg(),['old-face'])
        old_vision.vision_session_id=VISION_EPOCH
        visual.publish(old_vision)
        spin_for(.15)
        assert not any(m.vision_session_id==VISION_EPOCH
                       for m in received[before:]), (
                           'retired visual incarnation reentered the graph')

        # Explicit loss of camera tracks revokes any live pairing without
        # waiting for a timer-based TTL expiry.
        stamp=node.get_clock().now().to_msg()
        empty=vision_msg(2,stamp,[])
        empty.vision_session_id='d'*32
        visual.publish(empty)
        gone=find(lambda m:m.sequence_number>restarted.sequence_number
                  and m.vision_session_id=='d'*32
                  and not m.hypotheses)
        assert not gone.hypotheses
        # Reverse ambiguity: two equally plausible audio tracks compete
        # for one visual track. Neither pairing may be marked unique or
        # temporally confirmed, despite matching timestamps and bearings.
        stamp=node.get_clock().now().to_msg()
        prepare_tf(stamp)
        two_audio=audio_msg(7,stamp)
        competitor=AcousticSource()
        competitor.header=two_audio.sources[0].header
        competitor.last_seen=stamp
        competitor.state=AcousticSource.ACTIVE
        competitor.source_id='competing-sound'
        competitor.source_generation=1
        competitor.has_direction=True
        competitor.direction_unit.x=1.
        competitor.has_angular_variance=True
        competitor.angular_variance_rad2=.01
        two_audio.sources.append(competitor)
        one_visual=vision_msg(3,stamp,['face-new'])
        one_visual.vision_session_id='d'*32
        acoustic.publish(two_audio)
        visual.publish(one_visual)
        reverse=find(lambda m:m.sequence_number>gone.sequence_number
                     and m.vision_session_id=='d'*32
                     and len(m.hypotheses)==2)
        assert {p.audio_source_id for p in reverse.hypotheses}=={
            'speaker-hyp','competing-sound'}
        assert not any(p.unique_match or p.temporally_confirmed
                       for p in reverse.hypotheses), (
                           'audio competition incorrectly promoted a pair')

        # Acoustic source loss must revoke all AV matches, without
        # depending on camera shutdown or expiry of observation TTL.
        stamp=node.get_clock().now().to_msg()
        no_audio=audio_msg(8,stamp)
        no_audio.sources=[]
        acoustic.publish(no_audio)
        revoked=find(lambda m:m.sequence_number>reverse.sequence_number
                     and not m.hypotheses)
        assert not revoked.hypotheses

        print('PASS: live AV TF2 pairing, both ambiguity directions, TTL, '
              'retirement, identity/epoch changes and replay fencing')
        return 0
    except Exception as exc:
        print('FAIL: AV DDS/TF2 graph:',repr(exc),file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
