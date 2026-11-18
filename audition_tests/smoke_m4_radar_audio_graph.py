#!/usr/bin/env python3
"""M4-D integrated radar TF2 -> radar/audio diagnostic DDS smoke."""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from audition_msgs.msg import (
    AcousticSource, AcousticSourceArray,
    RadarTrack, RadarTrackArray, RadarObservationArray,
    RadarAudioCandidateArray)

NS = '/robot_ci/radar_integrated/'
AUDIO = 'a'*32
RADAR = 'e'*32


def main():
    rclpy.init()
    node = Node('radar_audio_graph_probe')
    audio_pub = node.create_publisher(AcousticSourceArray,NS+'sources',10)
    radar_pub = node.create_publisher(RadarTrackArray,NS+'radar/tracks',10)
    tf_pub = TransformBroadcaster(node)
    candidates = []
    radar_out = []
    node.create_subscription(RadarAudioCandidateArray,
        NS+'fusion/radar_audio_candidates',candidates.append,10)
    node.create_subscription(RadarObservationArray,
        NS+'fusion/radar_observations',radar_out.append,10)

    def until(predicate, seconds=6):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.04)
            if predicate():
                return True
        return False

    def transforms(stamp):
        for frame in ('ci_radar_mic','ci_radar_sensor'):
            t = TransformStamped()
            t.header.frame_id = 'odom'
            t.header.stamp = stamp
            t.child_frame_id = frame
            t.transform.rotation.w = 1.
            tf_pub.sendTransform(t)

    def radar_frame(seq,stamp, *, session=RADAR, tracks=1):
        frame=RadarTrackArray()
        frame.header.frame_id='ci_radar_sensor'
        frame.header.stamp=stamp
        frame.radar_session_id=session
        frame.sequence_number=seq
        for i in range(tracks):
            track=RadarTrack()
            track.radar_track_id='object-'+str(i)
            track.radar_track_generation=1
            track.active=True
            track.position_m.x=3.
            track.position_covariance_m2=[
                .04,0.,0.,0.,.04,0.,0.,0.,.04]
            track.observed_at=stamp
            frame.tracks.append(track)
        return frame

    def audio_frame(seq,stamp, *, session=AUDIO, sources=1):
        frame=AcousticSourceArray()
        frame.header.frame_id='ci_radar_mic'
        frame.header.stamp=stamp
        frame.scene_session_id=session
        frame.spatial_session_id='c'*32
        frame.generation=1
        frame.sequence_number=seq
        for i in range(sources):
            source=AcousticSource()
            source.header=frame.header
            source.source_id='speaker-'+str(i)
            source.source_generation=1
            source.last_seen=stamp
            source.state=AcousticSource.ACTIVE
            source.has_direction=True
            source.direction_unit.x=1.
            source.has_angular_variance=True
            source.angular_variance_rad2=.01
            frame.sources.append(source)
        return frame

    def publish_frame(seq, *, sources=1, tracks=1, session=RADAR):
        stamp=node.get_clock().now().to_msg()
        transforms(stamp)
        # Give the TF listener a chance to ingest the stamped transforms.
        until(lambda: False,.12)
        last_radar=max((x.sequence_number for x in radar_out),default=0)
        radar_pub.publish(radar_frame(seq,stamp,session=session,tracks=tracks))
        assert until(lambda:any(x.sequence_number>last_radar
                                 for x in radar_out)), (
            'radar adapter did not deliver a new snapshot')
        audio_pub.publish(audio_frame(seq,stamp,sources=sources))
        return stamp

    try:
        assert until(lambda:audio_pub.get_subscription_count()>=1
                     and radar_pub.get_subscription_count()>=1
                     and tf_pub.pub_tf.get_subscription_count()>=2), (
                         'integrated graph not discovered')
        stamp=node.get_clock().now().to_msg()
        transforms(stamp)
        until(lambda:False,.2)

        for seq in range(1,4):
            prior=max((x.sequence_number for x in candidates),default=0)
            publish_frame(seq)
            assert until(lambda:any(
                x.sequence_number>prior
                and len(x.candidates)==1
                and x.candidates[0].supporting_synchronized_observations==seq
                for x in candidates)), (
                    'independent radar/audio evidence did not advance',
                    [(x.sequence_number,len(x.candidates),
                      [c.supporting_synchronized_observations
                       for c in x.candidates]) for x in candidates[-8:]])
            current=next(x for x in reversed(candidates)
                         if x.sequence_number>prior
                         and len(x.candidates)==1
                         and x.candidates[0].supporting_synchronized_observations==seq)
            item=current.candidates[0]
            assert abs(item.radar_range_along_audio_ray_m-3.)<1e-6
            assert item.unique_match
            assert item.temporally_correlated==(seq==3)
            assert current.audio_scene_session_id==AUDIO
            assert current.radar_session_id==RADAR

        # One acoustic source and two radar objects: never uniquely match.
        prior=max(x.sequence_number for x in candidates)
        publish_frame(4,tracks=2)
        assert until(lambda:any(
            x.sequence_number>prior and len(x.candidates)==2
            for x in candidates)), 'radar competition not published'
        ambiguous=next(x for x in reversed(candidates)
                       if x.sequence_number>prior and len(x.candidates)==2)
        assert not any(c.unique_match or c.temporally_correlated
                       for c in ambiguous.candidates)

        # Reverse competition: two acoustic sources with one radar object.
        prior=ambiguous.sequence_number
        publish_frame(5,sources=2)
        assert until(lambda:any(
            x.sequence_number>prior and len(x.candidates)==2
            for x in candidates)), 'acoustic competition not published'
        ambiguous=next(x for x in reversed(candidates)
                       if x.sequence_number>prior and len(x.candidates)==2)
        assert not any(c.unique_match or c.temporally_correlated
                       for c in ambiguous.candidates)

        # An authoritative empty radar snapshot immediately revokes all.
        prior=max(x.sequence_number for x in candidates)
        stamp=node.get_clock().now().to_msg()
        radar_pub.publish(radar_frame(6,stamp,tracks=0))
        assert until(lambda:any(x.sequence_number>prior and not x.candidates
                                for x in candidates)), 'empty radar not revoked'

        # Fresh radar epoch can restart at sequence 1, without reviving old
        # confidence. Old epoch with a newer capture stamp is rejected.
        prior=max(x.sequence_number for x in candidates)
        stamp=publish_frame(1,session='f'*32)
        # This is not a pair count recovery guarantee: audio seq=1 is a replay
        # and cannot be admitted; radar epoch must still appear on output.
        assert until(lambda:any(x.sequence_number>prior
                and x.radar_session_id=='f'*32 for x in candidates))
        prev=max(x.sequence_number for x in candidates)
        radar_pub.publish(radar_frame(99,node.get_clock().now().to_msg(),
                                      session=RADAR,tracks=1))
        until(lambda:False,.15)
        assert not any(x.sequence_number>prev and x.radar_session_id==RADAR
                       for x in candidates), 'retired radar epoch resumed'

        topics=dict(node.get_topic_names_and_types())
        assert NS+'localized_sources' not in topics
        assert NS+'cmd_vel' not in topics
        print('PASS: integrated radar TF2 + audio DDS temporal/ambiguity, '
              'retirement, replay; no localization or motion')
        return 0
    except Exception as error:
        print('FAIL: radar/audio integrated graph:',repr(error),file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
