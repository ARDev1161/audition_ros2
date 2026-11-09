#!/usr/bin/env python3
"""Opt-in timestamped TF2 AV pair suggestions; no verified identity or range."""
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from audition_msgs.msg import (AcousticSource, AcousticSourceArray,
    VisualBearingObservationArray, AudioVisualPairHypothesis,
    AudioVisualPairHypothesisArray)
from av_matching import PairMatcher, Ray, StreamFence
from av_confirmation import TemporalPairGate


def ns(stamp):
    return (stamp.sec * 1000000000 + stamp.nanosec
            if stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000 else 0)


def rotated(v, q):
    vals = (v.x,v.y,v.z,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(x) for x in vals):
        return None
    norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if abs(norm-1) > 0.02 or norm < 1e-9:
        return None
    x,y,z,w = q.x/math.sqrt(norm),q.y/math.sqrt(norm),q.z/math.sqrt(norm),q.w/math.sqrt(norm)
    a,b,c=v.x,v.y,v.z
    tx,ty,tz = 2*(y*c-z*b),2*(z*a-x*c),2*(x*b-y*a)
    return (a+w*tx+(y*tz-z*ty),b+w*ty+(z*tx-x*tz),c+w*tz+(x*ty-y*tx))


class AvPairNode(Node):
    def __init__(self):
        super().__init__('av_pair_node')
        self.frame = self.declare_parameter('world_frame','odom').value
        self.age_ns = int(self.declare_parameter('maximum_observation_age_ms',800).value)*1000000
        self.future_ns = 100000000
        self.matcher=PairMatcher(
            maximum_skew_ms=float(self.declare_parameter('maximum_skew_ms',150.0).value),
            maximum_baseline_m=float(self.declare_parameter('maximum_sensor_separation_m',0.2).value),
            maximum_angle_rad=float(self.declare_parameter('maximum_pair_angle_rad',0.3).value),
            minimum_assumed_range_m=float(self.declare_parameter('minimum_assumed_range_m',1.0).value))
        self.temporal = TemporalPairGate(
            confirmations=int(self.declare_parameter('minimum_pair_confirmations',3).value),
            minimum_spacing_ns=int(self.declare_parameter('minimum_pair_spacing_ms',100).value)*1000000,
            maximum_gap_ns=int(self.declare_parameter('maximum_pair_gap_ms',600).value)*1000000)
        if (not isinstance(self.frame,str) or not self.frame or self.frame.startswith('/')
                or not 50000000 <= self.age_ns <= 5000000000):
            raise ValueError('invalid AV observation age or frame')
        self.audio_fence=StreamFence()
        self.vision_fence=StreamFence()
        self.audio=[]
        self.visual=[]
        # A TF2 transform can arrive after the DDS observation. Retain only
        # the newest admitted snapshot per stream, bounded by the age gate.
        self.pending_audio=None
        self.pending_visual=None
        self.audio_time=None
        self.vision_time=None
        self.had_pairs=False
        self.last_clock_ns=None
        self.out_sequence=0
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.pub=self.create_publisher(AudioVisualPairHypothesisArray,'fusion/av_pair_hypotheses',10)
        self.sub_audio=self.create_subscription(AcousticSourceArray,'sources',self.on_audio,10)
        self.sub_visual=self.create_subscription(
            VisualBearingObservationArray,'vision/bearings',self.on_visual,10)
        self.timer=self.create_timer(0.1,self.on_timer)

    def clock(self):
        now_ns=self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.audio_fence.reset()
            self.vision_fence.reset()
            self.audio=[]
            self.visual=[]
            self.pending_audio=None
            self.pending_visual=None
            self.audio_time=None
            self.vision_time=None
            self.temporal.reset()
            self.emit([])
        self.last_clock_ns=now_ns
        return now_ns

    def fresh(self,stamp,now_ns):
        t=ns(stamp)
        return t>0 and now_ns>0 and t<=now_ns+self.future_ns and now_ns-t<=self.age_ns

    def transform_ray(self,frame,stamp,vector):
        try:
            transform=self.tf_buffer.lookup_transform(
                self.frame,frame,Time.from_msg(stamp),timeout=Duration(seconds=0.0))
        except TransformException as exc:
            self.get_logger().warning(
                f'AV TF2 lookup rejected frame={frame!r} world={self.frame!r}: {exc}',
                throttle_duration_sec=2.0)
            return None
        origin=transform.transform.translation
        result=rotated(vector,transform.transform.rotation)
        origin_xyz=(origin.x,origin.y,origin.z)
        if (result is None or not all(math.isfinite(x) for x in origin_xyz)
                or abs(sum(x*x for x in result)-1)>0.001):
            return None
        return origin_xyz,result

    def observe(self,array_stamp,sample_stamp,now_ns):
        return (self.fresh(array_stamp,now_ns) and self.fresh(sample_stamp,now_ns)
                and ns(sample_stamp)<=ns(array_stamp))

    def revoke_audio(self):
        """Invalidate the audio side without discarding valid visual evidence."""
        self.audio=[]
        self.audio_time=None
        self.pending_audio=None
        self.temporal.reset()
        self.publish_pairs()

    def revoke_visual(self):
        """Invalidate the visual side without discarding valid audio evidence."""
        self.visual=[]
        self.vision_time=None
        self.pending_visual=None
        self.temporal.reset()
        self.publish_pairs()

    def on_audio(self,msg,retry=False):
        now_ns=self.clock()
        # A replayed or stale envelope is not an authoritative revocation:
        # reject it before touching a valid pending TF2 retry or live rays.
        if not self.fresh(msg.header.stamp,now_ns) or not msg.header.frame_id or msg.header.frame_id.startswith('/'):
            return
        if not retry and not self.audio_fence.accept(msg.scene_session_id,msg.sequence_number,ns(msg.header.stamp)):
            return
        if not retry:
            self.pending_audio=None
        if len(msg.sources)>self.matcher.maximum_tracks:
            # Over-capacity is invalid evidence, not a new empty observation.
            # Never carry confirmations across a rejected sensor snapshot.
            self.temporal.reset()
            self.audio=[]
            self.audio_time=time.monotonic()
            self.publish_pairs()
            return
        incoming=[]
        missing_tf=False
        invalid_candidate=False
        seen=set()
        for s in msg.sources:
            key=s.source_id
            if key in seen:
                invalid_candidate=True
                break
            seen.add(key)
            if s.state!=AcousticSource.ACTIVE:
                continue
            if (not s.has_direction
                    or not s.has_angular_variance or s.source_generation<=0
                    or (s.header.frame_id and s.header.frame_id!=msg.header.frame_id)
                    or not self.observe(msg.header.stamp,s.last_seen,now_ns)):
                invalid_candidate=True
                break
            transformed=self.transform_ray(msg.header.frame_id,s.last_seen,s.direction_unit)
            if transformed is None:
                missing_tf=True
            else:
                incoming.append(Ray(s.source_id,s.source_generation,ns(s.last_seen),
                                    transformed[0],transformed[1],s.angular_variance_rad2))
        if msg.sources and not incoming:
            self.get_logger().warning(
                f'AV audio snapshot rejected: tracks={len(msg.sources)} '
                f'frame={msg.header.frame_id!r} fresh={self.fresh(msg.header.stamp,now_ns)}',
                throttle_duration_sec=2.0)
        if invalid_candidate:
            # An invalid active competitor could otherwise make the remaining
            # association appear unique. Reject the entire sensor snapshot.
            incoming=[]
            self.temporal.reset()
        if missing_tf and not invalid_candidate:
            self.pending_audio=msg
            incoming=[]  # Never publish a misleading partial source snapshot.
        self.audio=incoming if len(msg.sources)<=self.matcher.maximum_tracks else []
        self.audio_time=time.monotonic()
        self.publish_pairs()

    def on_visual(self,msg,retry=False):
        now_ns=self.clock()
        if not self.fresh(msg.header.stamp,now_ns) or not msg.header.frame_id or msg.header.frame_id.startswith('/'):
            self.get_logger().warning(
                f'AV visual envelope rejected: fresh={self.fresh(msg.header.stamp,now_ns)} '
                f'frame={msg.header.frame_id!r} stamp={ns(msg.header.stamp)} now={now_ns}')
            return
        if not retry and not self.vision_fence.accept(msg.vision_session_id,msg.sequence_number,ns(msg.header.stamp)):
            self.get_logger().warning(
                f'AV visual fence rejected: session={msg.vision_session_id!r} '
                f'seq={msg.sequence_number} stamp={ns(msg.header.stamp)} '
                f'current_session={self.vision_fence.session!r} '
                f'current_seq={self.vision_fence.sequence}')
            return
        if not retry:
            self.pending_visual=None
        if len(msg.tracks)>self.matcher.maximum_tracks:
            # Over-capacity is invalid evidence, not a new empty observation.
            # Never carry confirmations across a rejected sensor snapshot.
            self.temporal.reset()
            self.visual=[]
            self.vision_time=time.monotonic()
            self.publish_pairs()
            return
        incoming=[]
        missing_tf=False
        invalid_candidate=False
        seen=set()
        for s in msg.tracks:
            key=s.visual_track_id
            if key in seen:
                invalid_candidate=True
                break
            seen.add(key)
            if not s.active:
                continue
            if (not s.activity_evidence or not s.has_angular_variance
                    or s.visual_track_generation<=0
                    or not self.observe(msg.header.stamp,s.observed_at,now_ns)):
                invalid_candidate=True
                break
            transformed=self.transform_ray(msg.header.frame_id,s.observed_at,s.direction_unit)
            if transformed is None:
                missing_tf=True
            else:
                incoming.append(Ray(s.visual_track_id,s.visual_track_generation,
                    ns(s.observed_at),transformed[0],transformed[1],
                    s.angular_variance_rad2,s.activity_evidence))
        if msg.tracks and not incoming:
            self.get_logger().warning(
                f'AV visual snapshot rejected: tracks={len(msg.tracks)} '
                f'frame={msg.header.frame_id!r} fresh={self.fresh(msg.header.stamp,now_ns)}',
                throttle_duration_sec=2.0)
        if invalid_candidate:
            incoming=[]
            self.temporal.reset()
        if missing_tf and not invalid_candidate:
            self.pending_visual=msg
            incoming=[]  # A missing ray can change association uniqueness.
        self.visual=incoming if len(msg.tracks)<=self.matcher.maximum_tracks else []
        self.get_logger().info(
            f'AV visual admitted: seq={msg.sequence_number} tracks={len(msg.tracks)} '
            f'rays={len(self.visual)} audio_rays={len(self.audio)}')
        self.vision_time=time.monotonic()
        self.publish_pairs()

    def publish_pairs(self):
        now_ns=self.get_clock().now().nanoseconds
        # Arrival-time TTL alone is insufficient: a snapshot may have been
        # admitted near the end of its capture-time freshness budget. Do not
        # emit a pair once either original observed timestamp is stale.
        expired = (self.audio_time is None or self.vision_time is None
                   or time.monotonic()-self.audio_time>self.age_ns/1e9
                   or time.monotonic()-self.vision_time>self.age_ns/1e9
                   or any(now_ns-r.stamp_ns>self.age_ns or
                          r.stamp_ns>now_ns+self.future_ns for r in self.audio)
                   or any(now_ns-r.stamp_ns>self.age_ns or
                          r.stamp_ns>now_ns+self.future_ns for r in self.visual))
        # An unresolved TF2 snapshot is not an empty observation. Never
        # associate against a partial sensor snapshot or advance temporal
        # confirmations until every admitted ray has been transformed.
        unresolved_tf = self.pending_audio is not None or self.pending_visual is not None
        # Missing sensor evidence cannot authorize even a diagnostic pair.
        # Do not invoke a matcher on a one-sided snapshot: an alternate
        # implementation might return a permissive or cached candidate.
        # Disjoint capture times cannot be promoted by a permissive or
        # cached matcher. Retain the evidence as pending until aligned.
        all_skewed = (bool(self.audio and self.visual) and
                      all(abs(a.stamp_ns-v.stamp_ns)>self.matcher.skew_ns
                          for a in self.audio for v in self.visual))
        pairs = ([] if expired or unresolved_tf or not self.audio or not self.visual
                 or all_skewed else self.matcher.propose(self.audio,self.visual))
        if not pairs and (self.audio or self.visual):
            self.get_logger().warning(
                f'AV no pairing: audio={len(self.audio)} visual={len(self.visual)} '
                f'expired={expired} audio_age='
                f'{None if self.audio_time is None else round(time.monotonic()-self.audio_time,3)} '
                f'visual_age='
                f'{None if self.vision_time is None else round(time.monotonic()-self.vision_time,3)} '
                f'audio_stamp={[a.stamp_ns for a in self.audio]} '
                f'visual_stamp={[v.stamp_ns for v in self.visual]}',
                throttle_duration_sec=1.0)
        # A pending DDS counterpart can preserve evidence only while BOTH
        # input snapshots remain fresh. Use all retained rays rather than
        # assuming the first pair represents a multi-track snapshot.
        pending = (not expired and (unresolved_tf or
                   (not pairs and self.audio and self.visual
                    and all(abs(a.stamp_ns-v.stamp_ns)>self.matcher.skew_ns
                            for a in self.audio for v in self.visual))))
        self.emit(pairs, pending=bool(pending))

    def emit(self,pairs,pending=False):
        confirmations = ({} if pending else self.temporal.observe(
            pairs,self.audio_fence.session,self.vision_fence.session,
            self.get_clock().now().nanoseconds))
        msg=AudioVisualPairHypothesisArray()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id=self.frame
        self.out_sequence+=1
        msg.sequence_number=self.out_sequence
        msg.audio_scene_session_id=self.audio_fence.session or ''
        msg.vision_session_id=self.vision_fence.session or ''
        for candidate in pairs:
            p=AudioVisualPairHypothesis()
            p.audio_scene_session_id=msg.audio_scene_session_id
            p.audio_source_id=candidate.audio.identity
            p.audio_source_generation=candidate.audio.generation
            p.vision_session_id=msg.vision_session_id
            p.visual_track_id=candidate.visual.identity
            p.visual_track_generation=candidate.visual.generation
            p.audio_observed_at.sec=candidate.audio.stamp_ns//1000000000
            p.audio_observed_at.nanosec=candidate.audio.stamp_ns%1000000000
            p.visual_observed_at.sec=candidate.visual.stamp_ns//1000000000
            p.visual_observed_at.nanosec=candidate.visual.stamp_ns%1000000000
            p.unique_match=candidate.unique_match
            evidence=confirmations.get(self.temporal.key(candidate))
            p.supporting_synchronized_observations=evidence.count if evidence else 0
            p.temporally_confirmed=bool(evidence and evidence.confirmed)
            p.angle_difference_rad=candidate.angle_rad
            p.normalized_residual=candidate.normalized_residual
            p.sensor_baseline_m=candidate.sensor_baseline_m
            msg.hypotheses.append(p)
        self.pub.publish(msg)
        self.had_pairs=bool(pairs)

    def on_timer(self):
        now_ns=self.clock()
        # Retry only already-admitted snapshots; never re-admit replayed
        # sequence numbers, never extend their original sensor timestamps.
        expired_pending=False
        if self.pending_audio is not None:
            msg=self.pending_audio
            self.pending_audio=None
            if self.fresh(msg.header.stamp,now_ns):
                self.on_audio(msg,retry=True)
            else:
                # A timed-out TF2 snapshot is a revoked observation, not
                # successful empty evidence. Do not carry pair confirmations.
                self.audio=[]
                self.audio_time=None
                expired_pending=True
        if self.pending_visual is not None:
            msg=self.pending_visual
            self.pending_visual=None
            if self.fresh(msg.header.stamp,now_ns):
                self.on_visual(msg,retry=True)
            else:
                self.visual=[]
                self.vision_time=None
                expired_pending=True
        if expired_pending:
            self.temporal.reset()
            # Explicitly revoke the last candidate snapshot even if the
            # opposite sensor remains fresh. No stale TF2 retry may revive it.
            if self.had_pairs:
                self.emit([])
        # An expired input must revoke its retained temporal evidence even
        # when the last published snapshot was empty (e.g. DDS skew). Do not
        # let a later observation inherit confirmations from stale evidence.
        expired = (self.audio_time is None or self.vision_time is None
                   or time.monotonic()-self.audio_time>self.age_ns/1e9
                   or time.monotonic()-self.vision_time>self.age_ns/1e9
                   or any(now_ns-r.stamp_ns>self.age_ns or
                          r.stamp_ns>now_ns+self.future_ns for r in self.audio)
                   or any(now_ns-r.stamp_ns>self.age_ns or
                          r.stamp_ns>now_ns+self.future_ns for r in self.visual))
        if expired and (self.audio or self.visual or self.had_pairs):
            # Retire only stale sides. A fresh opposite-side observation
            # must survive while TF2 catches up or a counterpart arrives.
            # Pending snapshots keep their original timestamps and are
            # independently checked by fresh() on every retry.
            now=time.monotonic()
            if (self.audio_time is None or now-self.audio_time>self.age_ns/1e9
                    or any(now_ns-r.stamp_ns>self.age_ns or
                           r.stamp_ns>now_ns+self.future_ns for r in self.audio)):
                self.audio=[]
                self.audio_time=None
            if (self.vision_time is None or now-self.vision_time>self.age_ns/1e9
                    or any(now_ns-r.stamp_ns>self.age_ns or
                           r.stamp_ns>now_ns+self.future_ns for r in self.visual)):
                self.visual=[]
                self.vision_time=None
            self.temporal.reset()
            # A timer transition must invalidate prior published candidates,
            # but must not discard the still-fresh counterpart.
            if self.had_pairs:
                self.emit([])


def main():
    rclpy.init()
    node=AvPairNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
