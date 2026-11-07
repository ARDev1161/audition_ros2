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
            maximum_skew_ms=float(self.declare_parameter('maximum_skew_ms',150).value),
            maximum_baseline_m=float(self.declare_parameter('maximum_sensor_separation_m',0.2).value),
            maximum_angle_rad=float(self.declare_parameter('maximum_pair_angle_rad',0.3).value),
            minimum_assumed_range_m=float(self.declare_parameter('minimum_assumed_range_m',1.0).value))
        if (not isinstance(self.frame,str) or not self.frame or self.frame.startswith('/')
                or not 50000000 <= self.age_ns <= 5000000000):
            raise ValueError('invalid AV observation age or frame')
        self.audio_fence=StreamFence()
        self.vision_fence=StreamFence()
        self.audio=[]
        self.visual=[]
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
            self.audio_time=None
            self.vision_time=None
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
        except TransformException:
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

    def on_audio(self,msg):
        now_ns=self.clock()
        if not self.fresh(msg.header.stamp,now_ns) or not msg.header.frame_id or msg.header.frame_id.startswith('/'):
            return
        if not self.audio_fence.accept(msg.scene_session_id,msg.sequence_number,ns(msg.header.stamp)):
            return
        if len(msg.sources)>self.matcher.maximum_tracks:
            self.audio=[]
            self.audio_time=time.monotonic()
            self.publish_pairs()
            return
        incoming=[]
        seen=set()
        for s in msg.sources:
            key=(s.source_id,s.source_generation)
            if key in seen:
                incoming=[]
                break
            seen.add(key)
            if (s.state!=AcousticSource.ACTIVE or not s.has_direction
                    or not s.has_angular_variance or s.source_generation<=0
                    or (s.header.frame_id and s.header.frame_id!=msg.header.frame_id)
                    or not self.observe(msg.header.stamp,s.last_seen,now_ns)):
                continue
            transformed=self.transform_ray(msg.header.frame_id,s.last_seen,s.direction_unit)
            if transformed is not None:
                incoming.append(Ray(s.source_id,s.source_generation,ns(s.last_seen),
                                    transformed[0],transformed[1],s.angular_variance_rad2))
        self.audio=incoming if len(msg.sources)<=self.matcher.maximum_tracks else []
        self.audio_time=time.monotonic()
        self.publish_pairs()

    def on_visual(self,msg):
        now_ns=self.clock()
        if not self.fresh(msg.header.stamp,now_ns) or not msg.header.frame_id or msg.header.frame_id.startswith('/'):
            return
        if not self.vision_fence.accept(msg.vision_session_id,msg.sequence_number,ns(msg.header.stamp)):
            return
        if len(msg.tracks)>self.matcher.maximum_tracks:
            self.visual=[]
            self.vision_time=time.monotonic()
            self.publish_pairs()
            return
        incoming=[]
        seen=set()
        for s in msg.tracks:
            key=(s.visual_track_id,s.visual_track_generation)
            if key in seen:
                incoming=[]
                break
            seen.add(key)
            if (not s.active or not s.activity_evidence or not s.has_angular_variance
                    or s.visual_track_generation<=0
                    or not self.observe(msg.header.stamp,s.observed_at,now_ns)):
                continue
            transformed=self.transform_ray(msg.header.frame_id,s.observed_at,s.direction_unit)
            if transformed is not None:
                incoming.append(Ray(s.visual_track_id,s.visual_track_generation,
                    ns(s.observed_at),transformed[0],transformed[1],
                    s.angular_variance_rad2,s.activity_evidence))
        self.visual=incoming if len(msg.tracks)<=self.matcher.maximum_tracks else []
        self.vision_time=time.monotonic()
        self.publish_pairs()

    def publish_pairs(self):
        if (self.audio_time is None or self.vision_time is None
                or time.monotonic()-self.audio_time>self.age_ns/1e9
                or time.monotonic()-self.vision_time>self.age_ns/1e9):
            pairs=[]
        else:
            pairs=self.matcher.propose(self.audio,self.visual)
        self.emit(pairs)

    def emit(self,pairs):
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
            p.angle_difference_rad=candidate.angle_rad
            p.normalized_residual=candidate.normalized_residual
            p.sensor_baseline_m=candidate.sensor_baseline_m
            msg.hypotheses.append(p)
        self.pub.publish(msg)
        self.had_pairs=bool(pairs)

    def on_timer(self):
        self.clock()
        if (self.had_pairs and
                (self.audio_time is None or self.vision_time is None
                 or time.monotonic()-self.audio_time>self.age_ns/1e9
                 or time.monotonic()-self.vision_time>self.age_ns/1e9)):
            self.audio=[]
            self.visual=[]
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
