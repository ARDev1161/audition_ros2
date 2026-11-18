"""M4 radar/audio *diagnostic candidates*, not acoustic emitter localization.

Matches an independently observed world-frame radar position to a world
audio bearing. The result is a plausible association only; no source range,
speaker identity, covariance fusion or Nav2 obstacle may be inferred.
"""
from dataclasses import dataclass
import math

from radar_geometry import rotate_covariance


@dataclass(frozen=True)
class RadarPoint:
    identity: str
    generation: int
    stamp_ns: int
    position: tuple
    covariance: tuple


@dataclass(frozen=True)
class RadarBearingCandidate:
    audio_id: str
    audio_generation: int
    radar_id: str
    radar_generation: int
    range_along_ray_m: float
    lateral_residual_m: float
    normalized_residual: float
    unique_match: bool


class RadarBearingMatcher:
    def __init__(self, *, maximum_skew_ms=150, maximum_angle_rad=.35,
                 maximum_normalized_residual=3., minimum_range_m=.5,
                 maximum_range_m=30., maximum_tracks=16,
                 uniqueness_margin=.8):
        vals=(maximum_skew_ms,maximum_angle_rad,
              maximum_normalized_residual,minimum_range_m,
              maximum_range_m,uniqueness_margin)
        if (not all(math.isfinite(v) for v in vals)
                or not 1 <= maximum_skew_ms <= 2000
                or not 0 < maximum_angle_rad <= math.pi/2
                or not 0 < maximum_normalized_residual <= 10
                or not 0 < minimum_range_m < maximum_range_m <= 200
                or not 1 <= maximum_tracks <= 64
                or not 0 <= uniqueness_margin <= 10):
            raise ValueError('invalid radar association limits')
        self.skew_ns=int(maximum_skew_ms*1_000_000)
        self.angle=maximum_angle_rad
        self.residual=maximum_normalized_residual
        self.min_range=minimum_range_m
        self.max_range=maximum_range_m
        self.max_tracks=maximum_tracks
        self.margin=uniqueness_margin

    def propose(self, audio_rays, radar_points):
        if len(audio_rays)>self.max_tracks or len(radar_points)>self.max_tracks:
            return []
        if (len({x.identity for x in audio_rays}) != len(audio_rays)
                or len({x.identity for x in radar_points}) != len(radar_points)):
            return []
        edges=[]
        for a in audio_rays:
            if (not a.identity or a.generation<=0 or a.stamp_ns<=0
                    or not a.activity_evidence
                    or len(a.origin)!=3 or len(a.direction)!=3
                    or not all(math.isfinite(x) for x in
                               (*a.origin,*a.direction,a.angular_variance_rad2))
                    or not 0<a.angular_variance_rad2<=1
                    or abs(sum(x*x for x in a.direction)-1)>1e-3):
                continue
            for r in radar_points:
                if (not r.identity or r.generation<=0 or r.stamp_ns<=0
                        or abs(r.stamp_ns-a.stamp_ns)>self.skew_ns
                        or len(r.position)!=3
                        or not all(math.isfinite(x) for x in r.position)
                        or rotate_covariance(r.covariance,
                           ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))) is None):
                    continue
                delta=tuple(r.position[i]-a.origin[i] for i in range(3))
                dist=math.sqrt(sum(x*x for x in delta))
                axial=sum(delta[i]*a.direction[i] for i in range(3))
                if not self.min_range<=axial<=self.max_range:
                    continue
                if dist==0:
                    continue
                angle=math.acos(max(-1.,min(1.,axial/dist)))
                if angle>self.angle:
                    continue
                lateral=math.sqrt(max(0.,dist*dist-axial*axial))
                # Conservative *diagnostic* uncertainty bound: angular
                # variance at range + trace of radar position covariance.
                # Not a calibrated joint innovation or physical confidence.
                var=dist*dist*a.angular_variance_rad2+sum(
                    r.covariance[4*i] for i in range(3))
                if not math.isfinite(var) or var<=0:
                    continue
                residual=lateral/math.sqrt(var)
                if residual>self.residual:
                    continue
                edges.append((a,r,axial,lateral,residual))
        by_audio={}
        by_radar={}
        for e in edges:
            by_audio.setdefault(e[0].identity,[]).append(e)
            by_radar.setdefault(e[1].identity,[]).append(e)
        for values in (*by_audio.values(),*by_radar.values()):
            values.sort(key=lambda e:(e[4],e[3],e[0].identity,e[1].identity))

        def isolated(group,edge):
            return (group[0] is edge
                    and (len(group)==1
                         or group[1][4]-edge[4]>=self.margin))

        result=[]
        for edge in sorted(edges,key=lambda e:(
                e[0].identity,e[4],e[1].identity)):
            a,r,axial,lateral,residual=edge
            result.append(RadarBearingCandidate(
                a.identity,a.generation,r.identity,r.generation,
                axial,lateral,residual,
                isolated(by_audio[a.identity],edge)
                and isolated(by_radar[r.identity],edge)))
        return result
