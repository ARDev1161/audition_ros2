"""Conservative M4 audio/visual *hypothesis* gate, independent of ROS.

Inputs are independently timestamped, calibrated world-frame 3D bearing rays.
Only pair suggestions are produced: no speaker identity, depth, person
presence, confidence probability, covariance or navigation occupancy.
"""
from dataclasses import dataclass
import math
import re


@dataclass(frozen=True)
class Ray:
    identity: str
    generation: int
    stamp_ns: int
    origin: tuple
    direction: tuple
    angular_variance_rad2: float
    activity_evidence: bool = True


@dataclass(frozen=True)
class Pair:
    audio: Ray
    visual: Ray
    angle_rad: float
    normalized_residual: float
    sensor_baseline_m: float
    unique_match: bool


class StreamFence:
    """Snapshot admission with replay fencing across epochs and clock resets."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.session = None
        self.sequence = 0
        self.stamp_ns = 0

    def accept(self, session, sequence, stamp_ns):
        if (not isinstance(session, str)
                or not re.fullmatch(r'[0-9a-f]{32}', session)
                or sequence <= 0 or stamp_ns <= 0):
            return False
        # A different epoch may be newer, but never permit an older snapshot
        # to reopen a retired incarnation without an explicit clock reset.
        if self.session is not None and stamp_ns <= self.stamp_ns:
            return False
        if self.session == session and sequence <= self.sequence:
            return False
        self.session = session
        self.sequence = sequence
        self.stamp_ns = stamp_ns
        return True


class PairMatcher:
    def __init__(self, *, maximum_skew_ms=150, maximum_baseline_m=0.2,
                 maximum_angle_rad=0.30, maximum_variance_rad2=0.12,
                 minimum_assumed_range_m=1.0, normalized_limit=3.0,
                 uniqueness_margin=0.8, maximum_tracks=16):
        vals = (maximum_skew_ms,maximum_baseline_m,maximum_angle_rad,
                maximum_variance_rad2,minimum_assumed_range_m,
                normalized_limit,uniqueness_margin)
        if (not all(math.isfinite(v) for v in vals)
                or not 1 <= maximum_skew_ms <= 2000
                or not 0 <= maximum_baseline_m <= 2
                or not 0 < maximum_angle_rad <= math.pi/2
                or not 0 < maximum_variance_rad2 <= 1
                or not 0.1 <= minimum_assumed_range_m <= 20
                or not 0 < normalized_limit <= 10
                or not 0 <= uniqueness_margin <= 10
                or not 1 <= maximum_tracks <= 64):
            raise ValueError('invalid audio/visual association limits')
        self.skew_ns = int(maximum_skew_ms*1_000_000)
        self.maximum_baseline_m = maximum_baseline_m
        self.maximum_angle_rad = maximum_angle_rad
        self.maximum_variance_rad2 = maximum_variance_rad2
        self.minimum_assumed_range_m = minimum_assumed_range_m
        self.normalized_limit = normalized_limit
        self.uniqueness_margin = uniqueness_margin
        self.maximum_tracks = maximum_tracks

    def valid_ray(self, ray):
        if (not isinstance(ray.identity,str) or not ray.identity
                or len(ray.identity.encode('utf8')) > 256
                or ray.generation <= 0 or ray.stamp_ns <= 0
                or not ray.activity_evidence
                or len(ray.origin) != 3 or len(ray.direction) != 3
                or not all(math.isfinite(x) for x in
                           (*ray.origin,*ray.direction,ray.angular_variance_rad2))
                or not 0 < ray.angular_variance_rad2 <= self.maximum_variance_rad2):
            return False
        return abs(sum(d*d for d in ray.direction)-1.0) <= 1e-3

    def propose(self, audios, visuals):
        if len(audios) > self.maximum_tracks or len(visuals) > self.maximum_tracks:
            return []
        # Duplicated track identities are ambiguous producer input: fail closed.
        if len({(a.identity,a.generation) for a in audios}) != len(audios):
            return []
        if len({(v.identity,v.generation) for v in visuals}) != len(visuals):
            return []
        edges = []
        for a in audios:
            if not self.valid_ray(a):
                continue
            for v in visuals:
                if not self.valid_ray(v):
                    continue
                if abs(a.stamp_ns - v.stamp_ns) > self.skew_ns:
                    continue
                baseline = math.dist(a.origin,v.origin)
                if baseline > self.maximum_baseline_m:
                    continue
                dot = max(-1.0,min(1.0,sum(x*y for x,y in zip(a.direction,v.direction))))
                angle = math.acos(dot)
                if angle > self.maximum_angle_rad:
                    continue
                # Conservative parallax allowance for *assumed* minimum source
                # range: not a measured source range or calibrated covariance.
                spread = math.sqrt(a.angular_variance_rad2+v.angular_variance_rad2+
                                   (baseline/self.minimum_assumed_range_m)**2)
                residual = angle/spread
                if residual > self.normalized_limit:
                    continue
                edges.append((a,v,angle,residual,baseline))
        by_audio = {}
        by_visual = {}
        for edge in edges:
            by_audio.setdefault((edge[0].identity,edge[0].generation),[]).append(edge)
            by_visual.setdefault((edge[1].identity,edge[1].generation),[]).append(edge)
        for group in list(by_audio.values())+list(by_visual.values()):
            group.sort(key=lambda e:(e[3],e[2],e[0].identity,e[1].identity))
        def isolated_winner(group,edge):
            if group[0] is not edge:
                return False
            return len(group)==1 or group[1][3]-edge[3] >= self.uniqueness_margin
        output = []
        for edge in sorted(edges,key=lambda e:(e[0].identity,e[0].generation,e[3],
                                                e[1].identity,e[1].generation)):
            a,v,angle,residual,baseline = edge
            unique = (isolated_winner(by_audio[(a.identity,a.generation)],edge)
                      and isolated_winner(by_visual[(v.identity,v.generation)],edge))
            output.append(Pair(a,v,angle,residual,baseline,unique))
        return output
