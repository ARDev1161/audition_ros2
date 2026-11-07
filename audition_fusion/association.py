"""Strict M4 source-session association for *diagnostic* 2-D intersections.

This module accepts already timestamped and TF-transformed world bearings.
It never infers source identity, height, covariance or a physical obstacle.
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import math
import re
from geometry import triangulate_2d


@dataclass(frozen=True)
class Observation:
    source_id: str
    source_generation: int
    stamp_ns: int
    origin: tuple
    direction: tuple


@dataclass(frozen=True)
class Candidate:
    source_id: str
    source_generation: int
    first_stamp_ns: int
    last_stamp_ns: int
    x: float
    y: float
    baseline_m: float
    parallax_rad: float
    worst_residual_m: float
    observations: int


class CandidateTracker:
    def __init__(self, *, minimum_baseline=0.5, minimum_sine=0.15,
                 maximum_residual=0.25, minimum_third_baseline=0.15,
                 window_ns=5_000_000_000, minimum_spacing_ns=100_000_000,
                 max_tracks=16, max_observations=8):
        if (not all(math.isfinite(x) for x in (minimum_baseline, minimum_sine,
                maximum_residual, minimum_third_baseline))
                or minimum_baseline <= 0 or not 0 < minimum_sine < 1
                or maximum_residual <= 0 or minimum_third_baseline <= 0
                or window_ns <= 0 or minimum_spacing_ns <= 0
                or not 2 <= max_tracks <= 64 or not 3 <= max_observations <= 32):
            raise ValueError('invalid M4 association limits')
        self.minimum_baseline = minimum_baseline
        self.minimum_sine = minimum_sine
        self.maximum_residual = maximum_residual
        self.minimum_third_baseline = minimum_third_baseline
        self.window_ns = window_ns
        self.minimum_spacing_ns = minimum_spacing_ns
        self.max_tracks = max_tracks
        self.max_observations = max_observations
        self.session = None
        self.last_sequence = 0
        self.last_scene_stamp_ns = 0
        self.tracks = OrderedDict()

    def begin_scene(self, session, sequence, stamp_ns):
        # A canonical source stream has a scene incarnation, strictly increasing
        # sequence and nonzero stamps. A restart clears all previous pairings.
        if (not re.fullmatch(r'[0-9a-f]{32}', session or '')
                or sequence <= 0 or stamp_ns <= 0):
            return False
        if self.session != session:
            self.session = session
            self.tracks.clear()
            self.last_sequence = 0
            self.last_scene_stamp_ns = 0
        if sequence <= self.last_sequence or stamp_ns <= self.last_scene_stamp_ns:
            return False
        self.last_sequence = sequence
        self.last_scene_stamp_ns = stamp_ns
        return True

    def observe(self, item):
        if (self.session is None or not item.source_id
                or len(item.source_id.encode('utf8')) > 256
                or item.source_generation <= 0 or item.stamp_ns <= 0
                or item.stamp_ns > self.last_scene_stamp_ns
                or self.last_scene_stamp_ns - item.stamp_ns > self.window_ns):
            return None
        vals = (*item.origin, *item.direction)
        if (not all(math.isfinite(x) for x in vals)
                or abs(math.hypot(*item.direction) - 1) > 1e-3):
            return None
        key = (item.source_id, item.source_generation)
        # Never combine different generations for the same source ID.
        for old in list(self.tracks):
            if old[0] == item.source_id and old != key:
                del self.tracks[old]
        history = self.tracks.get(key)
        if history is None:
            if len(self.tracks) >= self.max_tracks:
                self.tracks.popitem(last=False)
            history = deque(maxlen=self.max_observations)
            self.tracks[key] = history
        else:
            self.tracks.move_to_end(key)
        if history and item.stamp_ns - history[-1].stamp_ns < self.minimum_spacing_ns:
            return None
        history.append(item)
        while history and item.stamp_ns - history[0].stamp_ns > self.window_ns:
            history.popleft()
        return self._candidate(key, history)

    def retire_missing(self, active):
        # Source snapshots are authoritative; an absent or retired track is not
        # evidence of physical disappearance, only loss of usable association.
        for key in list(self.tracks):
            if key not in active:
                del self.tracks[key]

    def expire(self, now_ns):
        for key in list(self.tracks):
            history = self.tracks[key]
            if not history or now_ns < history[-1].stamp_ns or now_ns - history[-1].stamp_ns > self.window_ns:
                del self.tracks[key]

    def _candidate(self, key, history):
        if len(history) < 3:
            return None
        best = None
        for i in range(len(history) - 1):
            a = history[i]
            for j in range(i + 1, len(history)):
                b = history[j]
                baseline = math.dist(a.origin, b.origin)
                if baseline < self.minimum_baseline:
                    continue
                cross = a.direction[0]*b.direction[1]-a.direction[1]*b.direction[0]
                sine = abs(cross)
                if sine < self.minimum_sine:
                    continue
                point = triangulate_2d(
                    a.origin, a.direction, b.origin, b.direction,
                    minimum_baseline=self.minimum_baseline,
                    minimum_sine=self.minimum_sine,
                    maximum_ray_gap=self.maximum_residual)
                if point is None:
                    continue
                third = [x for k,x in enumerate(history) if k != i and k != j
                         and min(math.dist(x.origin, a.origin),
                                 math.dist(x.origin, b.origin)) >= self.minimum_third_baseline]
                if not third:
                    continue
                worst = 0.0
                valid = True
                for x in history:
                    delta = (point[0]-x.origin[0], point[1]-x.origin[1])
                    if delta[0]*x.direction[0]+delta[1]*x.direction[1] <= 0:
                        valid = False
                        break
                    worst = max(worst, abs(delta[0]*x.direction[1]-delta[1]*x.direction[0]))
                if not valid or worst > self.maximum_residual:
                    continue
                score = sine*baseline
                if best is None or score > best[0]:
                    best = (score, Candidate(key[0],key[1],
                        history[0].stamp_ns, history[-1].stamp_ns,
                        point[0],point[1], baseline, math.asin(sine),
                        worst, len(history)))
        return best[1] if best else None
