"""Bounded M4 diagnostic-only six-ray admission, never metric publication.

Six nonreused chronological rays form two three-ray windows. On six rays
the existing conditional covariance + stationarity gate is evaluated;
both success and failure clear the series. This makes conservative,
one-shot counters possible without emitting coordinates or navigating.
"""
from collections import OrderedDict
from dataclasses import dataclass

from diagnostic_pipeline import evaluate_disjoint_windows
from uncertainty import PlanarRay


@dataclass(frozen=True)
class EvidenceResult:
    evaluated: bool
    admitted: bool


class DiagnosticEvidenceBuffer:
    def __init__(self, *, max_sources=16, maximum_window_ns=5_000_000_000,
                 minimum_spacing_ns=100_000_000):
        if (type(max_sources) is not int or not 1 <= max_sources <= 64
                or type(maximum_window_ns) is not int
                or maximum_window_ns <= 0
                or type(minimum_spacing_ns) is not int
                or minimum_spacing_ns < 100_000_000):
            raise ValueError('invalid evidence buffer bounds')
        self.max_sources = max_sources
        self.maximum_window_ns = maximum_window_ns
        self.minimum_spacing_ns = minimum_spacing_ns
        self.tracks = OrderedDict()

    def reset(self):
        self.tracks.clear()

    def retire_missing(self, active):
        for key in list(self.tracks):
            if key not in active:
                del self.tracks[key]

    def add(self, session, source_id, generation, ray):
        if (not isinstance(session, str) or not isinstance(source_id, str)
                or not source_id or type(generation) is not int
                or generation <= 0 or not isinstance(ray, PlanarRay)
                or type(ray.stamp_ns) is not int or ray.stamp_ns <= 0):
            return EvidenceResult(False, False)
        key = (session, source_id, generation)
        for previous in list(self.tracks):
            if previous[1] == source_id and previous != key:
                del self.tracks[previous]
        sequence = self.tracks.get(key)
        if sequence is None:
            if len(self.tracks) >= self.max_sources:
                self.tracks.popitem(last=False)
            sequence = []
            self.tracks[key] = sequence
        self.tracks.move_to_end(key)
        if sequence and ray.stamp_ns - sequence[-1].stamp_ns < self.minimum_spacing_ns:
            return EvidenceResult(False, False)
        if sequence and ray.stamp_ns - sequence[0].stamp_ns > self.maximum_window_ns:
            sequence.clear()
        sequence.append(ray)
        if len(sequence) < 6:
            return EvidenceResult(False, False)
        # The two sub-windows must not overlap; no interpolation or reuse.
        consensus = evaluate_disjoint_windows(
            session, source_id, generation, tuple(sequence[:3]),
            tuple(sequence[3:6]))
        del self.tracks[key]
        return EvidenceResult(True, consensus is not None)
