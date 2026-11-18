"""Bounded, epoch-fenced M4 pose sample cache, independent of ROS.

No interpolation or extrapolation. Only nearest already-admitted samples
within an explicit skew are returned. The caller must use one clock domain.
"""
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class PoseMatch:
    sample: object
    skew_ns: int


class PoseSampleCache:
    def __init__(self, *, capacity=32, maximum_skew_ns=50_000_000,
                 maximum_age_ns=500_000_000):
        if (type(capacity) is not int or not 1 <= capacity <= 256
                or type(maximum_skew_ns) is not int
                or not 0 <= maximum_skew_ns <= 1_000_000_000
                or type(maximum_age_ns) is not int
                or not maximum_skew_ns <= maximum_age_ns <= 10_000_000_000):
            raise ValueError('invalid pose cache bounds')
        self.capacity = capacity
        self.maximum_skew_ns = maximum_skew_ns
        self.maximum_age_ns = maximum_age_ns
        self.samples = deque(maxlen=capacity)
        self.last_stamp_ns = 0
        self.last_clock_ns = 0

    def reset(self):
        self.samples.clear()
        self.last_stamp_ns = 0
        self.last_clock_ns = 0

    def tick(self, clock_ns):
        if type(clock_ns) is not int or clock_ns <= 0:
            self.reset()
            return False
        if self.last_clock_ns and clock_ns < self.last_clock_ns:
            self.reset()
            self.last_clock_ns = clock_ns
            return False
        self.last_clock_ns = clock_ns
        while self.samples and (
                clock_ns < self.samples[0].stamp_ns
                or clock_ns - self.samples[0].stamp_ns > self.maximum_age_ns):
            self.samples.popleft()
        return True

    def add(self, sample, clock_ns):
        if not self.tick(clock_ns):
            return False
        try:
            stamp = sample.stamp_ns
            if (type(stamp) is not int or stamp <= self.last_stamp_ns
                    or stamp > clock_ns
                    or clock_ns - stamp > self.maximum_age_ns
                    or not sample.frame_id or sample.origin is None):
                return False
        except (AttributeError, TypeError):
            return False
        self.samples.append(sample)
        self.last_stamp_ns = stamp
        return True

    def match(self, sample_stamp_ns, clock_ns):
        if not self.tick(clock_ns) or type(sample_stamp_ns) is not int:
            return None
        if (sample_stamp_ns <= 0 or sample_stamp_ns > clock_ns
                or clock_ns - sample_stamp_ns > self.maximum_age_ns):
            return None
        matches = [(abs(s.stamp_ns - sample_stamp_ns), s)
                   for s in self.samples
                   if abs(s.stamp_ns - sample_stamp_ns) <= self.maximum_skew_ns]
        if not matches:
            return None
        skew, selected = min(matches, key=lambda item: item[0])
        return PoseMatch(selected, skew)
