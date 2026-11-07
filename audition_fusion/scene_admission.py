"""Canonical M4 scene incarnation/replay guard, independent of ROS.

All acoustic observations in a scene snapshot share the same incarnation.
A new session clears previous evidence; previously seen incarnations cannot
be reinstated within the bounded replay history. Clock rollback requires a
full reset, and never makes an old session eligible automatically.
"""
from collections import deque
from dataclasses import dataclass
import re


@dataclass(frozen=True)
class Admission:
    accepted: bool
    restarted: bool = False


class SceneAdmission:
    def __init__(self, *, retired_capacity=16):
        if type(retired_capacity) is not int or not 1 <= retired_capacity <= 128:
            raise ValueError('invalid retired session bound')
        self.retired = deque(maxlen=retired_capacity)
        self.session = None
        self.sequence = 0
        self.stamp_ns = 0

    def reset(self):
        if self.session is not None and self.session not in self.retired:
            self.retired.append(self.session)
        self.session = None
        self.sequence = 0
        self.stamp_ns = 0

    def begin(self, session, sequence, stamp_ns):
        if (not isinstance(session, str)
                or not re.fullmatch(r'[0-9a-f]{32}', session)
                or type(sequence) is not int or sequence <= 0
                or type(stamp_ns) is not int or stamp_ns <= 0):
            return Admission(False)
        if self.session == session:
            if sequence <= self.sequence or stamp_ns <= self.stamp_ns:
                return Admission(False)
            self.sequence, self.stamp_ns = sequence, stamp_ns
            return Admission(True)
        if session in self.retired:
            return Admission(False)
        restarted = self.session is not None
        self.reset()
        self.session = session
        self.sequence, self.stamp_ns = sequence, stamp_ns
        return Admission(True, restarted)
