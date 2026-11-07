"""Capture PCM progress watchdog, independent of ROS and injectable in tests.

Only ALSA streams are required to remain live. Finite WAV replay is observed
but cannot trigger a stall fault after its expected EOF.
"""
from dataclasses import dataclass
import re
import threading
import time


_SESSION = re.compile(r"[0-9a-f]{32}")


@dataclass(frozen=True)
class AudioFlowSnapshot:
    enabled: bool
    healthy: bool
    timed_out: bool
    accepted_frames: int
    age_ms: int
    detail: str


class AudioFlowMonitor:
    def __init__(self, capture_kind, clock=None, start_grace_s=10.0,
                 stall_timeout_s=5.0, max_sessions=64):
        if start_grace_s <= 0 or stall_timeout_s <= 0 or max_sessions < 1:
            raise ValueError("flow watchdog limits must be positive")
        self.capture_kind = capture_kind
        self.clock = clock or time.monotonic
        self.start_grace_s = start_grace_s
        self.stall_timeout_s = stall_timeout_s
        self.max_sessions = max_sessions
        self._lock = threading.Lock()
        self._retired = set()
        self._blocked = False
        self._session = None
        self._sequence = -1
        self._started = None
        self._last_seen = None
        self._enabled = False
        self._accepted = 0

    def _retire_current(self):
        if self._session is not None:
            if len(self._retired) >= self.max_sessions:
                self._blocked = True
            else:
                self._retired.add(self._session)
        self._session = None
        self._sequence = -1

    def reset(self, spec):
        # Keep retired IDs across profile switches; delayed callbacks from the
        # previous launch must not count as progress for the replacement.
        kind = self.capture_kind(spec) if spec is not None else None
        with self._lock:
            self._retire_current()
            self._started = self.clock() if kind else None
            self._last_seen = None
            self._enabled = kind == "alsa"
            self._accepted = 0

    def observe(self, session, sequence, frame_count, channel_count, pcm_length):
        if (not isinstance(session, str) or not _SESSION.fullmatch(session) or
                type(sequence) is not int or sequence < 0 or
                type(frame_count) is not int or not 1 <= frame_count <= 4096 or
                type(channel_count) is not int or not 1 <= channel_count <= 32 or
                pcm_length != frame_count * channel_count):
            return False
        with self._lock:
            if (self._started is None or self._blocked or
                    session in self._retired):
                return False
            if session != self._session:
                self._retire_current()
                if self._blocked:
                    return False
                self._session = session
            if sequence <= self._sequence:
                return False
            self._sequence = sequence
            self._accepted += 1
            self._last_seen = self.clock()
            return True

    def snapshot(self):
        with self._lock:
            if self._started is None:
                return AudioFlowSnapshot(False, False, False, 0, 0, "offline")
            now = self.clock()
            age = max(0.0, now - self._last_seen) if self._last_seen is not None else 0.0
            fresh = self._last_seen is not None and age <= self.stall_timeout_s
            timed_out = self._enabled and (
                self._blocked or
                (self._last_seen is None and now - self._started > self.start_grace_s) or
                (self._last_seen is not None and age > self.stall_timeout_s))
            detail = ("retired capture session limit exceeded" if self._blocked else
                      "no capture PCM since launch" if self._last_seen is None else
                      "last accepted PCM %.1fs ago" % age)
            return AudioFlowSnapshot(self._enabled, fresh, timed_out,
                                     self._accepted, int(age * 1000), detail)
