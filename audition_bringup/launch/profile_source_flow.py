"""Spatial source PCM progress watchdog, independent of ROS.

Live mono ALSA must emit source PCM while capture is flowing. ODAS and
finite WAV are observed but never faulted for legitimate source silence/EOF.
"""
from dataclasses import dataclass
import re
import threading
import time


_SESSION = re.compile(r"[0-9a-f]{32}")


@dataclass(frozen=True)
class SourceFlowSnapshot:
    enabled: bool
    healthy: bool
    timed_out: bool
    accepted_frames: int
    age_ms: int
    detail: str


class SourceFlowMonitor:
    def __init__(self, profile_mode, clock=None, start_grace_s=10.0,
                 stall_timeout_s=5.0, max_sessions=64, max_sources=256):
        if (start_grace_s <= 0 or stall_timeout_s <= 0 or
                max_sessions < 1 or max_sources < 1):
            raise ValueError("source watchdog limits must be positive")
        self.profile_mode = profile_mode
        self.clock = clock or time.monotonic
        self.start_grace_s = start_grace_s
        self.stall_timeout_s = stall_timeout_s
        self.max_sessions = max_sessions
        self.max_sources = max_sources
        self._lock = threading.Lock()
        self._retired = set()
        self._session = None
        self._cursors = {}
        self._blocked = False
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
        self._cursors.clear()

    def reset(self, spec):
        mode = self.profile_mode(spec) if spec is not None else None
        with self._lock:
            self._retire_current()
            self._started = self.clock() if mode else None
            self._last_seen = None
            self._enabled = mode == ("alsa", "mono")
            self._accepted = 0

    def observe(self, session, source_id, generation, sequence,
                sample_rate_hz, frame_count, pcm_length):
        if (not isinstance(session, str) or not _SESSION.fullmatch(session) or
                not isinstance(source_id, str) or not source_id or
                type(generation) is not int or generation < 1 or
                type(sequence) is not int or sequence < 0 or
                type(sample_rate_hz) is not int or
                not 8000 <= sample_rate_hz <= 192000 or
                type(frame_count) is not int or not 1 <= frame_count <= 4096 or
                pcm_length != frame_count):
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
            key = (source_id, generation)
            previous = self._cursors.get(key)
            if previous is not None and sequence <= previous:
                return False
            if previous is None and len(self._cursors) >= self.max_sources:
                self._blocked = True
                return False
            self._cursors[key] = sequence
            self._accepted += 1
            self._last_seen = self.clock()
            return True

    def snapshot(self):
        with self._lock:
            if self._started is None:
                return SourceFlowSnapshot(False, False, False, 0, 0, "offline")
            now = self.clock()
            age = max(0.0, now - self._last_seen) if self._last_seen is not None else 0.0
            fresh = self._last_seen is not None and age <= self.stall_timeout_s
            timed_out = self._enabled and (
                self._blocked or
                (self._last_seen is None and now - self._started > self.start_grace_s) or
                (self._last_seen is not None and age > self.stall_timeout_s))
            detail = ("spatial session/source cursor limit exceeded" if self._blocked else
                      "no source PCM since launch" if self._last_seen is None else
                      "last accepted source PCM %.1fs ago" % age)
            return SourceFlowSnapshot(self._enabled, fresh, timed_out,
                                      self._accepted, int(age * 1000), detail)
