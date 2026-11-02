"""Process-owned pipeline supervisor with graph and PCM-flow fault containment."""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class ProfileSpec:
    profile: str
    namespace: str = "audition"
    wav_path: str = ""
    device: str = ""
    models_dir: str = ""
    frame_id: str = ""

    def overrides(self):
        return {k: v for k, v in (
            ("wav_path", self.wav_path), ("device", self.device),
            ("models_dir", self.models_dir), ("frame_id", self.frame_id)) if v}


@dataclass(frozen=True)
class SwitchResult:
    success: bool
    rollback_succeeded: bool
    reload_epoch: int
    active_profile: str
    message: str


@dataclass(frozen=True)
class SupervisorStatus:
    running: bool
    owned_process_present: bool
    active_profile: str
    last_good_profile: str
    reload_epoch: int
    fault: str
    graph_healthy: bool
    graph_miss_count: int
    audio_watchdog_enabled: bool
    audio_healthy: bool
    audio_miss_count: int
    accepted_audio_frames: int
    last_audio_age_ms: int


class PipelineSupervisor:
    """Injectable state machine: validate(spec), spawn(spec), probe.ready/gone/vacant."""
    def __init__(self, validate, spawn, probe, timeout=30,
                 graph_failure_threshold=5, health_check_interval=1.0, clock=None,
                 audio_flow=None, audio_failure_threshold=5):
        if (timeout <= 0 or graph_failure_threshold < 1 or
                audio_failure_threshold < 1 or health_check_interval <= 0):
            raise ValueError("timeouts and graph failure threshold must be positive")
        self.validate, self.spawn, self.probe = validate, spawn, probe
        self.timeout = timeout
        self.graph_failure_threshold = graph_failure_threshold
        self.health_check_interval = health_check_interval
        self.clock = clock or time.monotonic
        self._last_health_check = None
        self._graph_healthy = False
        self._graph_misses = 0
        self.audio_flow = audio_flow
        self.audio_failure_threshold = audio_failure_threshold
        self._audio_misses = 0
        self.process = None
        self.active = None
        self.last_good = None
        self.epoch = 0
        self.fault = ""
        self._lock = threading.Lock()

    def _result(self, ok, rollback, message):
        return SwitchResult(ok, rollback, self.epoch,
                            self.active.profile if self.active else "", message)

    def _status(self):
        audio = self.audio_flow.snapshot() if self.audio_flow is not None else None
        return SupervisorStatus(self.active is not None and self.process is not None
                                and self.process.poll() is None,
                                self.process is not None,
                                self.active.profile if self.active else "",
                                self.last_good.profile if self.last_good else "",
                                self.epoch, self.fault,
                                self._graph_healthy, self._graph_misses,
                                audio.enabled if audio else False,
                                audio.healthy if audio else False,
                                self._audio_misses,
                                audio.accepted_frames if audio else 0,
                                audio.age_ms if audio else 0)

    def status(self):
        with self._lock:
            return self._status()

    def _stop(self):
        # Retain the owned handle on failed cleanup: never allow a second
        # pipeline to start over surviving orphan workers.
        if self.process is not None:
            try:
                self.process.stop()
            except Exception:
                self.active = None
                self._graph_healthy = False
                if self.audio_flow is not None:
                    self.audio_flow.reset(None)
                self._audio_misses = 0
                raise
            self.process = None
        self.active = None
        self._graph_healthy = False
        self._graph_misses = 0
        self._audio_misses = 0
        if self.audio_flow is not None:
            self.audio_flow.reset(None)
        self._last_health_check = None

    def _launch(self, spec):
        process = self.spawn(spec)
        self.process, self.active = process, None
        try:
            if not self.probe.ready(spec, process, self.timeout):
                raise RuntimeError("pipeline nodes/capability service did not become ready")
            if self.audio_flow is not None:
                self.audio_flow.reset(spec)
        except Exception:
            self._stop()
            raise
        self.active = spec
        self.last_good = spec
        self._graph_healthy = True
        self._graph_misses = 0
        self._audio_misses = 0
        self._last_health_check = self.clock()
        self.fault = ""

    def start(self, spec):
        with self._lock:
            if self.process is not None:
                raise RuntimeError("supervisor already owns a pipeline")
            self.validate(spec)
            if not self.probe.vacant(spec):
                raise RuntimeError("namespace already contains pipeline nodes; refusing takeover")
            self._launch(spec)

    def _restore(self, previous, cause):
        try:
            if not self.probe.gone(previous, self.timeout):
                raise RuntimeError("previous/candidate ROS graph did not disappear")
            self.validate(previous)
            self._launch(previous)
            return self._result(False, True, cause + "; previous profile restored")
        except Exception as error:
            try:
                self._stop()
            except Exception as cleanup_error:
                self.fault = "rollback cleanup failed: " + str(cleanup_error)
                return self._result(False, False, cause + "; " + self.fault)
            self.fault = "ROLLBACK FAILED: " + str(error)
            return self._result(False, False, cause + "; " + self.fault)

    def monitor(self):
        """Check owned process and live ROS graph; contain sustained failures.

        Time-based sampling prevents frequent status requests from accelerating
        the failure threshold. A transient DDS discovery miss is not fatal.
        """
        if not self._lock.acquire(blocking=False):
            return
        try:
            if self.process is None:
                return
            previous = self.active
            if self.process.poll() is not None:
                cause = "owned launch process exited unexpectedly"
            else:
                if previous is None:
                    return  # Cleanup failed; ownership must be resolved first.
                now = self.clock()
                if (self._last_health_check is not None and
                        now - self._last_health_check < self.health_check_interval):
                    return
                self._last_health_check = now
                try:
                    healthy, detail = self.probe.health(previous)
                except Exception as error:
                    healthy, detail = False, "graph probe error: " + str(error)
                if healthy:
                    self._graph_healthy = True
                    self._graph_misses = 0
                    if self.audio_flow is None:
                        return
                    audio = self.audio_flow.snapshot()
                    if not audio.timed_out:
                        self._audio_misses = 0
                        return
                    self._audio_misses += 1
                    if self._audio_misses < self.audio_failure_threshold:
                        return
                    cause = ("capture PCM stalled for " + str(self._audio_misses) +
                             " consecutive checks: " + audio.detail)
                else:
                    self._graph_healthy = False
                    self._graph_misses += 1
                    self._audio_misses = 0
                    if self._graph_misses < self.graph_failure_threshold:
                        return
                    cause = ("ROS graph unhealthy for " + str(self._graph_misses) +
                             " consecutive checks: " + detail)
            self.fault = cause
            try:
                self._stop()
                if previous is not None and not self.probe.gone(previous, self.timeout):
                    self.fault += "; ROS graph still present (manual cleanup required)"
            except Exception as error:
                self.active = None
                self._graph_healthy = False
                self.fault += "; cleanup failed: " + str(error)
        finally:
            self._lock.release()

    def switch(self, candidate, dry_run=False):
        if not self._lock.acquire(blocking=False):
            return self._result(False, False, "another switch is in progress")
        try:
            if self.process is not None and self.process.poll() is not None:
                return self._result(False, False,
                                    "owned pipeline exited; wait for monitor cleanup")
            if self.process is not None and self.active is None:
                return self._result(False, False,
                                    "cleanup incomplete; owned process handle retained")
            recovering = self.active is None
            if recovering and self.last_good is None:
                return self._result(False, False, "no previously healthy profile to recover")
            try:
                self.validate(candidate)
            except Exception as error:
                return self._result(False, False, "preflight rejected: " + str(error))
            if dry_run:
                return self._result(True, False, "static preflight passed; no processes changed")
            if recovering:
                try:
                    if not self.probe.gone(candidate, self.timeout):
                        raise RuntimeError("stale pipeline ROS graph still present")
                    self._launch(candidate)
                    self.epoch += 1
                    return self._result(True, False, "pipeline manually recovered and ready")
                except Exception as error:
                    self.fault = "recovery failed: " + str(error)
                    return self._result(False, False, self.fault)
            previous = self.active
            try:
                self._stop()
            except Exception as error:
                self.fault = "old process cleanup failed: " + str(error)
                return self._result(False, False, self.fault)
            try:
                if not self.probe.gone(previous, self.timeout):
                    raise RuntimeError("old ROS graph did not disappear")
                self._launch(candidate)
            except Exception as error:
                try:
                    self._stop()
                except Exception as stop_error:
                    self.fault = "candidate cleanup failed: " + str(stop_error)
                    return self._result(False, False, self.fault)
                return self._restore(previous, "candidate failed: " + str(error))
            self.epoch += 1
            return self._result(True, False,
                "new pipeline ready; old process group stopped; new source sessions required")
        finally:
            self._lock.release()

    def shutdown(self):
        with self._lock:
            self._stop()
