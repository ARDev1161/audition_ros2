"""M2p process-owned supervisor with bounded recovery and fault containment."""
from dataclasses import dataclass
import threading


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


class PipelineSupervisor:
    """Injectable state machine: validate(spec), spawn(spec), probe.ready/gone/vacant."""
    def __init__(self, validate, spawn, probe, timeout=30):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.validate, self.spawn, self.probe = validate, spawn, probe
        self.timeout = timeout
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
        return SupervisorStatus(self.active is not None and self.process is not None
                                and self.process.poll() is None,
                                self.process is not None,
                                self.active.profile if self.active else "",
                                self.last_good.profile if self.last_good else "",
                                self.epoch, self.fault)

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
                raise
            self.process = None
        self.active = None

    def _launch(self, spec):
        process = self.spawn(spec)
        self.process, self.active = process, None
        try:
            if not self.probe.ready(spec, process, self.timeout):
                raise RuntimeError("pipeline nodes/capability service did not become ready")
        except Exception:
            self._stop()
            raise
        self.active = spec
        self.last_good = spec
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
        """Reap a dead launch and its process group; never auto-restart."""
        if not self._lock.acquire(blocking=False):
            return
        try:
            if self.process is None or self.process.poll() is None:
                return
            previous = self.active
            self.fault = "owned launch process exited unexpectedly"
            try:
                self._stop()
                if previous is not None and not self.probe.gone(previous, self.timeout):
                    self.fault += "; ROS graph still present (manual cleanup required)"
            except Exception as error:
                self.active = None
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
