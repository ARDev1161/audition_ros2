"""Process-owned, bounded profile switching. Not zero-downtime in-place reload."""
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


class PipelineSupervisor:
    """Injectable state machine: validate(spec), spawn(spec), probe.ready/gone."""
    def __init__(self, validate, spawn, probe, timeout=30):
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.validate, self.spawn, self.probe = validate, spawn, probe
        self.timeout = timeout
        self.process = None
        self.active = None
        self.epoch = 0
        self._lock = threading.Lock()

    def _result(self, ok, rollback, message):
        return SwitchResult(ok, rollback, self.epoch,
                            self.active.profile if self.active else "", message)

    def _stop(self):
        process, self.process = self.process, None
        self.active = None
        if process is not None:
            process.stop()

    def _launch(self, spec):
        process = self.spawn(spec)
        try:
            if not self.probe.ready(spec, process, self.timeout):
                raise RuntimeError("pipeline nodes/capability service did not become ready")
        except Exception:
            process.stop()
            raise
        self.process, self.active = process, spec

    def start(self, spec):
        with self._lock:
            if self.process is not None:
                raise RuntimeError("supervisor already owns a pipeline")
            self.validate(spec)
            self._launch(spec)

    def _restore(self, previous, cause):
        try:
            if not self.probe.gone(previous, self.timeout):
                raise RuntimeError("previous/candidate ROS graph did not disappear")
            self.validate(previous)
            self._launch(previous)
            return self._result(False, True, cause + "; previous profile restored")
        except Exception as error:
            self.process, self.active = None, None
            return self._result(False, False,
                                cause + "; ROLLBACK FAILED: " + str(error))

    def switch(self, candidate, dry_run=False):
        if not self._lock.acquire(blocking=False):
            return self._result(False, False, "another switch is in progress")
        try:
            if self.active is None or self.process is None:
                return self._result(False, False, "no supervised pipeline is running")
            if self.process.poll() is not None:
                return self._result(False, False,
                                    "owned pipeline exited; refusing unverified switch")
            try:
                self.validate(candidate)  # preflight BEFORE stopping active graph
            except Exception as error:
                return self._result(False, False, "preflight rejected: " + str(error))
            if dry_run:
                return self._result(True, False, "static preflight passed; no processes changed")
            previous = self.active
            try:
                self._stop()
                if not self.probe.gone(previous, self.timeout):
                    raise RuntimeError("old ROS graph did not disappear")
                self._launch(candidate)
            except Exception as error:
                try:
                    self._stop()
                except Exception as stop_error:
                    return self._result(False, False,
                        "candidate cleanup failed; refusing unsafe rollback: " + str(stop_error))
                return self._restore(previous, "candidate failed: " + str(error))
            self.epoch += 1
            return self._result(True, False,
                "new pipeline ready; old process group stopped; new source sessions required")
        finally:
            self._lock.release()

    def shutdown(self):
        with self._lock:
            self._stop()
