"""M2o stop/start and failback tests, with injected process and graph failures."""
import importlib.util
from pathlib import Path
import sys
import unittest

PATH = Path(__file__).resolve().parents[1] / "launch" / "profile_supervisor_core.py"
spec = importlib.util.spec_from_file_location("profile_supervisor_core", PATH)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
ProfileSpec, PipelineSupervisor = module.ProfileSpec, module.PipelineSupervisor


class FakeProcess:
    def __init__(self, name, events):
        self.name, self.events, self.dead = name, events, False
        events.append("spawn:" + name)

    def poll(self):
        return 1 if self.dead else None

    def stop(self):
        self.dead = True
        self.events.append("stop:" + self.name)


class FakeProbe:
    def __init__(self, events):
        self.events, self.fail, self.fail_gone = events, set(), False

    def ready(self, spec, process, timeout):
        self.events.append("ready:" + spec.profile)
        return spec.profile not in self.fail

    def gone(self, spec, timeout):
        self.events.append("gone:" + spec.profile)
        return not self.fail_gone


class SwitchTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.probe = FakeProbe(self.events)

        def validate(spec):
            self.events.append("validate:" + spec.profile)
            if spec.profile == "invalid":
                raise ValueError("invalid geometry")

        self.supervisor = PipelineSupervisor(
            validate, lambda spec: FakeProcess(spec.profile, self.events),
            self.probe, timeout=1)
        self.supervisor.start(ProfileSpec("old"))

    def tearDown(self):
        self.supervisor.shutdown()

    def test_preflight_rejection_never_stops_active(self):
        result = self.supervisor.switch(ProfileSpec("invalid"))
        self.assertFalse(result.success)
        self.assertEqual(result.active_profile, "old")
        self.assertNotIn("stop:old", self.events)

    def test_dry_run_never_touches_process_or_epoch(self):
        result = self.supervisor.switch(ProfileSpec("new"), True)
        self.assertTrue(result.success)
        self.assertEqual(result.reload_epoch, 0)
        self.assertNotIn("spawn:new", self.events)

    def test_successful_switch_is_ordered(self):
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertTrue(result.success)
        self.assertEqual(result.reload_epoch, 1)
        self.assertLess(self.events.index("validate:new"), self.events.index("stop:old"))
        self.assertLess(self.events.index("gone:old"), self.events.index("spawn:new"))

    def test_candidate_failure_rolls_back_without_epoch_increment(self):
        self.probe.fail.add("new")
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertTrue(result.rollback_succeeded)
        self.assertEqual(result.active_profile, "old")
        self.assertEqual(result.reload_epoch, 0)
        self.assertIn("stop:new", self.events)
        self.assertEqual(self.events.count("spawn:old"), 2)

    def test_rollback_failure_fails_closed(self):
        self.probe.fail.update({"new", "old"})
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertFalse(result.rollback_succeeded)
        self.assertEqual(result.active_profile, "")
        self.assertIsNone(self.supervisor.process)

    def test_old_graph_not_gone_refuses_candidate(self):
        self.probe.fail_gone = True
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertFalse(result.rollback_succeeded)
        self.assertNotIn("spawn:new", self.events)

    def test_dead_owned_process_rejected(self):
        self.supervisor.process.dead = True
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertIn("exited", result.message)


if __name__ == "__main__":
    unittest.main()
