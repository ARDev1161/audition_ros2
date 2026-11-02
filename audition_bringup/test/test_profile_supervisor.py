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
        self.name, self.events, self.dead, self.fail_stop = name, events, False, False
        events.append("spawn:" + name)

    def poll(self):
        return 1 if self.dead else None

    def stop(self):
        if self.fail_stop:
            raise RuntimeError("injected group cleanup failure")
        self.dead = True
        self.events.append("stop:" + self.name)


class FakeProbe:
    def __init__(self, events):
        self.events, self.fail, self.fail_gone, self.occupied = events, set(), False, False

    def vacant(self, spec):
        return not self.occupied

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

    def test_existing_namespace_is_not_taken_over(self):
        other = PipelineSupervisor(lambda spec: None,
            lambda spec: FakeProcess(spec.profile, self.events), self.probe, timeout=1)
        self.probe.occupied = True
        with self.assertRaisesRegex(RuntimeError, "namespace already"):
            other.start(ProfileSpec("other"))
        self.assertIsNone(other.process)

    def test_failed_old_cleanup_retains_ownership_and_blocks_candidate(self):
        original = self.supervisor.process
        original.fail_stop = True
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertIs(self.supervisor.process, original)
        self.assertNotIn("spawn:new", self.events)
        self.assertTrue(self.supervisor.status().owned_process_present)
        original.fail_stop = False

    def test_crash_monitor_cleans_up_and_manual_recovery(self):
        self.supervisor.process.dead = True
        self.supervisor.monitor()
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertEqual(state.last_good_profile, "old")
        self.assertIn("exited unexpectedly", state.fault)
        self.assertFalse(state.owned_process_present)
        recovered = self.supervisor.switch(ProfileSpec("old"))
        self.assertTrue(recovered.success)
        self.assertEqual(recovered.reload_epoch, 1)
        self.assertTrue(self.supervisor.status().running)
        self.assertEqual(self.supervisor.status().fault, "")

    def test_recovery_refuses_stale_graph(self):
        self.supervisor.process.dead = True
        self.supervisor.monitor()
        self.probe.fail_gone = True
        result = self.supervisor.switch(ProfileSpec("old"))
        self.assertFalse(result.success)
        self.assertIn("stale pipeline", result.message)
        self.assertEqual(self.events.count("spawn:old"), 1)

    def test_dead_owned_process_rejected(self):
        self.supervisor.process.dead = True
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertIn("exited", result.message)


if __name__ == "__main__":
    unittest.main()
