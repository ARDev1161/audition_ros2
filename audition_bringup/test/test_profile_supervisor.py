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
flow_spec = importlib.util.spec_from_file_location(
    "profile_audio_flow", PATH.with_name("profile_audio_flow.py"))
flow_module = importlib.util.module_from_spec(flow_spec)
sys.modules[flow_spec.name] = flow_module
flow_spec.loader.exec_module(flow_module)
AudioFlowMonitor = flow_module.AudioFlowMonitor
source_spec = importlib.util.spec_from_file_location(
    "profile_source_flow", PATH.with_name("profile_source_flow.py"))
source_module = importlib.util.module_from_spec(source_spec)
sys.modules[source_spec.name] = source_module
source_spec.loader.exec_module(source_module)
SourceFlowMonitor = source_module.SourceFlowMonitor
semantic_spec = importlib.util.spec_from_file_location(
    "profile_semantic_flow", PATH.with_name("profile_semantic_flow.py"))
semantic_module = importlib.util.module_from_spec(semantic_spec)
sys.modules[semantic_spec.name] = semantic_module
semantic_spec.loader.exec_module(semantic_module)
SemanticFlowMonitor = semantic_module.SemanticFlowMonitor


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds=1.1):
        self.value += seconds


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
        self.health_failure = ""
        self.raise_health = False

    def vacant(self, spec):
        return not self.occupied

    def ready(self, spec, process, timeout):
        self.events.append("ready:" + spec.profile)
        return spec.profile not in self.fail

    def health(self, spec):
        self.events.append("health:" + spec.profile)
        if self.raise_health:
            raise RuntimeError("injected discovery failure")
        return not self.health_failure, self.health_failure

    def gone(self, spec, timeout):
        self.events.append("gone:" + spec.profile)
        return not self.fail_gone


class SwitchTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.probe = FakeProbe(self.events)
        self.clock = FakeClock()

        def validate(spec):
            self.events.append("validate:" + spec.profile)
            if spec.profile == "invalid":
                raise ValueError("invalid geometry")

        self.supervisor = PipelineSupervisor(
            validate, lambda spec: FakeProcess(spec.profile, self.events),
            self.probe, timeout=1, clock=self.clock)
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

    def test_transient_graph_miss_recovers_without_restart(self):
        self.probe.health_failure = "missing semantic_perception"
        for _ in range(4):
            self.clock.advance()
            self.supervisor.monitor()
        state = self.supervisor.status()
        self.assertTrue(state.running)
        self.assertFalse(state.graph_healthy)
        self.assertEqual(state.graph_miss_count, 4)
        self.probe.health_failure = ""
        self.clock.advance()
        self.supervisor.monitor()
        self.assertTrue(self.supervisor.status().graph_healthy)
        self.assertEqual(self.supervisor.status().graph_miss_count, 0)
        self.assertNotIn("stop:old", self.events)

    def test_sustained_graph_failure_fences_and_manual_recovery(self):
        self.probe.health_failure = "missing semantic_perception"
        for _ in range(5):
            self.clock.advance()
            self.supervisor.monitor()
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertFalse(state.graph_healthy)
        self.assertFalse(state.owned_process_present)
        self.assertIn("5 consecutive checks", state.fault)
        self.assertEqual(self.events.count("stop:old"), 1)
        self.probe.health_failure = ""
        recovered = self.supervisor.switch(ProfileSpec("old"))
        self.assertTrue(recovered.success)
        self.assertTrue(self.supervisor.status().graph_healthy)
        self.assertEqual(self.supervisor.status().reload_epoch, 1)

    def test_status_polling_cannot_accelerate_watchdog(self):
        self.probe.health_failure = "missing acoustic_scene"
        self.clock.advance()
        for _ in range(30):
            self.supervisor.monitor()
            self.supervisor.status()
        self.assertEqual(self.supervisor.status().graph_miss_count, 1)
        self.assertEqual(self.events.count("health:old"), 1)
        self.assertTrue(self.supervisor.status().running)

    def test_discovery_exception_counts_as_failed_probe(self):
        self.probe.raise_health = True
        for _ in range(5):
            self.clock.advance()
            self.supervisor.monitor()
        self.assertIn("injected discovery failure", self.supervisor.status().fault)
        self.assertFalse(self.supervisor.status().running)

    def test_graph_failure_cleanup_error_retains_ownership(self):
        self.probe.health_failure = "missing mono_source"
        self.supervisor.process.fail_stop = True
        for _ in range(5):
            self.clock.advance()
            self.supervisor.monitor()
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertTrue(state.owned_process_present)
        self.assertIn("cleanup failed", state.fault)
        self.assertFalse(self.supervisor.switch(ProfileSpec("new")).success)
        self.assertNotIn("spawn:new", self.events)
        self.supervisor.process.fail_stop = False

    def test_successful_switch_resets_health_debounce(self):
        self.probe.health_failure = "missing acoustic_scene"
        self.clock.advance()
        self.supervisor.monitor()
        self.assertEqual(self.supervisor.status().graph_miss_count, 1)
        self.probe.health_failure = ""
        self.assertTrue(self.supervisor.switch(ProfileSpec("new")).success)
        self.assertTrue(self.supervisor.status().graph_healthy)
        self.assertEqual(self.supervisor.status().graph_miss_count, 0)

    def test_dead_owned_process_rejected(self):
        self.supervisor.process.dead = True
        result = self.supervisor.switch(ProfileSpec("new"))
        self.assertFalse(result.success)
        self.assertIn("exited", result.message)


class AudioFlowTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []
        self.probe = FakeProbe(self.events)
        self.flow = AudioFlowMonitor(lambda spec: "alsa", clock=self.clock,
                                     start_grace_s=2, stall_timeout_s=2)
        self.supervisor = PipelineSupervisor(
            lambda spec: None,
            lambda spec: FakeProcess(spec.profile, self.events),
            self.probe, timeout=1, clock=self.clock, audio_flow=self.flow,
            audio_failure_threshold=3)
        self.supervisor.start(ProfileSpec("old"))
        self.session_a, self.session_b = "a" * 32, "b" * 32

    def tearDown(self):
        self.supervisor.shutdown()

    def tick(self, count=1):
        for _ in range(count):
            self.clock.advance()
            self.supervisor.monitor()

    def test_no_pcm_after_grace_fails_closed(self):
        self.tick(5)
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertFalse(state.owned_process_present)
        self.assertIn("capture PCM stalled", state.fault)
        self.assertEqual(self.events.count("stop:old"), 1)

    def test_recent_pcm_keeps_live_pipeline_healthy(self):
        for sequence in range(12):
            self.assertTrue(self.flow.observe(self.session_a, sequence, 160, 1, 160))
            self.tick()
        state = self.supervisor.status()
        self.assertTrue(state.running and state.audio_healthy)
        self.assertTrue(state.audio_watchdog_enabled)
        self.assertEqual(state.accepted_audio_frames, 12)
        self.assertEqual(state.audio_miss_count, 0)

    def test_duplicate_and_malformed_pcm_do_not_refresh_heartbeat(self):
        self.assertTrue(self.flow.observe(self.session_a, 1, 160, 1, 160))
        self.clock.advance(3)
        self.assertFalse(self.flow.observe(self.session_a, 1, 160, 1, 160))
        self.assertFalse(self.flow.observe(self.session_a, 2, 160, 2, 160))
        self.tick(3)
        self.assertFalse(self.supervisor.status().running)
        self.assertEqual(self.events.count("stop:old"), 1)

    def test_transient_stall_recovers_before_debounce(self):
        self.assertTrue(self.flow.observe(self.session_a, 1, 160, 1, 160))
        self.clock.advance(2.1)
        self.supervisor.monitor()
        self.assertEqual(self.supervisor.status().audio_miss_count, 1)
        self.assertTrue(self.flow.observe(self.session_a, 2, 160, 1, 160))
        self.tick()
        self.assertTrue(self.supervisor.status().running)
        self.assertEqual(self.supervisor.status().audio_miss_count, 0)

    def test_reload_retires_old_capture_session(self):
        self.assertTrue(self.flow.observe(self.session_a, 3, 160, 1, 160))
        self.assertTrue(self.supervisor.switch(ProfileSpec("new")).success)
        self.assertFalse(self.flow.observe(self.session_a, 4, 160, 1, 160))
        self.assertTrue(self.flow.observe(self.session_b, 0, 160, 1, 160))
        state = self.supervisor.status()
        self.assertEqual(state.accepted_audio_frames, 1)
        self.assertTrue(state.audio_healthy)
        self.assertEqual(state.reload_epoch, 1)

    def test_replay_profile_reports_flow_without_enforcing_stall(self):
        replay = AudioFlowMonitor(lambda spec: "wav", clock=self.clock,
                                  start_grace_s=2, stall_timeout_s=2)
        replay.reset(ProfileSpec("replay_ci"))
        self.clock.advance(100)
        self.assertFalse(replay.snapshot().timed_out)
        self.assertFalse(replay.snapshot().enabled)
        self.assertTrue(replay.observe(self.session_a, 0, 160, 1, 160))
        self.assertTrue(replay.snapshot().healthy)

    def test_invalid_capture_session_and_sequence_never_count_as_pcm(self):
        self.assertFalse(self.flow.observe("not-a-session", 0, 160, 1, 160))
        self.assertFalse(self.flow.observe(self.session_a, -1, 160, 1, 160))
        self.assertFalse(self.flow.observe(self.session_a, 0, 0, 1, 0))
        self.assertEqual(self.flow.snapshot().accepted_frames, 0)
        self.assertTrue(self.flow.observe(self.session_a, 0, 160, 1, 160))
        self.assertFalse(self.flow.observe(self.session_a, 0, 160, 1, 160))
        self.assertEqual(self.flow.snapshot().accepted_frames, 1)

    def test_retired_session_limit_fails_closed(self):
        flow = AudioFlowMonitor(lambda spec: "alsa", clock=self.clock,
                                max_sessions=1)
        flow.reset(ProfileSpec("old"))
        self.assertTrue(flow.observe(self.session_a, 1, 160, 1, 160))
        flow.reset(ProfileSpec("new"))
        self.assertTrue(flow.observe(self.session_b, 1, 160, 1, 160))
        flow.reset(ProfileSpec("third"))
        self.assertFalse(flow.observe("c" * 32, 1, 160, 1, 160))
        self.assertTrue(flow.snapshot().timed_out)


class SourceFlowTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []
        self.probe = FakeProbe(self.events)
        self.raw = AudioFlowMonitor(lambda spec: "alsa", clock=self.clock,
                                    start_grace_s=2, stall_timeout_s=2)
        self.source = SourceFlowMonitor(
            lambda spec: ("alsa", "mono"), clock=self.clock,
            start_grace_s=2, stall_timeout_s=2)
        self.supervisor = PipelineSupervisor(
            lambda spec: None,
            lambda spec: FakeProcess(spec.profile, self.events),
            self.probe, timeout=1, clock=self.clock, audio_flow=self.raw,
            source_flow=self.source, source_failure_threshold=3)
        self.supervisor.start(ProfileSpec("old"))
        self.capture_session = "a" * 32
        self.spatial_session = "b" * 32

    def tearDown(self):
        self.supervisor.shutdown()

    def raw_tick(self, seq):
        assert self.raw.observe(self.capture_session, seq, 160, 1, 160)
        self.clock.advance()
        self.supervisor.monitor()

    def source_frame(self, seq, session=None, generation=1, source_id="mono"):
        return self.source.observe(
            session or self.spatial_session, source_id, generation, seq,
            16000, 160, 160)

    def test_live_mono_source_stall_contained_while_capture_flows(self):
        for seq in range(4):
            self.raw_tick(seq)
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertIn("source PCM stalled", state.fault)
        self.assertEqual(self.events.count("stop:old"), 1)

    def test_source_pcm_progress_keeps_live_mono_running(self):
        for seq in range(12):
            self.assertTrue(self.source_frame(seq))
            self.raw_tick(seq)
        state = self.supervisor.status()
        self.assertTrue(state.running and state.source_healthy)
        self.assertTrue(state.source_watchdog_enabled)
        self.assertEqual(state.accepted_source_frames, 12)
        self.assertEqual(state.source_miss_count, 0)

    def test_raw_stall_is_not_misdiagnosed_as_downstream_failure(self):
        self.raw_tick(0)
        for _ in range(8):
            self.clock.advance()
            self.supervisor.monitor()
        self.assertIn("capture PCM stalled", self.supervisor.status().fault)
        self.assertNotIn("source PCM stalled", self.supervisor.status().fault)

    def test_source_replay_invalid_frames_and_retired_session_rejected(self):
        self.assertFalse(self.source_frame(0, session="not-hex"))
        self.assertFalse(self.source.observe(self.spatial_session, "mono", 1,
                                            0, 16000, 160, 159))
        self.assertTrue(self.source_frame(0))
        self.assertFalse(self.source_frame(0))
        self.assertFalse(self.source_frame(1, generation=0))
        self.assertTrue(self.source_frame(1, session="c" * 32))
        self.assertFalse(self.source_frame(100))
        self.assertEqual(self.source.snapshot().accepted_frames, 2)

    def test_transient_source_stall_resets_debounce(self):
        self.assertTrue(self.source_frame(0))
        self.raw_tick(0)
        self.clock.advance(2.1)
        self.assertTrue(self.raw.observe(self.capture_session, 1, 160, 1, 160))
        self.supervisor.monitor()
        self.assertEqual(self.supervisor.status().source_miss_count, 1)
        self.assertTrue(self.source_frame(1))
        self.raw_tick(2)
        self.assertTrue(self.supervisor.status().running)
        self.assertEqual(self.supervisor.status().source_miss_count, 0)

    def test_switch_retires_old_spatial_session_and_resets_status(self):
        self.assertTrue(self.source_frame(1))
        self.assertTrue(self.supervisor.switch(ProfileSpec("new")).success)
        self.assertFalse(self.source_frame(2))
        self.assertTrue(self.source_frame(0, session="c" * 32))
        state = self.supervisor.status()
        self.assertEqual(state.accepted_source_frames, 1)
        self.assertEqual(state.reload_epoch, 1)

    def test_odas_and_finite_wav_do_not_enforce_source_stall(self):
        for mode in (("alsa", "odas"), ("wav", "mono")):
            monitor = SourceFlowMonitor(lambda spec, m=mode: m,
                                        clock=self.clock, start_grace_s=1,
                                        stall_timeout_s=1)
            monitor.reset(ProfileSpec("test"))
            self.clock.advance(20)
            self.assertFalse(monitor.snapshot().enabled)
            self.assertFalse(monitor.snapshot().timed_out)

    def test_source_session_limit_fails_closed(self):
        monitor = SourceFlowMonitor(lambda spec: ("alsa", "mono"),
                                    clock=self.clock, max_sessions=1)
        monitor.reset(ProfileSpec("old"))
        self.assertTrue(monitor.observe("a" * 32, "mono", 1, 0, 16000, 1, 1))
        self.assertTrue(monitor.observe("b" * 32, "mono", 1, 0, 16000, 1, 1))
        self.assertFalse(monitor.observe("c" * 32, "mono", 1, 0, 16000, 1, 1))
        self.assertTrue(monitor.snapshot().timed_out)



class SemanticFlowTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.events = []
        self.probe = FakeProbe(self.events)
        self.raw = AudioFlowMonitor(lambda spec: "alsa", clock=self.clock,
                                    start_grace_s=2, stall_timeout_s=2)
        self.source = SourceFlowMonitor(
            lambda spec: ("alsa", "mono"), clock=self.clock,
            start_grace_s=2, stall_timeout_s=2)
        self.semantic = SemanticFlowMonitor(
            lambda spec: ("alsa", "mono"), clock=self.clock,
            start_grace_s=2, stall_timeout_s=2)
        self.supervisor = PipelineSupervisor(
            lambda spec: None,
            lambda spec: FakeProcess(spec.profile, self.events),
            self.probe, timeout=1, clock=self.clock,
            audio_flow=self.raw, source_flow=self.source,
            semantic_flow=self.semantic, semantic_failure_threshold=3,
            source_failure_threshold=3, audio_failure_threshold=3)
        self.supervisor.start(ProfileSpec("old"))
        self.capture_session = "a" * 32
        self.spatial_session = "b" * 32

    def tearDown(self):
        self.supervisor.shutdown()

    def tick(self, seq, source=True, semantic=False):
        assert self.raw.observe(self.capture_session, seq, 160, 1, 160)
        if source:
            assert self.source.observe(self.spatial_session, "mono", 1,
                                       seq, 16000, 160, 160)
        if semantic:
            assert self.semantic.observe(self.spatial_session, "mono", 1,
                                         seq, 160)
        self.clock.advance()
        self.supervisor.monitor()

    def test_missing_semantic_ack_contained_when_upstream_healthy(self):
        for seq in range(4):
            self.tick(seq)
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertIn("semantic PCM stalled", state.fault)
        self.assertNotIn("source PCM stalled", state.fault)
        self.assertEqual(self.events.count("stop:old"), 1)

    def test_continuous_admitted_semantic_pcm_keeps_pipeline_alive(self):
        for seq in range(12):
            self.tick(seq, semantic=True)
        state = self.supervisor.status()
        self.assertTrue(state.running and state.semantic_healthy)
        self.assertTrue(state.semantic_watchdog_enabled)
        self.assertEqual(state.accepted_semantic_frames, 12)
        self.assertEqual(state.semantic_miss_count, 0)

    def test_semantic_replay_malformed_and_retired_sessions_rejected(self):
        self.assertFalse(self.semantic.observe("bad", "mono", 1, 0, 160))
        self.assertFalse(self.semantic.observe(self.spatial_session, "", 1, 0, 160))
        self.assertFalse(self.semantic.observe(self.spatial_session, "mono", 0, 0, 160))
        self.assertTrue(self.semantic.observe(self.spatial_session, "mono", 1, 0, 160))
        self.assertFalse(self.semantic.observe(self.spatial_session, "mono", 1, 0, 160))
        self.assertFalse(self.semantic.observe(self.spatial_session, "mono", 1, 1, 0))
        self.assertTrue(self.semantic.observe("c" * 32, "mono", 1, 0, 160))
        self.assertFalse(self.semantic.observe(self.spatial_session, "mono", 1, 999, 160))
        self.assertEqual(self.semantic.snapshot().accepted_frames, 2)

    def test_source_stall_is_not_misdiagnosed_as_semantic_failure(self):
        for seq in range(4):
            self.tick(seq, source=False)
        state = self.supervisor.status()
        self.assertFalse(state.running)
        self.assertIn("source PCM stalled", state.fault)
        self.assertNotIn("semantic PCM stalled", state.fault)

    def test_capture_stall_is_attributed_to_capture(self):
        self.tick(0, semantic=True)
        for _ in range(8):
            self.clock.advance()
            self.supervisor.monitor()
        state = self.supervisor.status()
        self.assertIn("capture PCM stalled", state.fault)
        self.assertNotIn("semantic PCM stalled", state.fault)

    def test_transient_semantic_delay_recovers(self):
        self.tick(0, semantic=True)
        self.clock.advance(2.1)
        assert self.raw.observe(self.capture_session, 1, 160, 1, 160)
        assert self.source.observe(self.spatial_session, "mono", 1, 1, 16000, 160, 160)
        self.supervisor.monitor()
        self.assertEqual(self.supervisor.status().semantic_miss_count, 1)
        self.tick(2, semantic=True)
        self.assertTrue(self.supervisor.status().running)
        self.assertEqual(self.supervisor.status().semantic_miss_count, 0)

    def test_switch_retires_semantic_session_and_resets_status(self):
        self.tick(0, semantic=True)
        self.assertTrue(self.supervisor.switch(ProfileSpec("new")).success)
        self.assertFalse(self.semantic.observe(self.spatial_session, "mono", 1, 1, 160))
        self.assertTrue(self.semantic.observe("c" * 32, "mono", 1, 0, 160))
        state = self.supervisor.status()
        self.assertEqual(state.accepted_semantic_frames, 1)
        self.assertEqual(state.reload_epoch, 1)

    def test_wav_and_odas_are_observation_only(self):
        for mode in (("wav", "mono"), ("alsa", "odas")):
            monitor = SemanticFlowMonitor(lambda spec, m=mode: m,
                                          clock=self.clock, start_grace_s=1,
                                          stall_timeout_s=1)
            monitor.reset(ProfileSpec("test"))
            self.clock.advance(20)
            self.assertFalse(monitor.snapshot().enabled)
            self.assertFalse(monitor.snapshot().timed_out)

    def test_session_history_exhaustion_fails_closed(self):
        monitor = SemanticFlowMonitor(lambda spec: ("alsa", "mono"),
                                      clock=self.clock, max_sessions=1)
        monitor.reset(ProfileSpec("test"))
        self.assertTrue(monitor.observe("a" * 32, "mono", 1, 0, 160))
        self.assertTrue(monitor.observe("b" * 32, "mono", 1, 0, 160))
        self.assertFalse(monitor.observe("c" * 32, "mono", 1, 0, 160))
        self.assertTrue(monitor.snapshot().timed_out)


if __name__ == "__main__":
    unittest.main()
