# M2o — supervised profile switching with bounded rollback

This is an **opt-in process-level stop/start transaction**, not a zero-downtime
two-phase, in-place ROS 2 parameter reload. The supervisor owns every process
it starts and never terminates unrelated ROS nodes. Do not launch a second
pipeline in the same namespace.

## Start

After sourcing Humble and the workspace:

    python3 $(ros2 pkg prefix audition_bringup)/share/audition_bringup/launch/profile_supervisor.py \
      --profile replay_ci --wav-path /absolute/mono-16k.wav

In another terminal:

    ros2 service call /audition/pipeline/reload audition_msgs/srv/ReloadPipeline \
      "{dry_run: true}"
    ros2 service call /audition/pipeline/reload audition_msgs/srv/ReloadPipeline \
      "{dry_run: false}"

The request accepts a built-in profile name or an absolute YAML path in
the profile field. It may override wav_path, device, models_dir and frame_id.
For the same profile, omitted overrides inherit the currently active values;
for a different profile they do not. Namespace is fixed for the supervisor
lifetime; start a separate supervisor per namespace.

## Guarantees and limits

1. Validate the candidate profile, WAV format, model paths and ODAS geometry
   **before** stopping the existing process. Invalid preflight or dry-run
   never changes active workers.
2. Stop only the owned ROS launch process group, wait until old graph node
   names disappear, then launch the candidate and wait for capture, spatial,
   scene and semantic nodes plus the semantic capability service.
3. On candidate launch/readiness failure, stop it, wait for graph removal,
   then restart the previous profile. Return rollback_succeeded=true only
   after its graph is ready.
4. If graph disappearance, cleanup or rollback fails, report failure and
   **fail closed**. Do not claim a running pipeline or successful rollback.
   reload_epoch increments only on successful candidate activation.
5. The supervisor has no durable journal; its own crash does not guarantee
   recovery. Capture is interrupted during switching. A restarted pipeline
   creates new capture/spatial/scene session IDs, fencing old PCM.
   Successful service response proves graph readiness, **not** measured model
   accuracy, microphone calibration or continuous audio delivery.
6. Preflight verifies file existence/schema, not available GPU/RAM or whether
   optional native backends can load. These failures may only be detected
   after stopping the old process; rollback is best effort. Service timeout
   or disconnected caller must not be treated as success.
7. This is not a fully atomic cross-node ReloadPipeline. For strict
   availability, add staged resource reservation and two-phase commit.

The generated ReloadPipeline.srv reports success, rollback_succeeded,
active_profile, reload_epoch and message. Per-launch logs are retained under
/tmp/audition-profile-*.log.

Unit tests inject candidate failure, graph disappearance failure, rollback
failure and preflight rejection. Humble CI exercises the real service on a
synthetic mono WAV graph. Native ODAS CI is a separate required gate;
physical ReSpeaker acceptance remains unverified.

## M2p: process death, ownership and explicit recovery

The supervisor refuses to start when the namespace already contains expected
pipeline nodes; it never adopts or kills an unrelated graph. Readiness requires
exactly one of each expected ROS node. A 1-second watchdog checks the owned
launch process. If it exits, the supervisor attempts to terminate the entire
owned process group (including children surviving the parent), records a fault,
and does not silently restart an unsafe microphone/model configuration.

    ros2 service call /audition/pipeline/get_status audition_msgs/srv/GetPipelineStatus "{}"

The typed status reports process liveness, owned process presence, active and
last-good profiles, reload epoch and fault. Running means the launch parent
remains alive and initial graph readiness passed; it does NOT continuously
verify child nodes, hardware, model inference or audio quality.

After an unexpected exit and successful cleanup, an explicit ReloadPipeline
request with empty profile recovers the last healthy profile and overrides.
Recovery first requires old graph disappearance, then preflight and startup.
Epoch increments only after successful readiness. Failed cleanup retains
the owned handle, blocks new launches and requires operator intervention.
Process group cleanup sends SIGINT for up to 8s, then SIGKILL for up to 5s,
even when the launch parent already exited. ROS graph disappearance is
discovery evidence, not proof of microphone health.

Unit tests inject crash, failed cleanup, occupied namespace, stale graph and
successful manual recovery. Humble graph smoke checks typed status before
and after a real profile switch. This is not zero-downtime two-phase commit.

## M2q: continuous graph health watchdog (2026-11-02)

After initial readiness, the supervisor checks the four expected ROS node
names (exactly one of each) and the semantic capability service every second.
A missing or duplicate node, missing service or graph-query exception counts
as a failed sample. **Five consecutive failed samples** trigger owned
process-group cleanup, preserve the last healthy profile, record the observed
fault and require explicit `ReloadPipeline` recovery. A healthy sample resets
the consecutive-miss counter. Frequent status requests cannot shorten the
one-second sampling interval; the initial launch readiness check remains
separate. An exited launch parent is still contained immediately.

`GetPipelineStatus.graph_healthy` reports the last observed graph sample;
`graph_miss_count` reports the current consecutive failure count (reset on
successful readiness or cleanup). `running=true` continues to mean the
launch parent is alive and initial readiness passed; it does not imply the
latest graph sample was healthy. If cleanup fails, the supervisor retains
ownership and refuses any new launch. No automatic restart is attempted.

This is **ROS discovery liveness**, not a heartbeat from audio processing,
ALSA health check, model inference probe or proof of useful PCM delivery.
Discovery can be temporarily stale, hence the debounce. It does not solve
zero-downtime two-phase reload, hardware fault diagnosis or microphone
calibration. The Humble suite injects missing nodes, recovery, graph API
exceptions, cleanup failures and high-frequency status polling, and verifies
serialization and live service fields.

## M2r: supervised capture PCM heartbeat and live-stream stall containment (2026-11-02)

The optional supervisor subscribes to the namespaced `audio/raw` stream using
sensor-data QoS. A valid heartbeat requires a nonempty, correctly sized
`AudioFrame`, a valid capture-session token, and strictly increasing sequence
within that session. Duplicate, malformed and retired-session frames do not
refresh the heartbeat. Each supervised activation clears counters and retires
the prior capture session (up to 64; exhaustion fails closed).

`GetPipelineStatus` now reports `audio_watchdog_enabled`, `audio_healthy`,
`audio_miss_count`, `accepted_audio_frames` and `last_audio_age_ms`.
`audio_healthy` means a fresh valid raw PCM message was observed, **not**
that the samples are non-silent or correctly separated. Zero accepted frames
means no heartbeat has been observed in the current activation.

For **live ALSA** only, no PCM within 10 seconds of graph readiness or no
fresh PCM for over 5 seconds is considered a stalled sample. Five consecutive
1-second failed samples cause owned process-group containment, fault recording
and manual recovery, as with the graph watchdog. A transient interruption
resets the audio miss count when fresh PCM resumes. **Finite WAV replay**
never triggers the stall watchdog after EOF, but reports observed PCM and
freshness. ROS graph readiness remains independent of audio readiness.

This is a transport-progress diagnostic, not a hardware timestamp, a model
inference heartbeat, DDS publisher authentication, calibrated DOA, or
zero-downtime reload. The supervisor must remain spinning for subscription
callbacks to be serviced; a blocking reload intentionally resets the
heartbeat for the new process. Humble tests cover missing PCM, live progress,
replay/duplicates, malformed packets, transient recovery, session retirement,
status serialization, and both replay and virtual ALSA ROS graph smokes.

## M2s: downstream source PCM watchdog (2026-11-02)

The supervisor now subscribes to the namespaced `internal/source_audio`
(`SourceAudioFrame`) stream. A source heartbeat requires a valid 128-bit
spatial session ID, nonempty source identity, positive generation,
monotonically increasing per-source sequence, valid sample rate and exactly
`frame_count` mono PCM samples. Replays, malformed frames and retired
spatial sessions do not refresh progress. Per-session cursors are bounded
to 256 source-generation keys; at most 64 retired spatial sessions are
remembered, with fail-closed behavior on exhaustion.

The typed `GetPipelineStatus` response adds `source_watchdog_enabled`,
`source_healthy`, `source_miss_count`, `accepted_source_frames` and
`last_source_age_ms`. Counters reset after successful activation.
These fields report *spatial output transport progress*, not classifier
or ASR completion.

**Enforcement is limited to live ALSA + mono routing.** That route emits
source PCM for each valid capture frame even during silence. After 10 seconds
without initial source PCM or 5 seconds without fresh source PCM, five
consecutive one-second checks trigger owned process containment **only
while upstream raw PCM is fresh**. A raw capture failure is diagnosed by
the capture watchdog instead. Transient delays reset the debounce.
WAV replay is finite and ODAS may legitimately have no separated source
during silence; both are observation-only to avoid false-positive restarts.

Humble tests cover mono progress/stall, upstream-vs-downstream fault
attribution, replay, malformed frames, spatial incarnation fencing,
recovery, profile switching, and service serialization. The virtual ALSA
ROS graph smoke requires both raw and mono source PCM progress. This does
not measure semantic inference latency, DOA accuracy, physical ReSpeaker
behavior or zero-downtime switching.
