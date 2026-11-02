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
