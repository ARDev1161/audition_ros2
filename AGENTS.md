# AGENTS.md

## Architecture
- libaudition owns reusable audio algorithms: DSP, ODAS/DOA, separation, source tracking, VAD, ASR, sound classification and acoustic processing.
- audition_ros2 is a ROS 2 wrapper for robots: ROS nodes/messages, parameters, lifecycle, TF2, robot actions, Nav2, and robot-specific multimodal integration.
- Reuse libaudition public APIs; do not duplicate audio algorithms in audition_ros2. A sound bearing is not a metric position or motion authorization.

## Branches and reviews
- Never push directly to main or merge PRs without explicit owner approval.
- M0–M3 are merged into main. M4 development stays in feat/m4-multimodal-fusion (PR #2); later stages use separate feature branches and PRs. Never push directly to main or merge without explicit owner approval.
- Website documentation, Sphinx/Furo theme, Mermaid viewer and GitHub Pages workflow belong ONLY in the separate documentation branch. Deploy Pages from documentation. Do not merge website-only work into main or the runtime PR.
- Changes to libaudition require a separate feature branch and a separate PR in ARDev1161/libaudition for owner review; never edit libaudition main. Link the dependency PR and pin the tested revision when necessary.

## Workflow and quality
- Follow docs/ROADMAP.md and docs/LIBAUDITION_INTEGRATION.md; documentation branch holds the published documentation.
- Prefer substantial cohesive milestones to tiny parameter/policy changes.
- Add tests; verify remote commit SHA, CI start and final status. Do not report queued checks as passing.
- Distinguish synthetic CI tests from real hardware, calibration, ASR accuracy and robot safety acceptance.
- Keep the documentation style aligned with libaudition: Furo, MyST, Mermaid, responsive layout, wheel zoom on desktop and pinch zoom on mobile.

## Independent test projects and acceptance
- This repository develops audition_ros2 according to docs/ROADMAP.md and docs/LIBAUDITION_INTEGRATION.md. Do NOT implement the separate libaudition acoustic room/source simulator here, copy its scope into this roadmap, or require it to complete the present ROS2 CI tasks.
- Software-only acceptance runs in CI (unit, DDS, deterministic fixtures, replay, failure handling). Keep real-array calibration, physical robot HIL and field accuracy NOT RUN / DEFERRED, never PASS by inference.
- Only later, by explicit owner decision, integrate external acoustic simulator outputs, then optional Gazebo/Isaac Sim robotic testing, then repaired physical-robot HIL. Treat these as independent future acceptance stages.
