# AGENTS.md

## Architecture
- libaudition owns reusable audio algorithms: DSP, ODAS/DOA, separation, source tracking, VAD, ASR, sound classification and acoustic processing.
- audition_ros2 is a ROS 2 wrapper for robots: ROS nodes/messages, parameters, lifecycle, TF2, robot actions, Nav2, and robot-specific multimodal integration.
- Reuse libaudition public APIs; do not duplicate audio algorithms in audition_ros2. A sound bearing is not a metric position or motion authorization.

## Branches and reviews
- Never push directly to main or merge PRs without explicit owner approval.
- Runtime development stays in feat/m0-ros2-foundation (PR #1), unless instructed otherwise.
- Website documentation, Sphinx/Furo theme, Mermaid viewer and GitHub Pages workflow belong ONLY in the separate documentation branch. Deploy Pages from documentation. Do not merge website-only work into main or the runtime PR.
- Changes to libaudition require a separate feature branch and a separate PR in ARDev1161/libaudition for owner review; never edit libaudition main. Link the dependency PR and pin the tested revision when necessary.

## Workflow and quality
- Follow docs/ROADMAP.md and docs/LIBAUDITION_INTEGRATION.md; documentation branch holds the published documentation.
- Prefer substantial cohesive milestones to tiny parameter/policy changes.
- Add tests; verify remote commit SHA, CI start and final status. Do not report queued checks as passing.
- Distinguish synthetic CI tests from real hardware, calibration, ASR accuracy and robot safety acceptance.
- Keep the documentation style aligned with libaudition: Furo, MyST, Mermaid, responsive layout, wheel zoom on desktop and pinch zoom on mobile.
