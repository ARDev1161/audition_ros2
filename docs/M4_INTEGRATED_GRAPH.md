# M4-A integrated ROS graph and staged acceptance

The optional `m4_fusion_diagnostics.launch.py` composes three existing ROS
consumers **under one namespace and world frame**:

- `av_pair_node`: canonical `sources` + `vision/bearings` to
  `fusion/av_pair_hypotheses`;
- `bearing_candidate_node`: source direction + stamped TF2 to
  `fusion/bearing_candidates` (non-authoritative diagnostics);
- `pose_evidence_node`: canonical source + `fusion/input_pose` to
  `fusion/pose_evidence_diagnostics`.

The launch starts no physical capture, audio DSP, visual detector, radar,
Nav2 controller, motion command or authoritative `LocalizedSource`. The
source publisher must be the canonical `libaudition`-backed ROS scene,
not an untrusted substitute. TF2 and camera observations remain optional
at runtime: loss of one must not prevent basic audio perception.

## CI contract

`audition_tests/smoke_m4_integrated_graph.py` verifies the **combined**
ROS 2 graph (not three separately launched graphs): node discovery,
Odometry ingestion, synchronized timestamped TF2, one active audio source,
one visual track, AV hypothesis, one-view bearing rejection, and
diagnostic-only pose matching. The test also checks source retirement across both AV and bearing-candidate outputs, fresh scene incarnation admission with pose-cache invalidation, and rejection of a delayed former incarnation. It explicitly checks there is no default
`localized_sources` or `cmd_vel` publisher in the namespace.

Separate existing tests cover replay/session changes, timestamp skew,
ambiguous AV association, missing/mismatched TF2, six-ray conditional
evidence, malformed frames and backend-specific native integrations.
The combined smoke proves wiring, discovery and one bounded positive
path; it **does not** demonstrate perception quality or verified identity.

## Acceptance stages

| Stage | Current criterion | Deferred criterion |
| --- | --- | --- |
| CI software | build, messages, DDS graph, bounds, ordering, negative cases, deterministic recorded/synthetic fixtures | none |
| Acoustic simulator (separate libaudition work) | not part of this M4-A deliverable | real RIR, moving sources/robot, model accuracy |
| Robot simulation (Gazebo or Isaac Sim) | not required to merge this software-only integration | synchronized TF2/robot trajectories, AV/radar ground truth, Nav2 |
| Robot HIL | not required now | array calibration, measured DOA/ASR, motor noise, end-to-end latency, stop/safety |

A successful software CI is **CI PASS / HIL NOT RUN**, not a claim that
the Roadmap's field/ground-truth acceptance gates have passed. M4 metric
localization and M5 Nav2 remain gated by the later validation stages.
