# M2h semantic worker restart and rollback

## M2h: transactional semantic worker restart (2026-11-02)

The namespaced `audition_msgs/srv/ReloadSemanticWorkers` service at
`<namespace>/semantic_perception/reload_workers` supports an explicit
**same-configuration** restart of classifier and speech workers. The
`dry_run` option builds candidate model workers without replacing live
instances. On commit, both candidates are constructed successfully before
the existing classifier/ASR workers are swapped. A model-load failure returns
`success=false` and leaves the old workers and canonical admission untouched.

After a successful commit, the semantic gate drops queued PCM and old
source snapshots until a strictly newer canonical source snapshot arrives.
Within the same upstream generation, PCM at or before that snapshot's capture
timestamp remains rejected. The old worker instances are destroyed without
draining their inference results to ROS, so stale classifications and ASR cannot
be published from the replaced workers. The upstream `source_generation`
is **not** incremented by this semantic-only operation. The read-only
`GetSemanticStatus` reports the local `reload_epoch`, successes, failures,
and whether a new source snapshot is awaited.

This is a complete **semantic worker restart transaction**, not a
`ReloadPipeline` across capture, ODAS and scene. It does not accept changed
models, paths, geometry or sample rate; ROS parameters remain immutable.
Model construction may temporarily duplicate model memory and block the
semantic executor. For hardware or model configuration changes, stop and
relaunch the validated profile. Full cross-node two-phase reload, upstream
generation bump, resource budgeting and failure-injection HIL remain open.

~~~bash
ros2 service call /audition/semantic_perception/reload_workers \
  audition_msgs/srv/ReloadSemanticWorkers "{dry_run: true}"
ros2 service call /audition/semantic_perception/reload_workers \
  audition_msgs/srv/ReloadSemanticWorkers "{dry_run: false}"
ros2 service call /audition/semantic_perception/get_semantic_status \
  audition_msgs/srv/GetSemanticStatus "{}"
~~~


## Transaction invariants

1. All semantic subscriptions, timer and reload service use the same
   mutually-exclusive ROS callback group; no old completion callback publishes
   during replacement.
2. Both new classifier and speech worker instances must be constructed before
   either live pointer changes. A thrown model load leaves the old workers in
   place.
3. A successful swap clears semantic source admission. A source snapshot must
   be newer than the last accepted snapshot (or have a newer real upstream
   generation) before PCM can resume.
4. Old queued PCM in the same upstream generation is rejected by capture
   timestamp. The service reports a *local* reload epoch; this is not a spatial
   source generation and is not exported as a fabricated source identity.
5. A dry run must not advance the local epoch or clear source admission.

## Coverage

`semantic_epoch_gate_test` covers stale snapshot rejection, old queued PCM,
same-generation readmission and a genuine upstream generation change.
`smoke_semantic_reload.py` exercises dry-run, commit, status and continued
mono replay on a real ROS graph. Native model-enabled workflows compile the
same service and re-instantiation paths; actual hot-reload model memory
requirements and physical-device timing are not benchmarked.
