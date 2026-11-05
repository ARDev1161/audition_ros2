# M3 LookAtSound advisory acquisition action

The opt-in `look_at_sound` action server listens to
`attention/target_base` (the TF2 bridge output) and returns the first
**fresh, valid, unit-length bearing** in the configured target frame.
This is **not a robot motion action**: it never publishes velocity,
navigation, or joint commands, and does not claim that a robot turned.

Launch `m3_attention_tf.launch.py` and
`m3_look_at_sound.launch.py` with the same namespace and target frame.
Call `/audition/look_at_sound` with an optional `source_id` filter
and `timeout_ms` (1–10000 by default). Invalid timeouts and concurrent
goals are rejected. Accepted goals support cancellation, feedback while
waiting, timeout abortion and successful acquisition. A target must have
arrived within `maximum_target_age_ms` (default 500), have `valid=true`,
the correct frame and a finite unit vector. Only one active goal is allowed.
The returned `AttentionTarget` retains source and scene provenance.

**Limitations:** the freshness gate is based on arrival monotonic time
and relies on the upstream TF2 bridge to validate observation stamps and
replay. It does not verify actual actuator rotation or robot orientation.
Downstream motion controllers must independently validate freshness,
frames, safety limits and collision clearance; they must not interpret
this action's success as physical completion.

## ROS graph regression

The Humble CI workflow launches the actual Python action server in a
separate process and DDS domain (115), then runs
`audition_tests/smoke_look_at_sound_graph.py`. It checks source-filtered
acquisition, rejection of concurrent and invalid goals, timeout abortion,
cancellation and preservation of source/scene provenance. The probe injects
synthetic transformed targets; it does not measure acoustic localization or
exercise robot actuators.

## Post-acceptance observation fence

A newly accepted goal requires a target message received after goal admission. A valid bearing cached before the goal cannot satisfy it, even inside the arrival-age window. The server records an ingress sequence under the existing lock and requires a strictly later sequence for success. This does not guarantee a new sensor measurement: upstream TF2 timestamp/replay fencing remains mandatory. The ROS graph regression tests cached-target rejection and unit tests cover sequence equality, regression and advancement.

## Stamp replay gate

Acquisition now requires a valid target observation timestamp strictly newer than the most recent valid stamp observed at goal acceptance, or a higher scene generation. This complements the post-admission DDS sequence gate: replaying the same stamped observation cannot complete a new action. The upstream TF2 bridge remains responsible for authenticating stream provenance and rejecting replay. A pure policy regression covers equal and advancing stamps.

## Live replay regression and server startup

The ROS graph smoke now exercises both stale cached observations and a
replayed identical observation stamp delivered after goal acceptance. Neither
may satisfy a new goal; a genuinely advancing stamp must still acquire.
The persistent stamp watermark is initialized before DDS subscriptions or
ActionServer callbacks are created, including when the first goal arrives
before the first acoustic observation. The replay probe uses a fixed stamp
for all replay publications and an advancing clock stamp for recovery.

## Validated watermark admission

Only valid unit-vector bearings in the configured target frame can advance the action server's persistent observation watermark. An invalid or wrong-frame advisory (even with a very high scene generation or timestamp) cannot poison the watermark and block later legitimate acquisition. A ROS graph regression publishes a wrong-frame high-generation observation, then checks recovery using a normal source. This does not replace upstream TF2 replay checks or authenticate message publishers.

## Admission-time deadline

The timeout starts when the ActionServer **accepts** a goal, not when its
execution callback eventually starts. The deadline is recorded with the
accepted goal under the same lock as its ingress watermark. The worker checks
expiration before attempting success, so a late valid observation cannot
turn an already expired goal into a success. Cancellation remains available.
A deterministic policy test freezes the admission clock to verify this
contract; the existing ROS graph smoke verifies timeout abortion.

## Graph regression determinism

The graph probe reuses a single stamped target during each acquisition rather than generating a new stamp on every publish. It also drains pending DDS callbacks before the cached-target rejection scenario. This prevents delayed deliveries from a previous successful acquisition from masquerading as a fresh measurement in the next goal. The server replay and admission rules are unchanged.

## Monotonic ingress admission and cache revocation

The action server now admits only **strictly advancing** valid transformed
bearings to its cached target and ingress sequence. Duplicate and older
DDS deliveries are ignored rather than replacing the newest bearing or
refreshing its arrival age. Invalid or wrong-frame advisories clear the
cached bearing but cannot consume the persistent watermark. A later valid,
advancing observation restores acquisition. This is a conservative
advisory-only cache policy, not publisher authentication or a motion safety
controller. Unit tests exercise older, duplicate, invalid and recovering
messages; the ROS graph probe injects out-of-order stamped DDS observations.

## Stale invalidation fence

Invalid advisories stamped at or before the latest accepted valid bearing no longer revoke its cache. Newer invalid advisories still revoke without advancing the valid watermark; a valid advancing observation can recover. Unstamped invalid advisories cannot be ordered and do not revoke. Unit and ROS graph regressions cover stale invalidation and recovery. This is not publisher authentication or robot motion safety.

## Optional acquisition bearing sector

`maximum_abs_azimuth_rad` (0 disables; otherwise 0..pi) and `maximum_abs_elevation_rad` (0 disables; otherwise 0..pi/2) restrict advisory bearing acquisition in `target_frame`. Azimuth uses `atan2(y,x)` and elevation uses `atan2(z,hypot(x,y))`. Parameters must be finite. Out-of-sector observations cannot satisfy a goal. Defaults preserve unrestricted acquisition. This is not actuator authorization, a collision check, or a motion controller.

## Sector-constrained live ROS graph acceptance

The Humble workflow additionally launches a dedicated LookAtSound server
in DDS domain 116 with both angular limits set to 0.5 rad. A synthetic
ROS publisher sends unit-length bearings outside azimuth and elevation
limits and verifies goal timeout, then sends an in-sector bearing and
verifies successful acquisition. This tests the configured launch-to-action
path rather than just the Python predicate. Freshness validation also
rejects non-finite coordinates and impossible future monotonic arrivals.
The test does not exercise physical robot motion or collision avoidance.
