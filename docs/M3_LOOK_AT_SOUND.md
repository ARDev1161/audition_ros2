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

## Optional multi-observation acquisition confirmation (2026-11-06)

`bearing_confirmations` (1–100, default 1) requires multiple distinct,
strictly advancing accepted target messages **after** goal admission before
reporting acquisition. Polling the same cached DDS observation repeatedly does
not add evidence. Source identity, scene generation and target frame must stay
consistent; a change restarts confirmation. Optional
`maximum_confirmation_jump_rad` (0 disables, otherwise 0..pi) requires every
confirmed bearing to remain within the configured angle of the **first**
observation in its current confirmation run. A discontinuity restarts the run.
The deadline remains measured from goal admission, including while gathering
evidence. No physical motion is performed. A single ROS publisher can still
fabricate multiple advancing observations: this is a stability filter, not
source authentication or independent sensor validation.

## Multi-observation ROS graph regression

Humble CI now starts a dedicated action server in DDS domain 117 with three
confirmations and a 0.2 rad angular limit. The live action client checks that
repeated delivery of the same timestamp does not count as independent evidence,
that a new source generation resets the evidence even if the source ID is
unchanged, and that an angular jump restarts confirmation. The final goal
succeeds only after three advancing, consistent observations. This validates
ROS message transport and action outcomes, not microphone or actuator hardware.

## Confirmation revocation fence (2026-11-06)

A newer invalid or wrong-frame advisory revokes both the cached bearing and
any partially accumulated multi-observation confirmation run. Subsequent
valid bearings must start a new run; observations on opposite sides of a
revocation cannot jointly satisfy `bearing_confirmations`. Stale or
unstamped invalidations do not revoke the cache or evidence. The invalid
advisory does not advance the valid-stamp watermark, so later valid bearings
can recover. A deterministic Python regression covers stale invalidation,
newer revocation and recovery. This is an advisory safety boundary, not
publisher authentication, independent sensor validation or actuator control.

## Atomic revocation and confirmation evaluation (2026-11-06)

The action worker now reads the cached target, its ingress sequence, and the
confirmation counter under the **same lock** used by DDS admission and
invalidation. Previously it could copy a valid bearing, release the lock,
receive a newer invalidation, then reacquire the lock and count the revoked
snapshot toward a successful action. This time-of-check/time-of-use race is
closed: a revoked snapshot cannot contribute new confirmation evidence.
Deadline eligibility is also checked inside the atomic evaluation; expiration
still causes an aborted action, not success. Deterministic policy regressions
cover a partially confirmed target revoked between worker polls, recovery
requiring a full fresh streak, and expiration. The check does not authenticate
publishers or guarantee that an actuator is safe to move.

## Live DDS revocation acceptance

The multi-confirmation Humble ROS graph probe also tests both sides of the
invalidation boundary: an out-of-order invalid advisory must not erase two
previous confirmations, whereas a newer invalid advisory must erase the
partial streak. After the newer invalidation, three advancing, consistent
bearings are required before the action succeeds. This is a live DDS/action
regression for advisory cache semantics; it does not verify authenticated
publishers, real microphones or actuator safety.

## Revocation timestamp watermark (2026-11-06)

A newer invalidation now advances a separate **revocation watermark**, even
though it does not advance the valid-observation watermark. Previously a
valid DDS message delayed behind a newer invalidation could be accepted and
repopulate the cache because only the last *valid* stamp was checked. A valid
bearing must now advance past both watermarks; an invalidation must advance
past both to revoke. Equal-stamp valid replay is rejected, and a strictly
newer valid observation restores acquisition. This closes delayed-DDS
resurrection of revoked observations, without requiring ROS clock comparisons
or claiming publisher authentication. Pure policy and live DDS graph
regressions cover delayed valid replay and recovery.

## Foreign-frame revocation isolation (2026-11-06)

A wrong-frame advisory is ignored before either the valid or revocation
watermark is evaluated. Previously a `map`-frame invalid advisory with an
arbitrarily high scene generation could permanently poison the revocation
watermark of a `base_link` LookAtSound server. Only advisories explicitly
in the configured target frame may revoke its cache or confirmation evidence.
The ROS graph poison/recovery smoke and deterministic regression cover this.
This is frame consistency, not DDS publisher authentication.

## Explicit revocation versus rejected bearings (2026-11-06)

Only a same-frame advisory with `valid=false` can revoke cached acquisition
and confirmation evidence. A `valid=true` advisory rejected because its unit
vector is malformed, non-finite or outside the configured angular sector is
ignored without advancing either watermark. Otherwise a future-stamped
out-of-sector observation could poison the revocation watermark and prevent
normal acquisition. A deterministic regression covers sector rejection,
non-unit and NaN vectors, recovery and explicit invalidation. This does not
authenticate publishers or imply robot motion safety.

## Bounded confirmation continuity (2026-11-06)

Optional `maximum_confirmation_gap_ms` (0 disables, otherwise 1–10000 ms) limits the elapsed monotonic ingress time between consecutive counted bearings. When the gap exceeds this limit, the partial evidence run restarts at the next eligible bearing, even if the last cached bearing is still fresh. This prevents widely separated observations from being treated as one continuous confirmation streak. It does not infer sensor independence or actuator safety. Default 0 preserves existing behavior. Deterministic regression exercises a two-bearing streak, a long interruption, and three new consecutive bearings.

## Bounded total confirmation window (2026-11-06)

Optional `maximum_confirmation_window_ms` (0 disables; otherwise 1–10000 ms) caps the monotonic ingress-time span from the **first** counted observation to the newest one in a confirmation run. If exceeded, the newest observation becomes the first of a new run. This complements the adjacent `maximum_confirmation_gap_ms` gate: many individually short gaps cannot accumulate evidence indefinitely. The first-observation anchor and window reset on invalidation, goal admission, and evidence discontinuity. Default 0 preserves earlier behavior. Deterministic regression covers short adjacent gaps exceeding the total window, restart and recovery. This remains advisory-only; it does not authorize robot motion.


### Minimum independent confirmation spacing

`minimum_confirmation_spacing_ms` (default `0`, disabled; accepted `0–10000`)
requires successive accepted bearing confirmations to be separated by at
least the configured monotonic arrival-time interval. Closely spaced DDS
updates do not advance the confirmation count, even when their timestamps
and sequence numbers differ. This prevents a burst of nearly identical
observations from satisfying a multi-confirmation goal. The last **counted**
observation remains the spacing reference; skipped samples do not extend it.
The existing maximum gap and total window constraints still apply. This is
an observation-cadence policy, not a claim of independent sensor evidence or
physical localization confidence. The default preserves existing behavior.


## Source-scoped revocation of cached bearings (2026-11-06)

An explicit invalidation for source B in the **same scene generation** no
longer erases a currently cached valid bearing for source A, nor its partial
confirmation evidence. This also prevents an unrelated source's high-stamped
invalidation from advancing the revocation watermark and blocking legitimate
later observations of A. An invalidation for the cached source still revokes
normally. A newer scene generation is treated as a global epoch change and
may revoke the old cached bearing regardless of source identity. Empty source
IDs retain conservative global invalidation semantics. Deterministic tests
cover unrelated-source isolation, same-source revocation, recovery and a
newer-epoch reset. This is not publisher authentication or motion safety.

## Source-generation invalidation fence (2026-11-06)

Within one scene epoch, a retired lower source generation cannot invalidate
an active newer incarnation of the same source, regardless of the invalidation
timestamp. Current or newer generations may still revoke, and newer scene
epochs remain global resets. This is a DDS ordering guard, not publisher
authentication or a physical speaker identity guarantee.

## Valid bearing tracker-generation rollback fence (2026-11-06)

A valid, higher-stamped DDS observation from an older `source_generation`
can no longer replace the cached bearing from a newer incarnation of the
same `source_id` within the same scene. Rejected observations do not advance
the watermark or ingress sequence, so the current tracker continues normally.
A newer `scene_generation` starts a new epoch and can admit a lower tracker
generation. The deterministic unit regression covers delayed valid replay,
normal continuation and a scene reset. This does not authenticate publishers.

## Persistent tracker-generation fence after revocation (2026-11-06)

A revoked cache no longer forgets the most recent source incarnation. The
server retains the scene/source/generation fence across explicit invalidation,
including invalidation from a newer tracker generation. Delayed valid DDS
messages from an older generation cannot resurrect a cleared cache merely by
carrying a later timestamp. Current-generation recovery remains possible and
a new scene epoch permits generation numbers to restart. A deterministic
regression checks both paths. This is not publisher authentication.

## Revocation fence after cache retirement (2026-11-06)

A stale `valid=false` retirement from an older `source_generation` cannot
advance the revocation watermark after the current source's cached bearing has
already been cleared. The server checks the retained source-generation fence
before handling retirement, including when `latest` is absent. A retirement
for the current or newer tracker remains effective; a newer scene epoch allows
the tracker numbering to restart. The deterministic regression covers delayed
older retirement, current-generation recovery, newer-generation retirement and
scene restart. This remains a best-effort DDS provenance filter, not publisher
authentication or a robot motion safety mechanism.


## End-to-end bounded confirmation cadence (2026-11-06)

The `m3_look_at_sound.launch.py` entry point now exposes all three existing
server-side cadence policies as typed integer millisecond launch arguments:
`minimum_confirmation_spacing_ms`, `maximum_confirmation_gap_ms`, and
`maximum_confirmation_window_ms`. Each defaults to zero (disabled); each
accepts 0–10000 ms, with server startup rejecting invalid values. This closes
the gap where these policy knobs existed in the Python action server but could
not be configured through the supported launch interface.

Humble CI exercises the real ROS graph with three required confirmations,
180 ms minimum spacing, 750 ms maximum adjacent gap, and 900 ms total window.
The probe verifies that a burst, a broken streak, and a too-long total window
cannot acquire a target, while a fresh correctly spaced streak succeeds.
The test uses synthetic DDS bearings and never commands motion; it is not a
claim of independent acoustic evidence or hardware timing accuracy.
