# M3b — sustained ASR queue pressure diagnostics

The native ASR workload sampler (M3a) now maintains two monotonic residency
clocks: uninterrupted nonempty pending-segment queue, and uninterrupted
nonempty result mailbox. A queue drain resets queue age but not mailbox age.
A mailbox drain resets mailbox age but not queue age.

`GetSemanticStatus` adds:
- `asr_queue_nonempty_age_ms` and `asr_mailbox_nonempty_age_ms`.
- `asr_sustained_queue_pressure`: at least 4 pending segments **now** and
  a queue observed continuously nonempty for at least 10 seconds.

The flag is diagnostic and read-only, not a model failure verdict. The native
queue excludes the running inference job; periodic sampling can miss brief
drains. No automatic restart, fault escalation or ASR deadline is enabled.
A sampling exception, source-generation reset or worker reload invalidates
both clocks; an empty queue or silent input never generates an alert.

The observer tests cover varying queue depths, independent drain behavior,
sampling errors, resets, silence and disabled thresholds. These are
model-independent; device/model throughput must be measured separately.

### M3b CI correction (2026-11-03)

The queue/mailbox residency tracker now explicitly starts a new clock on an
empty-to-nonempty transition, not only on first sample or when becoming empty.
This fixes the CI regression where a drained queue inherited its previous
residency. Tests cover both queue and mailbox drain/reappearance.
