# M2x — independent native workload sampling

Previously M2w tracked state residency only when clients requested GetSemanticStatus.
Now a dedicated one-second wall timer samples native YAMNet states and caches
the workload. Status requests do not poll the native runtime. Timer and service
use the default mutually exclusive callback group, preventing interleaved reloads.

New fields: classifier_has_workload_sample, classifier_workload_samples,
classifier_workload_sample_failures and classifier_last_workload_sample_age_ms.
Before the first sample, the flag is false and age is zero. A failed sample
preserves the last successful cache; clients must inspect its age. Worker reload
resets cache and counters.

This remains diagnostic only: sampling can miss short transitions; a source
state is not a native job identifier. ASR has no running-job start timestamp.
No automatic worker restart is enabled.