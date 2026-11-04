# M3ah — bounded source identity admission

An optional `attention_manager` startup parameter,
`maximum_source_id_bytes` (default 0 = disabled, valid 0–4096),
rejects an incoming `AcousticScene` containing any source ID whose
UTF-8 encoded byte length exceeds the configured limit.

The check runs before constructing duplicate-ID sets or reconciling
direction quarantines. It applies to every record, including ineligible,
foreign-generation, and low-activity sources. Rejection produces an
invalid advisory with reason `source_id_limit`, invalidates pending
acquisition/switch evidence, and preserves the last accepted stream
watermark for recovery on the next admissible snapshot.

This is a bounded processing safeguard, not a bound on DDS transport
allocation or a Unicode character-count restriction. The independent
`maximum_scene_sources` cap remains available. Both controls are opt-in
and do not change defaults. Three GTests cover fail-closed rejection,
non-candidate records, recovery and configuration validation.
No navigation or motor action is emitted.
