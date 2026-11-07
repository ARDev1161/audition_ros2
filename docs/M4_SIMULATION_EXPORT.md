# Simulation Export v1 → M4 evaluation

**Status:** An import contract for a future simulator export. The current
`libaudition` and `odas` repositories do **not** expose this recording
export schema yet. This importer does not start a simulator or infer truth.

Use this to connect an external acoustic simulator (including a moving
robot, several speakers, appliances, passing vehicles and reflective paths)
to the existing **trial-level** M4 evaluator without mixing simulation and
real recordings. Ground truth must be emitted independently by the
simulation engine, not reconstructed from detection residuals.

## Input contract

The importer expects exactly:

```json
{
  "format": "audition.simulation-export.v1",
  "domain": "simulation",
  "dataset_id": "table-seven-speakers-v1",
  "scenarios": [
    {
      "scenario_id": "scenario-01",
      "split": "held_out",
      "recording": "scenario-01/capture.dat",
      "recording_sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
      "trials": [
        {
          "trial_id": "scenario-01-event-0001",
          "stratum": "moving-robot-two-speakers",
          "truth_source": "simulator_authoritative",
          "truth": {
            "observable": true,
            "pair": {"audio_id": "a1", "radar_id": "r1"}
          },
          "predictions": {
            "audio_only": {"decision": "abstain", "pair": null},
            "radar_only": {"decision": "abstain", "pair": null},
            "instant_fusion": {
              "decision": "pair",
              "pair": {"audio_id": "a1", "radar_id": "r1"}
            },
            "temporal_fusion": {
              "decision": "pair",
              "pair": {"audio_id": "a1", "radar_id": "r1"}
            }
          }
        }
      ]
    }
  ]
}
```

The digest above is an **illustrative placeholder** and must be replaced
with the SHA-256 of the actual referenced recording artifact. The file
could be a bundled recording/archive or another immutable artifact, as
long as its bytes match the declared digest. Path is relative to
`--artifacts-root`; traversal, absolute paths and symlinks are rejected.
Only explicitly simulated inputs and independently supplied truth source
`simulator_authoritative` are accepted.

All four predeclared comparison strategies must have an explicit
`pair`, `none`, or `abstain` decision in every trial. The importer
never fabricates missing models, truth, timestamps, distance, identities,
or replay results. Each trial ID is unique. One immutable recording
cannot appear in both development and held-out splits. The recording
hash is checked against actual bytes before conversion.

## Conversion and scoring

```bash
python3 audition_fusion/benchmarks/import_simulation_export.py \
  /data/simulation-export.json \
  --artifacts-root /data/recordings \
  --out /data/simulation-field-manifest.json

python3 audition_fusion/benchmarks/evaluate_field_manifest.py \
  /data/simulation-field-manifest.json > /data/simulation-report.json
```

The importer writes the existing strict `field_evaluation.py` manifest
schema. Standard scoring reports precision, recall, false matches,
abstentions and results by adversarial stratum. This is an **association
report**, not a measured estimate of acoustic range. Reports generated
from simulator exports must retain the `simulation` provenance separately
from real-world sensor measurement reports; do **not** combine totals into
a single claimed hardware accuracy number.

In the next simulator integration phase, export actual synchronized
audio/radar/camera/robot trajectories and predictions for every strategy
with explicit event-to-truth IDs. This v1 importer does not validate
timing, sensor calibration or raw sample accuracy because the format
contains trial-level decisions and artifact hashes, not those tracks.
These remain separate M4 gate requirements.

## Simulation provenance sidecar

The import command additionally writes
`simulation-field-manifest.json.provenance.json` next to the output.
It contains `domain: simulation`, `physical_validation_claim: false`,
`simulation_export_sha256`, `manifest_sha256` and the trial count.
**Keep this sidecar with every derived report.** Validate the latter
SHA-256 against the scored manifest before aggregation; results without
matching provenance should be marked `unknown-origin` and excluded
from physical-validation claims. The field v1 scorer intentionally
remains compatible with independently annotated real recordings; its
output schema alone does not classify simulated versus physical sources.

## Strict simulation-origin scoring (M4 provenance gate)

For a simulation-derived manifest, use the **fail-closed** evaluation mode:

```bash
python3 audition_fusion/benchmarks/evaluate_field_manifest.py \
  /data/simulation-field-manifest.json \
  --require-simulation-provenance > /data/simulation-report.json
```

This reads the adjacent `.provenance.json`, checks its exact schema, declared
simulation domain, explicit prohibition of physical-validation claims, trial
count, and SHA-256 of the scored manifest bytes. Missing, modified, malformed
or mismatched sidecars are an error (exit 2), not an implicit claim of physical
evidence. The report includes `data_domain: simulation`,
`provenance_verified: true`, and the export lineage digest. The digest
records lineage only; without external signatures it is **not** a proof of
independent sensor truth or tamper-proof external attestation.

For independently annotated physical field datasets, the existing evaluator
remains usable without this flag; such reports explicitly say
`data_domain: unverified` and `provenance_verified: false`. They are not
automatically physical evidence just because the simulator flag is absent.
Do not merge the simulated and physical metrics or interpret this contract
gate as validation of measured localization accuracy.
