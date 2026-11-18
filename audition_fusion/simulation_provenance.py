"""Verify simulator provenance sidecars before scoring simulation-derived manifests.

The field evaluator remains usable for independently annotated physical data.
A simulated dataset must opt into strict origin verification; no metadata is
inferred from the filename or from an apparently plausible score.
"""
import hashlib
import json
import pathlib
import re

SHA256 = re.compile(r"[0-9a-f]{64}\Z")
MAX_SIDECAR = 64 * 1024


def strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate provenance JSON key: " + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError("nonfinite provenance number: " + value)


def verified_simulation_provenance(manifest_path, manifest_bytes, dataset):
    """Validate the adjacent immutable import record; return safe report fields.

    The export hash records lineage but is not independently authenticated by
    this sidecar. It must not be represented as an external attestation.
    """
    path = pathlib.Path(manifest_path)
    sidecar = path.with_name(path.name + ".provenance.json")
    raw = sidecar.read_bytes()
    if len(raw) > MAX_SIDECAR:
        raise ValueError("simulation provenance sidecar too large")
    data = json.loads(raw.decode("utf-8"), object_pairs_hook=strict_object,
                      parse_constant=reject_constant)
    if not isinstance(data, dict) or set(data) != {
            "domain", "physical_validation_claim", "simulation_export_sha256",
            "manifest_sha256", "trials"}:
        raise ValueError("invalid simulation provenance schema")
    if data["domain"] != "simulation" or data["physical_validation_claim"] is not False:
        raise ValueError("simulation provenance must explicitly forbid physical claims")
    for name in ("manifest_sha256", "simulation_export_sha256"):
        value = data[name]
        if not isinstance(value, str) or SHA256.fullmatch(value) is None:
            raise ValueError("invalid provenance digest: " + name)
    if data["manifest_sha256"] != hashlib.sha256(manifest_bytes).hexdigest():
        raise ValueError("simulation provenance manifest digest mismatch")
    if type(data["trials"]) is not int or data["trials"] != len(dataset["trials"]):
        raise ValueError("simulation provenance trial count mismatch")
    return {
        "data_domain": "simulation",
        "provenance_verified": True,
        "simulation_export_sha256": data["simulation_export_sha256"],
        "provenance_sidecar_name": sidecar.name,
    }
