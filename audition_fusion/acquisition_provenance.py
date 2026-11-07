"""Strict cross-check for external acquisition metadata against the M4 trial manifest.

The sidecar is independently *supplied*, NOT cryptographically attested.
An accurate hash of this metadata proves reproducibility, not authenticity.
Missing, mismatched or split-leaking records must fail closed.
"""
import hashlib
import json
import pathlib

from safety_diagnostic import provenance_blockers
from acquisition_independence import summarize, paired_group_sensitivity
from field_evaluation import validate

MAX_AUDIT_BYTES = 4 * 1024 * 1024


def strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate acquisition provenance JSON key: " + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError("nonfinite acquisition provenance number: " + value)


def audit_acquisition_provenance(path, dataset):
    """Return a reproducible audit only if every trial exactly matches.

    All development and held-out rows must be present; an audit restricted to
    held-out rows cannot prove that scene/session/subject split leakage is absent.
    """
    raw = pathlib.Path(path).read_bytes()
    if len(raw) > MAX_AUDIT_BYTES:
        raise ValueError("acquisition provenance exceeds 4 MiB")
    metadata = json.loads(
        raw.decode("utf-8"), object_pairs_hook=strict_object,
        parse_constant=reject_constant)
    issues = provenance_blockers(metadata)
    if issues:
        raise ValueError("invalid acquisition provenance: " + ", ".join(issues))
    expected = {
        row["trial_id"]: (row["split"], row["recording_sha256"])
        for row in dataset["trials"]
    }
    observed = {
        row["trial_id"]: (row["split"], row["recording_sha256"])
        for row in metadata["trials"]
    }
    if set(expected) != set(observed):
        raise ValueError("acquisition provenance trial ID set mismatch")
    mismatches = [
        tid for tid in sorted(expected) if observed[tid] != expected[tid]
    ]
    if mismatches:
        raise ValueError("acquisition provenance trial split/hash mismatch: "
                         + ", ".join(mismatches[:8]))
    return {
        "status": "crosschecked",
        "crosscheck_passed": True,
        "trial_count": len(expected),
        "independence_summary": summarize(dataset, metadata["trials"]),
        "paired_group_sensitivity": paired_group_sensitivity(
            validate(dataset), metadata["trials"]),
        "provenance_file_name": pathlib.Path(path).name,
        "provenance_sha256": hashlib.sha256(raw).hexdigest(),
        "independently_authenticated": False,
        "physical_validation_claim": False,
        "note": ("All manifest trial IDs/splits/hashes match the supplied "
                 "metadata; split leakage was not seen in its claimed "
                 "session/scene/subject IDs. Data origin is not authenticated."),
    }
