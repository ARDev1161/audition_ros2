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
    # Count independent claimed acquisition groups separately for positive
    # and negative observable held-out truth; trial/recording counts cannot
    # substitute for scene/session diversity.
    by_trial = {row["trial_id"]: row for row in metadata["trials"]}
    groups = {}
    for trial in dataset["trials"]:
        if trial["split"] != "held_out" or not trial["truth"]["observable"]:
            continue
        kind = "negative" if trial["truth"]["pair"] is None else "positive"
        entry = groups.setdefault(trial["stratum"], {}).setdefault(kind, {
            "session_id": set(), "scene_id": set(), "subject_id": set()})
        source = by_trial[trial["trial_id"]]
        for field in entry:
            entry[field].add(source[field])
    class_group_counts = {
        stratum: {kind: {field: len(values) for field, values in fields.items()}
                  for kind, fields in classes.items()}
        for stratum, classes in groups.items()}
    # Derive scene-level false-pair exposure from the same validated bytes.
    # Never re-open the provenance file after checking its SHA and trial IDs:
    # the path could be replaced between validation and scoring (TOCTOU).
    false_pair_scenes = {}
    for trial in dataset["trials"]:
        if trial["split"] != "held_out" or not trial["truth"]["observable"]:
            continue
        prediction = trial["predictions"]["temporal_fusion"]
        if prediction["decision"] == "pair" and prediction["pair"] != trial["truth"]["pair"]:
            false_pair_scenes.setdefault(trial["stratum"], set()).add(
                by_trial[trial["trial_id"]]["scene_id"])
    false_pair_scene_counts = {
        stratum: len(scenes) for stratum, scenes in sorted(false_pair_scenes.items())}
    # Class-conditional scene exposure: a single negative scene must not be
    # diluted by many positive scenes in the same adverse stratum.
    scene_exposure = {}
    for trial in dataset["trials"]:
        if trial["split"] != "held_out" or not trial["truth"]["observable"]:
            continue
        kind = "negative" if trial["truth"]["pair"] is None else "positive"
        group = scene_exposure.setdefault(trial["stratum"], {}).setdefault(
            kind, {"observable": set(), "false_pair": set()})
        scene = by_trial[trial["trial_id"]]["scene_id"]
        group["observable"].add(scene)
        candidate = trial["predictions"]["temporal_fusion"]
        if candidate["decision"] == "pair" and candidate["pair"] != trial["truth"]["pair"]:
            group["false_pair"].add(scene)
    scene_class_counts = {
        name: {kind: {"observable_scenes": len(groups["observable"]),
                      "false_pair_scenes": len(groups["false_pair"])}
               for kind, groups in classes.items()}
        for name, classes in scene_exposure.items()}
    return {
        "held_out_scene_truth_class_exposure": scene_class_counts,
        "held_out_false_pair_scene_counts": false_pair_scene_counts,
        "held_out_truth_class_groups": class_group_counts,
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
