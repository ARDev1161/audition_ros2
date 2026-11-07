"""Fail-closed *diagnostic* assessment of held-out M4 association evidence.

This checks whether a pooled gain is hiding a false-pair regression in any
observable adverse stratum. It NEVER authorizes physical localization, a
LocalizedSource topic, or Nav2 integration, even on clean synthetic trials.
"""
from paired_comparison import BASELINES


def assess(report):
    paired = report["paired_comparison"]
    clustered = report["clustered_comparison"]
    blockers = []
    if report["evaluation_split"] != "held_out":
        blockers.append("development_data_included")
    for stratum, methods in sorted(paired["per_stratum"].items()):
        cluster_info = clustered["per_stratum"][stratum]
        for baseline in BASELINES:
            counts = methods[baseline]
            cluster_stats = cluster_info["baselines"][baseline]
            n = counts["observable_trials"]
            if n == 0:
                blockers.append("unknown_truth:" + stratum + ":" + baseline)
                continue
            if counts["candidate_false_pairs"] > counts["baseline_false_pairs"]:
                blockers.append("false_pair_regression:" + stratum + ":" + baseline)
            if counts["candidate_only_correct"] < counts["baseline_only_correct"]:
                blockers.append("paired_accuracy_regression:" + stratum + ":" + baseline)
            if cluster_stats["recording_clusters"] < 5:
                blockers.append("insufficient_recording_clusters:" + stratum + ":" + baseline)
    for baseline in BASELINES:
        overall = paired["baseline_comparisons"][baseline]
        if overall["observable_trials"] == 0:
            blockers.append("no_observable_trials:" + baseline)
        elif overall["candidate_only_correct"] <= overall["baseline_only_correct"]:
            blockers.append("no_pooled_accuracy_gain:" + baseline)
    if not paired["per_stratum"]:
        blockers.append("no_evaluable_strata")
    return {
        "scope": "held_out_trial_diagnostics_only",
        "candidate": paired["candidate"],
        "compared_baselines": list(BASELINES),
        "blockers": sorted(set(blockers)),
        "no_observed_safety_regression": not any(
            reason.startswith(("false_pair_regression:", "paired_accuracy_regression:"))
            for reason in blockers),
        "diagnostic_promotion_eligible": not blockers,
        "physical_localization_accepted": False,
        "nav2_admission": False,
        "interpretation": (
            "No observed regressions is not proof of safety. Independent "
            "field truth, sensor calibration, scene/subject/session splits "
            "and prespecified acceptance thresholds remain mandatory."),
    }


def provenance_blockers(manifest):
    """Reject leakage across acquisition sessions, scenes and physical emitters.

    This is an optional external audit manifest. Missing or contradictory
    provenance always blocks diagnostic promotion; it never upgrades safety.
    """
    if not isinstance(manifest, dict) or set(manifest) != {"trials"} or not isinstance(manifest["trials"], list):
        return ["provenance_manifest_missing_or_invalid"]
    seen = set()
    recording_sessions = {}
    splits = {"recording_sha256": {}, "session_id": {}, "scene_id": {}, "subject_id": {}}
    problems = []
    for row in manifest["trials"]:
        if not isinstance(row, dict) or set(row) != {"trial_id", "split", *splits}:
            problems.append("provenance_trial_invalid")
            continue
        tid = row["trial_id"]
        if not isinstance(tid, str) or not tid or tid in seen:
            problems.append("provenance_trial_duplicate_or_invalid")
        else:
            seen.add(tid)
        split = row["split"]
        if split not in ("development", "held_out"):
            problems.append("provenance_split_invalid")
            continue
        for field, groups in splits.items():
            value = row[field]
            if not isinstance(value, str) or not value.strip():
                problems.append("provenance_missing:" + field)
                continue
            if field == "recording_sha256" and (len(value) != 64 or any(c not in "0123456789abcdef" for c in value)):
                problems.append("provenance_invalid_recording_hash")
            previous = groups.setdefault(value, split)
            if previous != split:
                problems.append("provenance_split_leak:" + field)
        # One immutable recording cannot originate from two acquisition
        # sessions. Subjects may legitimately differ within a recording.
        digest, session = row["recording_sha256"], row["session_id"]
        if (isinstance(digest, str) and isinstance(session, str)
                and len(digest) == 64 and session.strip()):
            existing = recording_sessions.setdefault(digest, session)
            if existing != session:
                problems.append("provenance_recording_session_conflict")
    if not seen:
        problems.append("provenance_no_trials")
    return sorted(set(problems))
