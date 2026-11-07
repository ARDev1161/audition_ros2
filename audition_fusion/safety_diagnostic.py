"""Fail-closed *diagnostic* assessment of held-out M4 association evidence.

This checks whether a pooled gain is hiding a false-pair regression in any
observable adverse stratum. It NEVER authorizes physical localization, a
LocalizedSource topic, or Nav2 integration, even on clean synthetic trials.
"""
import re
from paired_comparison import BASELINES

PROVENANCE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}\Z")


def assess(report):
    paired = report["paired_comparison"]
    clustered = report["clustered_comparison"]
    blockers = []
    if report["evaluation_split"] != "held_out":
        blockers.append("development_data_included")
    for stratum, methods in sorted(paired["per_stratum"].items()):
        # A partly unobservable stratum must not be described as fully
        # evaluated: missing independent truth is not a negative label.
        if report["per_stratum"][stratum][BASELINES[0]]["unobservable"] > 0:
            blockers.append("unobservable_ground_truth:" + stratum)
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
        # This is an evidence-qualified statement, not merely a comparison
        # of the observable subset. Insufficient clusters, missing truth or
        # development leakage cannot support a clean safety claim.
        "no_observed_safety_regression": not blockers,
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
    recording_scenes = {}
    splits = {"recording_sha256": {}, "session_id": {}, "scene_id": {}, "subject_id": {}}
    problems = []
    for row in manifest["trials"]:
        if not isinstance(row, dict) or set(row) != {"trial_id", "split", *splits}:
            problems.append("provenance_trial_invalid")
            continue
        tid = row["trial_id"]
        if not isinstance(tid, str) or PROVENANCE_ID.fullmatch(tid) is None or tid in seen:
            problems.append("provenance_trial_duplicate_or_invalid")
        else:
            seen.add(tid)
        split = row["split"]
        if not isinstance(split, str) or split not in ("development", "held_out"):
            problems.append("provenance_split_invalid")
            continue
        for field, groups in splits.items():
            value = row[field]
            if not isinstance(value, str) or not value.strip():
                problems.append("provenance_missing:" + field)
                continue
            if field == "recording_sha256" and (len(value) != 64 or any(c not in "0123456789abcdef" for c in value)):
                problems.append("provenance_invalid_recording_hash")
            if field != "recording_sha256" and PROVENANCE_ID.fullmatch(value) is None:
                problems.append("provenance_invalid_identifier:" + field)
                continue
            previous = groups.setdefault(value, split)
            if previous != split:
                problems.append("provenance_split_leak:" + field)
        # One immutable recording cannot originate from different acquisition
        # sessions or scenes. Subjects may legitimately differ within a recording.
        digest, session = row["recording_sha256"], row["session_id"]
        scene = row["scene_id"]
        if (isinstance(digest, str) and isinstance(session, str)
                and len(digest) == 64 and session.strip()):
            existing = recording_sessions.setdefault(digest, session)
            if existing != session:
                problems.append("provenance_recording_session_conflict")
            if isinstance(scene, str) and scene.strip():
                previous_scene = recording_scenes.setdefault(digest, scene)
                if previous_scene != scene:
                    problems.append("provenance_recording_scene_conflict")
    if not seen:
        problems.append("provenance_no_trials")
    return sorted(set(problems))
