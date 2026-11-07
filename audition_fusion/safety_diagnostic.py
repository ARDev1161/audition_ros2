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
