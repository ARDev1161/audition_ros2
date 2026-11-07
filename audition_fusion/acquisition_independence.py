"""Conservative acquisition-level independence counts for held-out M4 trials.

Recording files are not necessarily independent sessions or rooms. This
module counts supplied, cross-checked identifiers but NEVER authenticates
them and NEVER estimates physical localization performance.
"""
from collections import defaultdict

DIMENSIONS = ("session_id", "scene_id", "subject_id")
MIN_GROUPS = 5


def summarize(dataset, provenance_rows, *, held_out_only=True):
    by_id = {row["trial_id"]: row for row in provenance_rows}
    overall = {field: set() for field in DIMENSIONS}
    by_stratum = defaultdict(lambda: {
        "groups": {field: set() for field in DIMENSIONS},
        "observable_trials": 0,
        "unknown_truth_excluded": 0,
    })
    unknown = 0
    eligible = 0
    for trial in dataset["trials"]:
        if held_out_only and trial["split"] != "held_out":
            continue
        stratum = trial["stratum"]
        group = by_stratum[stratum]
        if not trial["truth"]["observable"]:
            group["unknown_truth_excluded"] += 1
            unknown += 1
            continue
        eligible += 1
        group["observable_trials"] += 1
        row = by_id[trial["trial_id"]]
        for field in DIMENSIONS:
            group["groups"][field].add(row[field])
            overall[field].add(row[field])
    return {
        "scope": "held_out_only" if held_out_only else "all_splits_diagnostic",
        "min_groups_for_diagnostic": MIN_GROUPS,
        "observable_trials": eligible,
        "unknown_truth_excluded": unknown,
        "overall_distinct_groups": {name: len(overall[name]) for name in DIMENSIONS},
        "per_stratum": {
            name: {
                "observable_trials": value["observable_trials"],
                "unknown_truth_excluded": value["unknown_truth_excluded"],
                "distinct_groups": {
                    field: len(value["groups"][field]) for field in DIMENSIONS
                },
            }
            for name, value in sorted(by_stratum.items())
        },
        "groups_are_unverified_claims": True,
        "physical_validation_claim": False,
        "interpretation": (
            "Count supplied session, scene and emitter groups independently; "
            "five recording hashes never substitute for five distinct rooms, "
            "sessions or emitters. Metadata origin is not authenticated."),
    }


def independence_blockers(summary):
    blockers = []
    for stratum, result in summary["per_stratum"].items():
        if not result["observable_trials"]:
            continue  # unknown truth has its own fail-closed blocker
        for field, count in sorted(result["distinct_groups"].items()):
            if count < MIN_GROUPS:
                blockers.append("insufficient_acquisition_groups:" +
                                stratum + ":" + field)
    if summary["observable_trials"] == 0:
        blockers.append("no_observable_acquisition_truth")
    return sorted(blockers)
