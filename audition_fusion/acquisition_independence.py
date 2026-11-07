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


def paired_group_sensitivity(validated_trials, provenance_rows, *,
                             held_out_only=True):
    """Leave-one-*acquisition-group*-out M4 association comparison.

    Compute the three grouping dimensions separately; they are overlapping
    views of the same observations, not independent samples to pool.
    The caller must have cross-checked every provenance row and validated
    all trial decisions before calling this function.
    """
    from paired_comparison import BASELINES, CANDIDATE, empty, observe, summary

    def new_group():
        return {name: empty() for name in BASELINES}

    def new_stratum():
        return {field: defaultdict(new_group) for field in DIMENSIONS}

    def analyse(groups, baseline):
        ordered = [groups[key][baseline] for key in sorted(groups)]
        total = empty()
        for counts in ordered:
            for key in total:
                total[key] += counts[key]
        result = summary(total)
        out = {
            "acquisition_groups": len(ordered),
            "observable_trials": total["observable_trials"],
            "min_observable_trials_per_group": min(
                (v["observable_trials"] for v in ordered), default=None),
            "max_observable_trials_per_group": max(
                (v["observable_trials"] for v in ordered), default=None),
            "paired_accuracy_delta": result["paired_accuracy_delta"],
            "paired_false_pair_delta_per_trial":
                result["paired_false_pair_delta_per_trial"],
            "leave_one_group_out_accuracy_range": None,
            "leave_one_group_out_false_pair_range": None,
        }
        if len(ordered) >= 2:
            loo = []
            for removed in ordered:
                retained = {
                    key: total[key] - removed[key] for key in total
                }
                loo.append(summary(retained))
            # Unknown truth cannot be scored and never becomes a group.
            out["leave_one_group_out_accuracy_range"] = [
                min(row["paired_accuracy_delta"] for row in loo),
                max(row["paired_accuracy_delta"] for row in loo),
            ]
            out["leave_one_group_out_false_pair_range"] = [
                min(row["paired_false_pair_delta_per_trial"] for row in loo),
                max(row["paired_false_pair_delta_per_trial"] for row in loo),
            ]
        return out

    by_id = {row["trial_id"]: row for row in provenance_rows}
    groups = defaultdict(new_stratum)
    excluded = defaultdict(int)
    for tid, stratum, split, observable, truth, predictions in validated_trials:
        if held_out_only and split != "held_out":
            continue
        current = groups[stratum]  # Keep unknown-only strata visible.
        if not observable:
            excluded[stratum] += 1
            continue
        metadata = by_id[tid]
        for field in DIMENSIONS:
            bucket = current[field][metadata[field]]
            for baseline in BASELINES:
                observe(bucket[baseline], True, truth, predictions, baseline)

    return {
        "candidate": CANDIDATE,
        "scope": "held_out" if held_out_only else "all_splits_diagnostic",
        "per_stratum": {
            stratum: {
                "unobservable_excluded": excluded[stratum],
                "dimensions": {
                    field: {
                        "baselines": {
                            name: analyse(field_groups, name)
                            for name in BASELINES
                        }
                    }
                    for field, field_groups in sorted(stratum_groups.items())
                },
            }
            for stratum, stratum_groups in sorted(groups.items())
        },
        "acquisition_group_labels_authenticated": False,
        "physical_validation_claim": False,
        "interpretation": (
            "Leave-one-session, scene or emitter-out is a descriptive "
            "stress test on supplied identifiers, not a confidence interval, "
            "a demonstration of statistical independence, or field acceptance."),
    }


def group_robustness_blockers(analysis):
    """Require each sufficient-sized stratum's gain to survive group removal.

    Small groups are already rejected by independence_blockers(). This
    separate diagnostic gate deliberately forbids a positive result if one
    acquisition group accounts for all of a claimed advantage.
    """
    blockers = []
    for stratum, data in analysis["per_stratum"].items():
        for dimension, payload in data["dimensions"].items():
            for baseline, stats in payload["baselines"].items():
                if stats["acquisition_groups"] < MIN_GROUPS:
                    continue
                gain = stats["leave_one_group_out_accuracy_range"]
                false_pairs = stats["leave_one_group_out_false_pair_range"]
                if gain is None or gain[0] <= 0:
                    blockers.append(
                        "fragile_acquisition_gain:" + stratum + ":" +
                        dimension + ":" + baseline)
                if false_pairs is None or false_pairs[1] > 0:
                    blockers.append(
                        "fragile_acquisition_false_pairs:" + stratum + ":" +
                        dimension + ":" + baseline)
    return sorted(set(blockers))
