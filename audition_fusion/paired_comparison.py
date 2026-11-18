"""Paired diagnostic comparison of M4 association decisions on identical trials.

This is descriptive evidence only. Repeated trials from one recording may be
correlated; no p-values, independent-trial confidence intervals, or physical
validation claims are derived from these counts.
"""
from collections import defaultdict

BASELINES = ("audio_only", "radar_only", "instant_fusion")
CANDIDATE = "temporal_fusion"


def correct(truth_pair, prediction):
    """An abstention is never success; a verified negative requires 'none'."""
    decision, proposed = prediction
    if decision == "abstain":
        return False
    if truth_pair is None:
        return decision == "none"
    return decision == "pair" and proposed == truth_pair


def empty():
    return {"both_correct": 0, "candidate_only_correct": 0,
            "baseline_only_correct": 0, "both_incorrect": 0,
            "observable_trials": 0, "unobservable_excluded": 0,
            "candidate_false_pairs": 0, "baseline_false_pairs": 0}


def observe(counts, observable, truth_pair, predictions, baseline):
    if not observable:
        counts["unobservable_excluded"] += 1
        return
    counts["observable_trials"] += 1
    candidate = predictions[CANDIDATE]
    reference = predictions[baseline]
    a, b = correct(truth_pair, candidate), correct(truth_pair, reference)
    key = ("both_correct" if a and b else
           "candidate_only_correct" if a else
           "baseline_only_correct" if b else "both_incorrect")
    counts[key] += 1
    # An emitted wrong pair is a false association even on positive truth.
    counts["candidate_false_pairs"] += int(
        candidate[0] == "pair" and candidate[1] != truth_pair)
    counts["baseline_false_pairs"] += int(
        reference[0] == "pair" and reference[1] != truth_pair)


def summary(counts):
    result = dict(counts)
    n = counts["observable_trials"]
    result["paired_accuracy_delta"] = (
        (counts["candidate_only_correct"] - counts["baseline_only_correct"]) / n
        if n else None)
    result["paired_false_pair_delta_per_trial"] = (
        (counts["candidate_false_pairs"] - counts["baseline_false_pairs"]) / n
        if n else None)
    result["descriptive_improvement"] = (
        counts["candidate_only_correct"] > counts["baseline_only_correct"]
        if n else None)
    return result


def compare(selected_trials):
    overall = {name: empty() for name in BASELINES}
    strata = defaultdict(lambda: {name: empty() for name in BASELINES})
    for _, stratum, _, observable, truth, predictions in selected_trials:
        for name in BASELINES:
            observe(overall[name], observable, truth, predictions, name)
            observe(strata[stratum][name], observable, truth, predictions, name)
    return {
        "candidate": CANDIDATE,
        "baseline_comparisons": {name: summary(c) for name, c in overall.items()},
        "per_stratum": {stratum: {name: summary(c) for name, c in group.items()}
                       for stratum, group in sorted(strata.items())},
        "positive_gain_validated": False,
        "comparison_note": (
            "Trial-paired descriptive accuracy and false-association deltas; "
            "unknown truth excluded, abstention not correct. "
            "Not independent recording-level inference or field validation."),
    }
