"""Recording-clustered sensitivity of paired M4 association comparisons.

Deterministic cluster resampling explores sampling variability only; scene,
subject and acquisition-session leakage are not addressed by SHA-256 grouping.
No report from this module is a physical localization acceptance claim.
"""
import random
from collections import defaultdict

from paired_comparison import BASELINES, CANDIDATE, correct

BOOTSTRAP_DRAWS = 512
BOOTSTRAP_SEED = 20261107
MIN_CLUSTERS_FOR_INTERVAL = 5


def _stats(items):
    n = wins = losses = candidate_fp = baseline_fp = 0
    for truth, candidate, baseline in items:
        n += 1
        a, b = correct(truth, candidate), correct(truth, baseline)
        wins += int(a and not b)
        losses += int(b and not a)
        candidate_fp += int(candidate[0] == "pair" and candidate[1] != truth)
        baseline_fp += int(baseline[0] == "pair" and baseline[1] != truth)
    return (n, wins - losses, candidate_fp - baseline_fp)


def _add(values):
    return tuple(sum(x) for x in zip(*values))


def _rates(stats):
    return (stats[1] / stats[0], stats[2] / stats[0]) if stats[0] else (None, None)


def _percentile(samples, rank):
    # Fixed-order, nearest-rank empirical bootstrap percentile, deterministic.
    return sorted(samples)[max(0, min(len(samples) - 1,
                                     int(rank * (len(samples) - 1))))]


def _interval(samples):
    return [_percentile(samples, .025), _percentile(samples, .975)]


def _analyze(groups, baseline):
    clusters = []
    for digest in sorted(groups):
        samples = [(truth, predictions[CANDIDATE], predictions[baseline])
                   for truth, predictions in groups[digest]]
        clusters.append(_stats(samples))
    aggregate = _add(clusters)
    accuracy, false_pair = _rates(aggregate)
    count = len(clusters)
    out = {
        "recording_clusters": count,
        "observable_trials": aggregate[0],
        "min_observable_trials_per_recording": min((x[0] for x in clusters), default=None),
        "max_observable_trials_per_recording": max((x[0] for x in clusters), default=None),
        "paired_accuracy_delta": accuracy,
        "paired_false_pair_delta_per_trial": false_pair,
        "leave_one_recording_out_accuracy_range": None,
        "leave_one_recording_out_false_pair_range": None,
        "bootstrap_95pct_accuracy_interval": None,
        "bootstrap_95pct_false_pair_interval": None,
        "bootstrap_seed": BOOTSTRAP_SEED,
        "bootstrap_draws": BOOTSTRAP_DRAWS,
    }
    if count >= 2 and all(x[0] > 0 for x in clusters):
        loo = [_rates(_add(clusters[:i] + clusters[i+1:]))
               for i in range(count)]
        out["leave_one_recording_out_accuracy_range"] = [
            min(v[0] for v in loo), max(v[0] for v in loo)]
        out["leave_one_recording_out_false_pair_range"] = [
            min(v[1] for v in loo), max(v[1] for v in loo)]
    if count >= MIN_CLUSTERS_FOR_INTERVAL and all(x[0] > 0 for x in clusters):
        rng = random.Random(BOOTSTRAP_SEED)
        accuracy_samples, false_pair_samples = [], []
        for _ in range(BOOTSTRAP_DRAWS):
            sampled = _add([clusters[rng.randrange(count)] for _ in range(count)])
            a, fp = _rates(sampled)
            accuracy_samples.append(a)
            false_pair_samples.append(fp)
        out["bootstrap_95pct_accuracy_interval"] = _interval(accuracy_samples)
        out["bootstrap_95pct_false_pair_interval"] = _interval(false_pair_samples)
    return out


def clustered_comparison(data, validated_trials, *, held_out_only=True):
    """Group *validated* trial decisions by immutable recording digest.

    Trials have the same input order as the original manifest; validate()
    checks this alignment, trial identity, splits and digest format first.
    """
    assert len(data["trials"]) == len(validated_trials)
    groups = defaultdict(list)
    excluded_unknown = 0
    for item, trial in zip(data["trials"], validated_trials):
        _, _, split, observable, truth, predictions = trial
        if held_out_only and split != "held_out":
            continue
        if not observable:
            excluded_unknown += 1
            continue
        groups[item["recording_sha256"]].append((truth, predictions))
    return {
        "candidate": CANDIDATE,
        "baselines": {name: _analyze(groups, name) for name in BASELINES},
        "unobservable_excluded": excluded_unknown,
        "recording_independence_only": True,
        "physical_validation_claim": False,
        "interpretation": (
            "Exploratory recording-cluster sensitivity, not a physical "
            "validation gate. Shared rooms, subjects and sessions can remain "
            "dependent; few recordings or zero observable trials yield null "
            "uncertainty bounds."),
    }
