"""Strict M4 independent-ground-truth *trial-level* association scorer.

No ROS dependency. Labels are supplied by independent annotators, never
inferred from geometry, detector outputs, or another method's predictions.
A wrong pair on a positive trial contributes both FP and FN; they represent
two separate errors, not two trials. Abstention is not a true negative.
"""
from collections import defaultdict
import math
import re

STRATEGIES = (
    "audio_only", "radar_only", "instant_fusion", "temporal_fusion"
)
DECISIONS = ("pair", "none", "abstain")
ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:/-]{0,127}\Z")
HASH_RE = re.compile(r"[0-9a-f]{64}\Z")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def identity(value, name):
    require(isinstance(value, str) and ID_RE.fullmatch(value) is not None,
            "invalid " + name)
    return value


def pair(value, name):
    require(isinstance(value, dict)
            and set(value) == {"audio_id", "radar_id"},
            "invalid " + name)
    return (identity(value["audio_id"], name + ".audio_id"),
            identity(value["radar_id"], name + ".radar_id"))


def validate(data):
    require(isinstance(data, dict)
            and set(data) == {"schema_version", "dataset_id", "trials"},
            "expected schema_version, dataset_id and trials")
    require(type(data["schema_version"]) is int
            and data["schema_version"] == 1, "unsupported schema")
    identity(data["dataset_id"], "dataset_id")
    trials = data["trials"]
    require(isinstance(trials, list) and 1 <= len(trials) <= 100000,
            "invalid trial count")
    seen = set()
    recording_splits = {}
    result = []
    for i, trial in enumerate(trials):
        name = "trials[" + str(i) + "]"
        require(isinstance(trial, dict) and set(trial) == {
            "trial_id", "stratum", "split", "recording_sha256",
            "truth", "predictions"
        }, "invalid " + name + " keys")
        tid = identity(trial["trial_id"], name + ".trial_id")
        require(tid not in seen, "duplicate trial_id " + tid)
        seen.add(tid)
        stratum = identity(trial["stratum"], name + ".stratum")
        require(trial["split"] in ("development", "held_out"),
                "invalid trial split")
        recording_hash = trial["recording_sha256"]
        require(isinstance(recording_hash, str)
                and HASH_RE.fullmatch(recording_hash) is not None,
                "invalid recording sha256")
        previous_split = recording_splits.setdefault(
            recording_hash, trial["split"])
        require(previous_split == trial["split"],
                "recording appears in development and held_out splits")
        truth = trial["truth"]
        require(isinstance(truth, dict)
                and set(truth) == {"observable", "pair"},
                "invalid truth")
        require(type(truth["observable"]) is bool,
                "observable must be boolean")
        true_pair = (None if truth["pair"] is None else
                     pair(truth["pair"], name + ".truth.pair"))
        require(truth["observable"] or true_pair is None,
                "unobservable ground truth cannot contain a pair")
        require(isinstance(trial["predictions"], dict)
                and set(trial["predictions"]) == set(STRATEGIES),
                "all comparison strategies required in every trial")
        predictions = {}
        for strategy in STRATEGIES:
            d = trial["predictions"][strategy]
            require(isinstance(d, dict) and set(d) == {"decision", "pair"},
                    "invalid prediction " + strategy)
            decision = d["decision"]
            require(isinstance(decision, str) and decision in DECISIONS,
                    "invalid decision " + strategy)
            predicted_pair = (None if d["pair"] is None else
                              pair(d["pair"], name + "." + strategy))
            require((decision == "pair") == (predicted_pair is not None),
                    "pair is required iff decision=pair")
            predictions[strategy] = (decision, predicted_pair)
        result.append((tid, stratum, trial["split"],
                       truth["observable"], true_pair, predictions))
    return result


def empty():
    return {"tp": 0, "fp": 0, "tn": 0, "fn": 0,
            "abstained": 0, "unobservable": 0, "trials": 0}


def increment(counts, observable, true_pair, decision, proposed_pair):
    counts["trials"] += 1
    if not observable:
        counts["unobservable"] += 1
        # Never force an unknown real-world identity into a negative label.
        return
    if decision == "abstain":
        counts["abstained"] += 1
        if true_pair is not None:
            counts["fn"] += 1
        return
    if proposed_pair is not None and proposed_pair == true_pair:
        counts["tp"] += 1
    elif proposed_pair is not None:
        counts["fp"] += 1
        if true_pair is not None:
            counts["fn"] += 1
    elif true_pair is None:
        counts["tn"] += 1
    else:
        counts["fn"] += 1


def summarise(c):
    c = dict(c)
    def ratio(n, d):
        return n / d if d else None
    c["precision"] = ratio(c["tp"], c["tp"] + c["fp"])
    c["recall"] = ratio(c["tp"], c["tp"] + c["fn"])
    c["false_positive_rate"] = ratio(c["fp"], c["fp"] + c["tn"])
    p, r = c["precision"], c["recall"]
    c["f1"] = (2*p*r/(p+r) if p is not None and r is not None
               and p+r > 0 else None)
    c["abstention_rate_observable"] = ratio(
        c["abstained"], c["trials"] - c["unobservable"])
    return c


def evaluate(data, *, held_out_only=True):
    trials = validate(data)
    selected = [t for t in trials if not held_out_only or t[2] == "held_out"]
    require(selected, "no eligible trials")
    overall = {name: empty() for name in STRATEGIES}
    strata = defaultdict(lambda: {name: empty() for name in STRATEGIES})
    for tid, stratum, split, observable, truth, predictions in selected:
        for name, (decision, proposed) in predictions.items():
            increment(overall[name], observable, truth, decision, proposed)
            increment(strata[stratum][name], observable, truth,
                      decision, proposed)
    return {
        "dataset_id": data["dataset_id"],
        "schema_version": 1,
        "evaluation_split": "held_out" if held_out_only else "all",
        "trial_count": len(selected),
        "scoring_note": ("wrong positive pair yields FP and FN; unknown "
                         "truth excluded; abstentions are never TN"),
        "strategies": {k: summarise(v) for k, v in overall.items()},
        "per_stratum": {
            k: {name: summarise(v) for name,v in group.items()}
            for k, group in sorted(strata.items())
        },
        "physical_validation_claim": False,
    }
