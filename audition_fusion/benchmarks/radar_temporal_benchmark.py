#!/usr/bin/env python3
"""M4-D paired instantaneous-vs-temporal synthetic evaluation.

Fixtures deliberately include physically indistinguishable impostors.
This benchmark establishes reproducible regression counts, NOT a measured
gain in real-world source association or acoustic ranging.
"""
from dataclasses import replace
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from radar_bearing_matching import RadarBearingMatcher
from radar_temporal import RadarPairTemporalGate
from benchmarks.radar_association_benchmark import (
    CLASSES, fixture,
)

FRAMES = 3
STEP_NS = 150_000_000
CASES = tuple(CLASSES) + ("negative_one_frame_coincidence",)


def score(truth, predicted):
    if truth:
        return "tp" if predicted else "fn"
    return "fp" if predicted else "tn"


def evaluate():
    matcher = RadarBearingMatcher()
    overall = {
        strategy: {"tp": 0, "fp": 0, "tn": 0, "fn": 0}
        for strategy in ("single_frame_any", "three_independent_updates")
    }
    per_class = {}
    for kind in CASES:
        entry = {key: dict.fromkeys(("tp", "fp", "tn", "fn"), 0)
                 for key in overall}
        for seed in range(20):
            # Input and ground truth must remain independent; changing
            # predictions cannot change the labels of these fixtures.
            base_kind = ("true_direct" if kind ==
                         "negative_one_frame_coincidence" else kind)
            audios, radars, _ = fixture(base_kind, seed)
            truth = kind.startswith("true_")
            gate = RadarPairTemporalGate()
            single = False
            temporal = False
            for frame in range(FRAMES):
                offset = frame * STEP_NS
                observed_audio = [
                    replace(a, stamp_ns=a.stamp_ns + offset)
                    for a in audios
                ]
                observed_radar = [
                    replace(r, stamp_ns=r.stamp_ns + offset)
                    for r in radars
                ]
                if kind == "negative_one_frame_coincidence" and frame > 0:
                    observed_radar = [
                        replace(r, position=(0.0, 3.0, 0.0))
                        for r in observed_radar
                    ]
                candidates = matcher.propose(observed_audio, observed_radar)
                single |= any(c.unique_match for c in candidates)
                evidence = gate.observe(
                    candidates,
                    audio_session="a" * 32,
                    radar_session="b" * 32,
                    audio_stamp_ns=observed_audio[0].stamp_ns,
                    radar_stamp_ns=observed_radar[0].stamp_ns,
                    clock_ns=observed_audio[0].stamp_ns + 10_000_000,
                )
                temporal |= any(
                    e.temporally_correlated for e in evidence.values()
                )
            for strategy, predicted in (
                ("single_frame_any", single),
                ("three_independent_updates", temporal),
            ):
                key = score(truth, predicted)
                entry[strategy][key] += 1
                overall[strategy][key] += 1
        per_class[kind] = entry
    result = {
        "scenario_count": len(CASES) * 20,
        "frames_per_scenario": FRAMES,
        "ground_truth": "hand-authored synthetic fixture labels",
        "measured_physical_accuracy": False,
        "per_class": per_class,
        "strategies": {},
    }
    for strategy, counts in overall.items():
        predicted_positive = counts["tp"] + counts["fp"]
        positives = counts["tp"] + counts["fn"]
        negatives = counts["tn"] + counts["fp"]
        result["strategies"][strategy] = {
            "confusion": counts,
            "precision": counts["tp"] / predicted_positive if predicted_positive else None,
            "recall": counts["tp"] / positives if positives else None,
            "false_positive_rate": counts["fp"] / negatives if negatives else None,
        }
    # These fixed counts are an intentionally *limited* regression oracle.
    # Reflections and perfectly aligned impostors survive the temporal gate.
    expected = {
        "single_frame_any": {"tp": 40, "fp": 60, "tn": 80, "fn": 0},
        "three_independent_updates": {"tp": 40, "fp": 40, "tn": 100, "fn": 0},
    }
    for strategy, counts in expected.items():
        assert result["strategies"][strategy]["confusion"] == counts, result
    assert per_class["negative_one_frame_coincidence"][
        "three_independent_updates"]["tn"] == 20
    for name in ("negative_aligned_impostor", "negative_aligned_reflection"):
        assert per_class[name]["three_independent_updates"]["fp"] == 20
    return result


if __name__ == "__main__":
    print(json.dumps(evaluate(), indent=2, sort_keys=True))
