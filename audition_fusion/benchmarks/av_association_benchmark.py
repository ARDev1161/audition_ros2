#!/usr/bin/env python3
"""Reproducible, deliberately adversarial *synthetic* AV association benchmark.

This benchmark is NOT evidence of real-world accuracy. It includes two
unidentifiable hard negatives that the geometry + temporal policy *must*
false-match, making remaining limitations measurable rather than hidden.
"""
import json
import math
import pathlib
import random
import sys

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from av_matching import PairMatcher, Ray
from av_confirmation import TemporalPairGate

CASES = (
    "true_static", "true_moving",
    "false_angle", "false_skew", "false_inactive", "false_ambiguous",
    "false_variance", "false_baseline",
    "false_same_bearing_distractor", "false_reflection",
    "false_one_frame", "false_replayed_vision",
)
POSITIVE = {"true_static", "true_moving"}
HARD_NEGATIVES = {"false_same_bearing_distractor", "false_reflection"}


def direction(angle):
    return (math.cos(angle),math.sin(angle),0.0)


def trial(case, seed):
    rng = random.Random(seed)
    matcher = PairMatcher()
    gate = TemporalPairGate()
    instant = False
    corroborated = False
    start = 1_000_000_000
    for step in range(3):
        now = start + step*200_000_000
        base_angle = rng.uniform(-0.25,0.25)
        if case == "true_moving":
            base_angle += step*0.08
        a_angle = base_angle + rng.uniform(-0.008,0.008)
        v_angle = base_angle + rng.uniform(-0.008,0.008)
        v_stamp = now
        v_activity = True
        v_variance = 0.01
        v_origin = (0.05,0.0,0.0)
        if case == "false_angle":
            v_angle += 0.65
        if case == "false_skew":
            v_stamp -= 300_000_000
        if case == "false_inactive":
            v_activity = False
        if case == "false_variance":
            v_variance = 0.0
        if case == "false_baseline":
            v_origin = (0.6,0.0,0.0)
        if case == "false_one_frame" and step > 0:
            v_activity = False
        if case == "false_replayed_vision":
            v_stamp = start
        # Reflections and wrongly associated co-linear emitters look
        # geometrically indistinguishable from a genuine pair here.
        audio = Ray("source",1,now,(0.,0.,0.),direction(a_angle),0.01)
        visual = Ray("face",1,v_stamp,v_origin,direction(v_angle),
                     v_variance,v_activity)
        visuals = [visual]
        if case == "false_ambiguous":
            visuals.append(Ray("other_face",1,v_stamp,v_origin,
                               direction(v_angle+0.015),0.01,True))
        pairs = matcher.propose([audio],visuals)
        instant |= any(p.unique_match for p in pairs)
        states = gate.observe(pairs,"a"*32,"b"*32,now)
        corroborated |= any(result.confirmed for result in states.values())
    return instant,corroborated


def confusion(predictions):
    tp=sum(real and pred for real,pred in predictions)
    fp=sum(not real and pred for real,pred in predictions)
    tn=sum(not real and not pred for real,pred in predictions)
    fn=sum(real and not pred for real,pred in predictions)
    return {"tp":tp,"fp":fp,"tn":tn,"fn":fn,
            "precision":tp/(tp+fp) if tp+fp else 0.0,
            "recall":tp/(tp+fn) if tp+fn else 0.0,
            "false_positive_rate":fp/(fp+tn) if fp+tn else 0.0}


def benchmark(repetitions=20):
    if not 1 <= repetitions <= 1000:
        raise ValueError("invalid repetition count")
    baseline=[]
    temporal=[]
    by_case={}
    for i,case in enumerate(CASES):
        outputs=[]
        for n in range(repetitions):
            instantaneous,confirmed=trial(case,seed=20261107+i*10000+n)
            truth=case in POSITIVE
            baseline.append((truth,instantaneous))
            temporal.append((truth,confirmed))
            outputs.append((instantaneous,confirmed))
        by_case[case]={"truth":case in POSITIVE,
                       "instantaneous_positives":sum(x for x,_ in outputs),
                       "temporally_confirmed":sum(y for _,y in outputs)}
    return {"dataset":"procedural_v1","seed":20261107,
            "cases_per_class":repetitions,"total_trials":len(temporal),
            "instantaneous":confusion(baseline),
            "temporal":confusion(temporal),"per_case":by_case,
            "limitations":["synthetic unit rays only","no room acoustics or multipath engine",
            "no calibration uncertainty or real AV dataset",
            "two same-bearing hard negatives are intentionally not identifiable",
            "not a measure of real-world association quality"]}


if __name__ == "__main__":
    print(json.dumps(benchmark(),sort_keys=True,indent=2))
