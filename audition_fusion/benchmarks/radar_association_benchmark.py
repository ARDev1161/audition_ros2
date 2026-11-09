#!/usr/bin/env python3
"""Deterministic M4-C radar/audio *candidate* benchmark; no sensor truth claim.

Every fixture carries an explicit truth label independent of the matcher.
Same-bearing impostors and reflections are deliberately indistinguishable
from true emitters under this input model, and must count as false positives.
"""
import json
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from av_matching import Ray
from radar_bearing_matching import RadarPoint, RadarBearingMatcher

COV=(.04,0.,0.,0.,.04,0.,0.,0.,.04)
T0=1_000_000_000
CLASSES=(
    'true_direct', 'true_small_offset', 'negative_off_axis',
    'negative_behind', 'negative_skew', 'negative_ambiguous',
    'negative_aligned_impostor', 'negative_aligned_reflection',
)


def fixture(kind, seed):
    t=T0+seed*1_000_000_000
    a=Ray('audio',1,t,(0.,0.,0.),(1.,0.,0.),.01)
    r=RadarPoint('radar',1,t,(3.,0.,0.),COV)
    radars=[r]
    truth=kind.startswith('true_')
    if kind=='true_small_offset':
        radars=[RadarPoint('radar',1,t,(3.,.1,0.),COV)]
    elif kind=='negative_off_axis':
        radars=[RadarPoint('radar',1,t,(0.,3.,0.),COV)]
    elif kind=='negative_behind':
        radars=[RadarPoint('radar',1,t,(-3.,0.,0.),COV)]
    elif kind=='negative_skew':
        radars=[RadarPoint('radar',1,t+200_000_000,(3.,0.,0.),COV)]
    elif kind=='negative_ambiguous':
        radars=[r,RadarPoint('impostor',1,t,(3.,0.,0.),COV)]
    # The last two classes *share the exact same measurements* as
    # 'true_direct' but are labelled false in ground truth by construction.
    return [a],radars,truth


def report():
    matcher=RadarBearingMatcher()
    counts={'tp':0,'fp':0,'tn':0,'fn':0}
    per_class={}
    for kind in CLASSES:
        current={'tp':0,'fp':0,'tn':0,'fn':0}
        for seed in range(20):
            audio,radar,truth=fixture(kind,seed)
            proposals=matcher.propose(audio,radar)
            predicted=any(x.unique_match for x in proposals)
            label=('t' if truth else 'f')+('p' if predicted else 'n')
            # The correct confusion-matrix class is conditioned on truth:
            # true+prediction -> TP, true+abstain -> FN, false+prediction -> FP.
            key=('tp' if truth and predicted else
                 'fn' if truth else
                 'fp' if predicted else 'tn')
            current[key]+=1
            counts[key]+=1
        per_class[kind]=current
    precision=counts['tp']/(counts['tp']+counts['fp'])
    recall=counts['tp']/(counts['tp']+counts['fn'])
    fpr=counts['fp']/(counts['fp']+counts['tn'])
    result={'fixtures':len(CLASSES)*20,'confusion':counts,
            'precision':precision,'recall':recall,'false_positive_rate':fpr,
            'per_class':per_class,
            'status':'synthetic_only_not_physical_validation'}
    assert counts=={'tp':40,'fp':40,'tn':80,'fn':0}, result
    assert per_class['negative_aligned_impostor']['fp']==20
    assert per_class['negative_aligned_reflection']['fp']==20
    return result


if __name__=='__main__':
    print(json.dumps(report(),indent=2,sort_keys=True))
