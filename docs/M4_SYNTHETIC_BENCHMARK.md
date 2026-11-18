# M4 synthetic AV pairing benchmark (no real-world accuracy claim)

Run:

```bash
python3 audition_fusion/benchmarks/av_association_benchmark.py
python3 -m unittest discover -s audition_fusion/test -p 'test_*.py' -v
```

The benchmark generates **240 deterministic, ground-truth-labeled synthetic
three-frame scenarios** (12 classes × 20 seeds) and prints JSON containing
instantaneous mutual-unique pairing and three-frame temporal corroboration
confusion matrices plus per-class results.

It explicitly covers correct stationary and changing bearings; bad angles,
clock skew, inactive visual evidence, two equally plausible faces, invalid
variance, excessive microphone–camera separation, single-frame coincidences
and replayed visual timestamps. Crucially it also includes a visual
**impostor on the same bearing** and **a reflected source** with plausible
bearings. Their synthetic observations are indistinguishable from genuine
association using only these inputs, so the gate deliberately **fails**
these cases; this is reported as false positives, not hidden.

The deterministic expected regression outcome is:

| Method | TP | FP | TN | FN | Precision | Recall | FPR |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Best single-frame pair ever | 40 | 80 | 120 | 0 | 33.3% | 100% | 40.0% |
| Three independent refreshed pairs | 40 | 40 | 160 | 0 | 50.0% | 100% | 20.0% |

This quantifies the benefit of a temporal gate **on these specifically
constructed examples only**. It does *not* validate calibration, moving
robot/source synchronization, voice identity, reflection robustness,
uncertainty estimates, or accuracy on real-world people. In particular,
the remaining 40 adversarial false associations show why these are only
diagnostic suggestions. No `LocalizedSource` or Nav2 cost is emitted.
Ground-truth AV recordings and a measured independent-sensing baseline
remain M4 release gates.
