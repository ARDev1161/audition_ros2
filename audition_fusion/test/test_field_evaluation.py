"""Field metrics are trial-scoped, with explicit unobservable abstention."""
import copy
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from field_evaluation import evaluate, validate


def pair(a, r):
    return {"audio_id": a, "radar_id": r}


def prediction(decision, actual=None):
    return {"decision": decision, "pair": actual}


def trial(id_, truth, strategies, *, observable=True, split="held_out",
          stratum="direct"):
    return {
        "trial_id": id_, "stratum": stratum, "split": split,
        "recording_sha256": "a" * 64,
        "truth": {"observable": observable, "pair": truth},
        "predictions": strategies,
    }


def example():
    pos = pair("source-1", "radar-1")
    impostor = pair("source-1", "radar-2")
    return {
        "schema_version": 1, "dataset_id": "synthetic-contract-example",
        "trials": [
            trial("direct", pos, {
                "audio_only": prediction("pair", pos),
                "radar_only": prediction("abstain"),
                "instant_fusion": prediction("pair", pos),
                "temporal_fusion": prediction("pair", pos),
            }),
            trial("wrong-pair", pos, {
                "audio_only": prediction("none"),
                "radar_only": prediction("pair", impostor),
                "instant_fusion": prediction("pair", impostor),
                "temporal_fusion": prediction("none"),
            }, stratum="impostor"),
            trial("real-negative", None, {
                "audio_only": prediction("none"),
                "radar_only": prediction("pair", impostor),
                "instant_fusion": prediction("pair", impostor),
                "temporal_fusion": prediction("none"),
            }, stratum="impostor"),
            trial("unobservable-identity", None, {
                "audio_only": prediction("pair", pos),
                "radar_only": prediction("abstain"),
                "instant_fusion": prediction("pair", pos),
                "temporal_fusion": prediction("none"),
            }, observable=False, stratum="occluded"),
            trial("development-excluded", pos, {
                name: prediction("none") for name in (
                    "audio_only", "radar_only", "instant_fusion",
                    "temporal_fusion")
            }, split="development"),
        ],
    }


class FieldEvaluationTest(unittest.TestCase):
    def test_metrics_preserve_wrong_positive_and_unknown_truth(self):
        report = evaluate(example())
        self.assertEqual(report["trial_count"], 4)
        instant = report["strategies"]["instant_fusion"]
        self.assertEqual(
            {k: instant[k] for k in ("tp","fp","tn","fn","unobservable")},
            {"tp": 1, "fp": 2, "tn": 0, "fn": 1, "unobservable": 1},
        )
        temporal = report["strategies"]["temporal_fusion"]
        self.assertEqual(
            {k: temporal[k] for k in ("tp","fp","tn","fn")},
            {"tp":1,"fp":0,"tn":1,"fn":1},
        )
        self.assertEqual(temporal["precision"], 1.0)
        self.assertEqual(temporal["recall"], .5)
        self.assertEqual(
            report["strategies"]["radar_only"]["abstained"], 1)
        self.assertEqual(report["per_stratum"]["occluded"]
                         ["instant_fusion"]["unobservable"], 1)
        self.assertFalse(report["physical_validation_claim"])

    def test_split_separation(self):
        self.assertEqual(evaluate(example(),held_out_only=False)["trial_count"],5)
        item=example()
        for t in item["trials"]:
            t["split"]="development"
        with self.assertRaisesRegex(ValueError,"no eligible trials"):
            evaluate(item)

    def test_bad_labels_duplicates_missing_methods_and_unknown_fields(self):
        baseline=example()
        variants=[]
        v=copy.deepcopy(baseline)
        v["trials"][1]["trial_id"]="direct"
        variants.append(v)
        v=copy.deepcopy(baseline)
        del v["trials"][0]["predictions"]["radar_only"]
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["trials"][0]["predictions"]["temporal_fusion"]["pair"]=None
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["trials"][3]["truth"]["pair"]=pair("source-1","radar-1")
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["trials"][0]["recording_sha256"]="unknown"
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["trials"][0]["predictions"]["instant_fusion"]["confidence"]=.99
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["schema_version"]=2
        variants.append(v)
        v=copy.deepcopy(baseline)
        v["trials"][0]["truth"]["observable"]=1
        variants.append(v)
        for case in variants:
            with self.subTest(case=variants.index(case)), self.assertRaises(ValueError):
                validate(case)

    def test_empty_and_unobservable_cannot_be_claimed_accurate(self):
        item=example()
        item["trials"]=[item["trials"][3]]
        report=evaluate(item)
        for metric in report["strategies"].values():
            self.assertIsNone(metric["precision"])
            self.assertIsNone(metric["recall"])
            self.assertIsNone(metric["false_positive_rate"])
            self.assertEqual(metric["unobservable"],1)
        item["trials"]=[]
        with self.assertRaises(ValueError):
            evaluate(item)


if __name__=="__main__":
    unittest.main()
