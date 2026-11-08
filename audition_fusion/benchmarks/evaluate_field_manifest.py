#!/usr/bin/env python3
"""Score an independently labeled M4 JSON manifest; no implicit truth."""
import argparse
import hashlib
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from field_evaluation import evaluate
from acquisition_provenance import audit_acquisition_provenance
from acquisition_independence import independence_blockers, group_robustness_blockers
from simulation_provenance import verified_simulation_provenance
from recording_artifact_audit import verify_recording_artifacts


def strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError("invalid nonfinite JSON number: " + value)


def apply_diagnostic_blockers(report, reasons):
    """Keep all diagnostic verdict fields consistent after external audits."""
    safety = report["safety_diagnostic"]
    safety["blockers"] = sorted(set(safety["blockers"]) | set(reasons))
    eligible = not safety["blockers"]
    safety["diagnostic_promotion_eligible"] = eligible
    safety["no_observed_safety_regression"] = eligible
    # Never upgrade diagnostic evidence to physical or navigation admission.
    safety["physical_localization_accepted"] = False
    safety["nav2_admission"] = False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=pathlib.Path)
    parser.add_argument("--all-splits", action="store_true",
                        help="include development trials (not acceptance)")
    parser.add_argument("--require-simulation-provenance", action="store_true",
                        help="reject absent, tampered or invalid adjacent simulation sidecar")
    parser.add_argument("--acquisition-provenance", type=pathlib.Path,
                        help="strict external session/scene/subject provenance audit JSON")
    parser.add_argument("--recording-index", type=pathlib.Path,
                        help="JSON mapping every declared recording SHA to a relative artifact path")
    parser.add_argument("--recordings-root", type=pathlib.Path,
                        help="root directory of real recording files; required with --recording-index")
    parser.add_argument("--required-stratum", action="append", default=[], metavar="NAME",
                        help="require an observable held-out stratum; repeat for adverse cases")
    parser.add_argument("--require-both-truth-classes", action="store_true",
                        help="required held-out strata must have observable positive and negative truth")
    parser.add_argument("--minimum-recordings-per-truth-class", type=int, default=0,
                        metavar="N", help="minimum distinct held-out recording hashes per truth class in each required stratum")
    parser.add_argument("--minimum-acquisition-groups-per-truth-class", type=int,
                        default=0, metavar="N",
                        help="minimum independent claimed session/scene/subject IDs per truth class and required stratum")
    parser.add_argument("--maximum-false-pair-recordings-per-stratum", type=int,
                        default=None, metavar="N",
                        help="maximum distinct held-out recordings with wrong temporal-fusion pairs per required stratum")
    parser.add_argument("--maximum-false-pair-recording-fraction", type=float,
                        default=None, metavar="F",
                        help="maximum fraction of distinct observable held-out recordings with wrong pairs per required stratum")
    parser.add_argument("--maximum-false-pair-fraction-per-truth-class", type=float,
                        default=None, metavar="F",
                        help="maximum wrong-pair recording fraction separately for positive and negative held-out truth")
    parser.add_argument("--maximum-false-pair-scenes-per-stratum", type=int,
                        default=None, metavar="N",
                        help="maximum distinct claimed acquisition scenes with false temporal-fusion pairs")
    parser.add_argument("--require-held-out-recording-gain", action="store_true",
                        help="require more distinct recordings helped than harmed versus each baseline in each required stratum")
    parser.add_argument("--require-diagnostic-eligibility", action="store_true",
                        help="exit 3 if any diagnostic blocker remains (never physical admission)")
    args = parser.parse_args(argv)
    try:
        raw = args.manifest.read_bytes()
        if len(raw) > 16 * 1024 * 1024:
            raise ValueError("manifest larger than 16 MiB")
        data = json.loads(raw.decode("utf-8"),
                          object_pairs_hook=strict_object,
                          parse_constant=reject_constant)
        report = evaluate(data, held_out_only=not args.all_splits)
        if args.require_both_truth_classes and not args.required_stratum:
            raise ValueError("--require-both-truth-classes requires --required-stratum")
        if args.minimum_recordings_per_truth_class < 0 or args.minimum_recordings_per_truth_class > 100000:
            raise ValueError("invalid minimum recordings per truth class")
        if args.minimum_recordings_per_truth_class and not args.required_stratum:
            raise ValueError("--minimum-recordings-per-truth-class requires --required-stratum")
        if not 0 <= args.minimum_acquisition_groups_per_truth_class <= 100000:
            raise ValueError("invalid minimum acquisition groups per truth class")
        if args.minimum_acquisition_groups_per_truth_class and not args.required_stratum:
            raise ValueError("--minimum-acquisition-groups-per-truth-class requires --required-stratum")
        if args.maximum_false_pair_recordings_per_stratum is not None:
            if not 0 <= args.maximum_false_pair_recordings_per_stratum <= 100000:
                raise ValueError("invalid maximum false-pair recordings per stratum")
            if not args.required_stratum:
                raise ValueError("--maximum-false-pair-recordings-per-stratum requires --required-stratum")
        if args.maximum_false_pair_recording_fraction is not None:
            import math
            if not math.isfinite(args.maximum_false_pair_recording_fraction) or not (0 <= args.maximum_false_pair_recording_fraction <= 1):
                raise ValueError("invalid maximum false-pair recording fraction")
            if not args.required_stratum:
                raise ValueError("--maximum-false-pair-recording-fraction requires --required-stratum")
        if args.maximum_false_pair_fraction_per_truth_class is not None:
            import math
            if not math.isfinite(args.maximum_false_pair_fraction_per_truth_class) or not (0 <= args.maximum_false_pair_fraction_per_truth_class <= 1):
                raise ValueError("invalid maximum false-pair fraction per truth class")
            if not args.required_stratum:
                raise ValueError("--maximum-false-pair-fraction-per-truth-class requires --required-stratum")
        if args.maximum_false_pair_scenes_per_stratum is not None:
            if not 0 <= args.maximum_false_pair_scenes_per_stratum <= 100000:
                raise ValueError("invalid maximum false-pair scenes per stratum")
            if not args.required_stratum:
                raise ValueError("--maximum-false-pair-scenes-per-stratum requires --required-stratum")
        if args.require_held_out_recording_gain and not args.required_stratum:
            raise ValueError("--require-held-out-recording-gain requires --required-stratum")
        # Predeclared adverse strata must actually be represented by
        # independently observable held-out trials. A missing stratum is
        # an explicit diagnostic blocker, not an empty success.
        if args.required_stratum:
            # Coverage must always be measured on held-out independent truth,
            # even when --all-splits is used to print development diagnostics.
            # Otherwise development-only examples could satisfy a required
            # adverse condition and silently launder it into acceptance.
            held_out_coverage = {}
            for trial in data["trials"]:
                if trial["split"] != "held_out":
                    continue
                entry = held_out_coverage.setdefault(trial["stratum"], {
                    "observable_positive": 0, "observable_negative": 0,
                    "unobservable": 0})
                if not trial["truth"]["observable"]:
                    entry["unobservable"] += 1
                elif trial["truth"]["pair"] is None:
                    entry["observable_negative"] += 1
                else:
                    entry["observable_positive"] += 1
            # Trial counts alone are insufficient: many trials can come from
            # one recording and falsely inflate adverse-condition coverage.
            recording_groups = {}
            for trial in data["trials"]:
                if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                    continue
                kind = ("negative" if trial["truth"]["pair"] is None else "positive")
                recording_groups.setdefault(trial["stratum"], {
                    "positive": set(), "negative": set()})[kind].add(trial["recording_sha256"])
            missing = set()
            for name in args.required_stratum:
                if not name or name.strip() != name or len(name) > 128:
                    raise ValueError("invalid required stratum name")
                counts = held_out_coverage.get(name)
                if counts is None or not (counts["observable_positive"]
                                          + counts["observable_negative"]):
                    missing.add(name)
            report["required_strata"] = sorted(set(args.required_stratum))
            report["required_strata_held_out_coverage"] = {
                name: held_out_coverage.get(name, {
                    "observable_positive": 0, "observable_negative": 0,
                    "unobservable": 0})
                for name in report["required_strata"]}
            if args.require_both_truth_classes:
                for name, counts in report["required_strata_held_out_coverage"].items():
                    if not counts["observable_positive"] or not counts["observable_negative"]:
                        apply_diagnostic_blockers(report, {
                            "required_stratum_missing_truth_class:" + name})
            if args.minimum_recordings_per_truth_class:
                report["required_strata_recording_coverage"] = {
                    name: {kind: len(recording_groups.get(name, {}).get(kind, set()))
                           for kind in ("positive", "negative")}
                    for name in report["required_strata"]}
                for name, counts in report["required_strata_recording_coverage"].items():
                    for kind, count in counts.items():
                        if count < args.minimum_recordings_per_truth_class:
                            apply_diagnostic_blockers(report, {
                                "required_stratum_insufficient_recordings:" + name + ":" + kind})
            if args.maximum_false_pair_recordings_per_stratum is not None:
                false_recordings = {name: set() for name in report["required_strata"]}
                for trial in data["trials"]:
                    if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                        continue
                    name = trial["stratum"]
                    if name not in false_recordings:
                        continue
                    candidate = trial["predictions"]["temporal_fusion"]
                    if candidate["decision"] == "pair" and candidate["pair"] != trial["truth"]["pair"]:
                        false_recordings[name].add(trial["recording_sha256"])
                report["required_strata_false_pair_recordings"] = {
                    name: len(hashes) for name, hashes in sorted(false_recordings.items())}
                for name, count in report["required_strata_false_pair_recordings"].items():
                    if count > args.maximum_false_pair_recordings_per_stratum:
                        apply_diagnostic_blockers(report, {
                            "required_stratum_false_pair_recording_limit:" + name})
            if args.maximum_false_pair_recording_fraction is not None:
                exposure = {name: {"observable": set(), "false_pair": set()}
                            for name in report["required_strata"]}
                for trial in data["trials"]:
                    if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                        continue
                    name = trial["stratum"]
                    if name not in exposure:
                        continue
                    digest = trial["recording_sha256"]
                    exposure[name]["observable"].add(digest)
                    candidate = trial["predictions"]["temporal_fusion"]
                    if candidate["decision"] == "pair" and candidate["pair"] != trial["truth"]["pair"]:
                        exposure[name]["false_pair"].add(digest)
                report["required_strata_false_pair_recording_fraction"] = {}
                for name, groups in sorted(exposure.items()):
                    denominator = len(groups["observable"])
                    numerator = len(groups["false_pair"])
                    fraction = numerator / denominator if denominator else None
                    report["required_strata_false_pair_recording_fraction"][name] = {
                        "observable_recordings": denominator,
                        "false_pair_recordings": numerator,
                        "fraction": fraction}
                    if fraction is None or fraction > args.maximum_false_pair_recording_fraction:
                        apply_diagnostic_blockers(report, {
                            "required_stratum_false_pair_recording_fraction_limit:" + name})
            if args.maximum_false_pair_fraction_per_truth_class is not None:
                exposure_by_class = {
                    name: {kind: {"observable": set(), "false_pair": set()}
                           for kind in ("positive", "negative")}
                    for name in report["required_strata"]}
                for trial in data["trials"]:
                    if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                        continue
                    name = trial["stratum"]
                    if name not in exposure_by_class:
                        continue
                    kind = "negative" if trial["truth"]["pair"] is None else "positive"
                    groups = exposure_by_class[name][kind]
                    digest = trial["recording_sha256"]
                    groups["observable"].add(digest)
                    candidate = trial["predictions"]["temporal_fusion"]
                    if candidate["decision"] == "pair" and candidate["pair"] != trial["truth"]["pair"]:
                        groups["false_pair"].add(digest)
                report["required_strata_false_pair_fraction_by_truth_class"] = {}
                for name, classes in sorted(exposure_by_class.items()):
                    report["required_strata_false_pair_fraction_by_truth_class"][name] = {}
                    for kind, groups in classes.items():
                        denominator = len(groups["observable"])
                        numerator = len(groups["false_pair"])
                        fraction = numerator / denominator if denominator else None
                        report["required_strata_false_pair_fraction_by_truth_class"][name][kind] = {
                            "observable_recordings": denominator,
                            "false_pair_recordings": numerator, "fraction": fraction}
                        if fraction is None or fraction > args.maximum_false_pair_fraction_per_truth_class:
                            apply_diagnostic_blockers(report, {
                                "required_stratum_false_pair_truth_class_fraction_limit:" + name + ":" + kind})
            if args.require_held_out_recording_gain:
                # One recording is one unit of evidence: repeated event trials
                # cannot manufacture independent positive gain.
                from paired_comparison import BASELINES, correct
                by_recording = {name: {} for name in report["required_strata"]}
                for trial in data["trials"]:
                    if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                        continue
                    name = trial["stratum"]
                    if name not in by_recording:
                        continue
                    digest = trial["recording_sha256"]
                    record = by_recording[name].setdefault(digest, {baseline: [0, 0] for baseline in BASELINES})
                    truth = trial["truth"]["pair"]
                    true_pair = None if truth is None else (truth["audio_id"], truth["radar_id"])
                    predictions = trial["predictions"]
                    def scored(strategy):
                        item = predictions[strategy]
                        proposed = item["pair"]
                        pair = None if proposed is None else (proposed["audio_id"], proposed["radar_id"])
                        return correct(true_pair, (item["decision"], pair))
                    candidate_ok = scored("temporal_fusion")
                    for baseline in BASELINES:
                        record[baseline][0] += int(candidate_ok)
                        record[baseline][1] += int(scored(baseline))
                report["required_strata_recording_gain"] = {}
                for name, recordings in sorted(by_recording.items()):
                    report["required_strata_recording_gain"][name] = {}
                    for baseline in BASELINES:
                        helped = harmed = tied = 0
                        for scores in recordings.values():
                            candidate_count, baseline_count = scores[baseline]
                            if candidate_count > baseline_count:
                                helped += 1
                            elif candidate_count < baseline_count:
                                harmed += 1
                            else:
                                tied += 1
                        report["required_strata_recording_gain"][name][baseline] = {
                            "helped": helped, "harmed": harmed, "tied": tied}
                        if helped <= harmed:
                            apply_diagnostic_blockers(report, {
                                "required_stratum_no_recording_gain:" + name + ":" + baseline})
            if missing:
                apply_diagnostic_blockers(report, {
                    "required_stratum_missing_or_unobservable:" + name
                    for name in missing})
        if args.require_simulation_provenance:
            report.update(verified_simulation_provenance(args.manifest, raw, data))
        else:
            report["data_domain"] = "unverified"
            report["provenance_verified"] = False
        # A valid simulated lineage is still simulated. A supplied acquisition
        # manifest must not launder simulated observations into field evidence.
        # Conversely an unverified domain cannot be promoted as field evidence.
        if report["data_domain"] == "simulation":
            apply_diagnostic_blockers(report, {"simulation_only_evidence"})
        else:
            apply_diagnostic_blockers(report, {"data_domain_unverified"})
        if (args.recording_index is None) != (args.recordings_root is None):
            raise ValueError("--recording-index and --recordings-root must both be supplied")
        if args.recording_index is not None:
            report["recording_artifact_audit"] = verify_recording_artifacts(
                args.recording_index, args.recordings_root, data)
        else:
            report["recording_artifact_audit"] = {
                "status": "not_provided",
                "recording_bytes_verified": False,
                "origin_authenticated": False,
                "physical_validation_claim": False,
            }
            apply_diagnostic_blockers(report, {"recording_bytes_not_verified"})
        if args.acquisition_provenance is not None:
            audit = audit_acquisition_provenance(args.acquisition_provenance, data)
            report["acquisition_provenance_audit"] = audit
            new_blockers = independence_blockers(audit["independence_summary"])
            new_blockers += group_robustness_blockers(
                audit["paired_group_sensitivity"])
            if args.minimum_acquisition_groups_per_truth_class:
                coverage = audit["held_out_truth_class_groups"]
                report["required_strata_acquisition_group_coverage"] = {
                    name: {kind: {field: coverage.get(name, {}).get(kind, {}).get(field, 0)
                                  for field in ("session_id", "scene_id", "subject_id")}
                           for kind in ("positive", "negative")}
                    for name in sorted(set(args.required_stratum))}
                for name, classes in report["required_strata_acquisition_group_coverage"].items():
                    for kind, fields in classes.items():
                        for field, count in fields.items():
                            if count < args.minimum_acquisition_groups_per_truth_class:
                                new_blockers.append(
                                    "required_stratum_insufficient_acquisition_groups:"
                                    + name + ":" + kind + ":" + field)
            if args.maximum_false_pair_scenes_per_stratum is not None:
                # Audit has already validated trial identity, recording digest,
                # split isolation and stable scene labels for each recording.
                scene_by_trial = {row["trial_id"]: row["scene_id"]
                                  for row in audit["trials"]} if "trials" in audit else None
                # The audit result is a summary, not the original provenance
                # records. Re-read only after strict audit validation succeeds.
                if scene_by_trial is None:
                    provenance = json.loads(args.acquisition_provenance.read_text(encoding="utf-8"),
                                            object_pairs_hook=strict_object,
                                            parse_constant=reject_constant)
                    scene_by_trial = {row["trial_id"]: row["scene_id"]
                                      for row in provenance["trials"]}
                affected = {name: set() for name in sorted(set(args.required_stratum))}
                for trial in data["trials"]:
                    if trial["split"] != "held_out" or not trial["truth"]["observable"]:
                        continue
                    name = trial["stratum"]
                    if name not in affected:
                        continue
                    prediction = trial["predictions"]["temporal_fusion"]
                    if prediction["decision"] == "pair" and prediction["pair"] != trial["truth"]["pair"]:
                        affected[name].add(scene_by_trial[trial["trial_id"]])
                report["required_strata_false_pair_scene_coverage"] = {
                    name: len(scenes) for name, scenes in sorted(affected.items())}
                for name, count in report["required_strata_false_pair_scene_coverage"].items():
                    if count > args.maximum_false_pair_scenes_per_stratum:
                        new_blockers.append("required_stratum_false_pair_scene_limit:" + name)
            if new_blockers:
                apply_diagnostic_blockers(report, new_blockers)
        else:
            report["acquisition_provenance_audit"] = {
                "status": "not_provided",
                "crosscheck_passed": False,
                "independently_authenticated": False,
                "physical_validation_claim": False,
            }
            # The standalone evaluator can still diagnose synthetic fixtures,
            # but a CLI score without external collection metadata is never
            # eligible even for diagnostic promotion.
            apply_diagnostic_blockers(report, {"acquisition_provenance_not_supplied"})
            if args.minimum_acquisition_groups_per_truth_class:
                apply_diagnostic_blockers(report, {"required_acquisition_groups_unverifiable"})
            if args.maximum_false_pair_scenes_per_stratum is not None:
                apply_diagnostic_blockers(report, {"required_false_pair_scenes_unverifiable"})
        report["manifest_sha256"] = hashlib.sha256(raw).hexdigest()
        report["manifest_name"] = args.manifest.name
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        if args.require_diagnostic_eligibility and not report["safety_diagnostic"]["diagnostic_promotion_eligible"]:
            print("M4 diagnostic eligibility blocked: " + ", ".join(
                report["safety_diagnostic"]["blockers"]), file=sys.stderr)
            return 3
        return 0
    except (ValueError, OSError, UnicodeError) as error:
        print("Invalid M4 evaluation input:", error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
