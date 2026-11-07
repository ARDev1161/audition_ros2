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
