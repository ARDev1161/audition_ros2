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
from acquisition_independence import independence_blockers
from simulation_provenance import verified_simulation_provenance


def strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError("invalid nonfinite JSON number: " + value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=pathlib.Path)
    parser.add_argument("--all-splits", action="store_true",
                        help="include development trials (not acceptance)")
    parser.add_argument("--require-simulation-provenance", action="store_true",
                        help="reject absent, tampered or invalid adjacent simulation sidecar")
    parser.add_argument("--acquisition-provenance", type=pathlib.Path,
                        help="strict external session/scene/subject provenance audit JSON")
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
        if args.acquisition_provenance is not None:
            audit = audit_acquisition_provenance(args.acquisition_provenance, data)
            report["acquisition_provenance_audit"] = audit
            new_blockers = independence_blockers(audit["independence_summary"])
            if new_blockers:
                safety = report["safety_diagnostic"]
                safety["blockers"] = sorted(set(safety["blockers"]) | set(new_blockers))
                safety["diagnostic_promotion_eligible"] = False
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
            safety = report["safety_diagnostic"]
            safety["blockers"] = sorted(set(safety["blockers"]) |
                                        {"acquisition_provenance_not_supplied"})
            safety["diagnostic_promotion_eligible"] = False
        report["manifest_sha256"] = hashlib.sha256(raw).hexdigest()
        report["manifest_name"] = args.manifest.name
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        return 0
    except (ValueError, OSError, UnicodeError) as error:
        print("Invalid M4 evaluation input:", error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
