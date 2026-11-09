#!/usr/bin/env python3
"""Import independently labeled simulation export v1 into M4 manifest JSON."""
import argparse
import hashlib
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from simulation_import import convert
from evaluate_field_manifest import strict_object, reject_constant


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export", type=pathlib.Path)
    parser.add_argument("--artifacts-root", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args(argv)
    try:
        raw = args.export.read_bytes()
        if len(raw) > 16 * 1024 * 1024:
            raise ValueError("export exceeds 16 MiB")
        data = json.loads(raw.decode("utf-8"), object_pairs_hook=strict_object,
                          parse_constant=reject_constant)
        manifest = convert(data, artifacts_root=args.artifacts_root)
        # The manifest is intentionally the existing v1 scorer schema.
        # The output filename is explicitly marked 'simulation' by caller.
        output = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        provenance = {
            "domain": "simulation", "physical_validation_claim": False,
            "simulation_export_sha256": hashlib.sha256(raw).hexdigest(),
            "manifest_sha256": hashlib.sha256(output.encode()).hexdigest(),
            "trials": len(manifest["trials"]),
        }
        # The v1 scorer deliberately has a fixed schema. Keep origin and
        # hashes in a required adjacent machine-readable sidecar instead of
        # quietly stripping the simulation provenance during conversion.
        sidecar = args.out.with_name(args.out.name + ".provenance.json")
        args.out.write_text(output, encoding="utf-8")
        sidecar.write_text(json.dumps(provenance, sort_keys=True, indent=2)
                           + "\n", encoding="utf-8")
        print(json.dumps(provenance, sort_keys=True), file=sys.stderr)
        return 0
    except (ValueError, OSError, UnicodeError) as error:
        print("Invalid simulator export:", error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
