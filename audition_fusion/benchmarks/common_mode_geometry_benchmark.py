#!/usr/bin/env python3
"""Emit the M4 correlated-error negative control as machine-readable JSON."""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common_mode_audit import report, assert_expected


def main():
    result = report()
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    try:
        assert_expected(result)
    except AssertionError:
        print("FAIL: correlated-error geometry contract changed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
