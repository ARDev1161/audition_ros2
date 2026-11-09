"""Strict simulator-export v1 to M4 independent trial-manifest conversion.

The simulator is an *external producer*; this consumes explicit authoritative
labels and frozen strategy results. It must never infer true physical identity
from acoustic/radar geometry or fill missing predictions with success.
"""
import hashlib
import pathlib

from field_evaluation import STRATEGIES, identity, require, validate


def convert(export, *, artifacts_root):
    require(isinstance(export, dict) and set(export) == {
        "format", "dataset_id", "domain", "scenarios"
    }, "invalid simulator export root")
    require(export["format"] == "audition.simulation-export.v1",
            "unsupported simulator export format")
    require(export["domain"] == "simulation", "only simulated recordings accepted")
    dataset_id = identity(export["dataset_id"], "dataset_id")
    scenarios = export["scenarios"]
    require(isinstance(scenarios, list) and 1 <= len(scenarios) <= 10000,
            "invalid scenario list")
    root = pathlib.Path(artifacts_root).resolve(strict=True)
    require(root.is_dir(), "artifacts_root is not a directory")
    used_ids = set()
    trial_ids = set()
    trials = []
    for scenario in scenarios:
        require(isinstance(scenario, dict) and set(scenario) == {
            "scenario_id", "split", "recording", "recording_sha256", "trials"
        }, "invalid scenario fields")
        sid = identity(scenario["scenario_id"], "scenario_id")
        require(sid not in used_ids, "duplicate scenario_id")
        used_ids.add(sid)
        require(scenario["split"] in ("development", "held_out"),
                "invalid scenario split")
        filename = scenario["recording"]
        require(isinstance(filename, str) and filename
                and pathlib.PurePosixPath(filename).as_posix() == filename
                and not pathlib.PurePosixPath(filename).is_absolute()
                and ".." not in pathlib.PurePosixPath(filename).parts
                and not filename.startswith("./")
                and "\\" not in filename,
                "invalid relative recording path")
        file_path = (root / filename).resolve(strict=True)
        require(file_path.is_relative_to(root) and file_path.is_file(),
                "recording outside artifacts root or not a file")
        # A symlink in the export is deliberately forbidden to ensure the
        # frozen export cannot silently switch its underlying artifact.
        require(not any(p.is_symlink() for p in (
            root / filename, *(root / filename).parents
        ) if p != root), "symlink recording path not allowed")
        expected_sha = scenario["recording_sha256"]
        require(isinstance(expected_sha, str) and len(expected_sha) == 64
                and all(c in "0123456789abcdef" for c in expected_sha),
                "invalid recording digest")
        digest = hashlib.sha256()
        with file_path.open("rb") as artifact:
            while True:
                block = artifact.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
        require(digest.hexdigest() == expected_sha,
                "recording SHA256 mismatch: " + sid)
        items = scenario["trials"]
        require(isinstance(items, list) and items,
                "scenario without trials")
        for item in items:
            require(isinstance(item, dict) and set(item) == {
                "trial_id", "stratum", "truth_source", "truth", "predictions"
            }, "invalid simulated trial fields")
            tid = identity(item["trial_id"], "trial_id")
            require(tid not in trial_ids, "duplicate trial_id")
            trial_ids.add(tid)
            require(item["truth_source"] == "simulator_authoritative",
                    "truth must be independent simulator authority")
            trials.append({
                "trial_id": tid,
                "stratum": item["stratum"],
                "split": scenario["split"],
                "recording_sha256": expected_sha,
                "truth": item["truth"],
                "predictions": item["predictions"],
            })
    manifest = {"schema_version": 1, "dataset_id": dataset_id,
                "trials": trials}
    validate(manifest)  # Full v1 contract plus cross-split hash fencing.
    return manifest
