"""Verify recording file bytes referenced by an independently annotated M4 trial manifest.

This is an artifact-integrity check, not authentication of physical origin,
sensor calibration, labels, or evidence that a sound source was localized.
"""
import hashlib
import json
import os
import pathlib
import re
import stat

MAX_INDEX_BYTES = 16 * 1024 * 1024
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
CHUNK = 1024 * 1024


def strict_object(pairs):
    result = {}
    for name, value in pairs:
        if name in result:
            raise ValueError("duplicate recording index JSON key: " + name)
        result[name] = value
    return result


def reject_constant(value):
    raise ValueError("nonfinite number in recording index: " + value)


def relative_parts(value):
    if (not isinstance(value, str) or not value or len(value) > 2048
            or "\\" in value or "\x00" in value):
        raise ValueError("invalid relative recording path")
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise ValueError("recording path must remain beneath the root")
    return parts


def read_digest(root_fd, relative_path):
    """Use no-follow directory descriptors: neither parents nor files may be links."""
    parts = relative_parts(relative_path)
    directory_fd = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            next_fd = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                              dir_fd=directory_fd)
            os.close(directory_fd)
            directory_fd = next_fd
        file_fd = os.open(parts[-1],
                          os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK,
                          dir_fd=directory_fd)
        try:
            before = os.fstat(file_fd)
            if not stat.S_ISREG(before.st_mode):
                raise ValueError("recording artifact is not a regular file")
            digest = hashlib.sha256()
            total = 0
            while True:
                block = os.read(file_fd, CHUNK)
                if not block:
                    break
                digest.update(block)
                total += len(block)
            after = os.fstat(file_fd)
            if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
                    after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns
                    ) or total != before.st_size:
                raise ValueError("recording changed while verifying bytes")
            return digest.hexdigest(), total
        finally:
            os.close(file_fd)
    finally:
        os.close(directory_fd)


def verify_recording_artifacts(index_path, recordings_root, dataset):
    """Require exactly one matching, regular file for every declared recording SHA."""
    if not all(hasattr(os, name) for name in ("O_DIRECTORY", "O_NOFOLLOW",
                                              "O_NONBLOCK")):
        raise ValueError("recording verification requires no-follow POSIX file access")
    raw = pathlib.Path(index_path).read_bytes()
    if len(raw) > MAX_INDEX_BYTES:
        raise ValueError("recording index larger than 16 MiB")
    data = json.loads(raw.decode("utf-8"), object_pairs_hook=strict_object,
                      parse_constant=reject_constant)
    if (not isinstance(data, dict)
            or set(data) != {"schema_version", "recordings"}
            or type(data["schema_version"]) is not int
            or data["schema_version"] != 1
            or not isinstance(data["recordings"], list)):
        raise ValueError("invalid recording index schema")
    expected = {trial["recording_sha256"] for trial in dataset["trials"]}
    found = {}
    paths = set()
    for item in data["recordings"]:
        if not isinstance(item, dict) or set(item) != {"recording_sha256", "path"}:
            raise ValueError("invalid recording entry")
        digest, path = item["recording_sha256"], item["path"]
        if not isinstance(digest, str) or SHA256.fullmatch(digest) is None:
            raise ValueError("invalid recording index sha256")
        relative_parts(path)
        if digest in found:
            raise ValueError("duplicate recording SHA-256 in index")
        if path in paths:
            raise ValueError("recording artifact path reused for two digests")
        found[digest] = path
        paths.add(path)
    if found.keys() != expected:
        raise ValueError("recording index missing or extra declared SHA-256")
    root_fd = os.open(recordings_root,
                      os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        total_bytes = 0
        for declared, path in sorted(found.items()):
            calculated, count = read_digest(root_fd, path)
            if calculated != declared:
                raise ValueError("recording bytes SHA-256 mismatch: " + path)
            total_bytes += count
    finally:
        os.close(root_fd)
    return {
        "status": "all_recording_bytes_match",
        "recording_bytes_verified": True,
        "verified_unique_recordings": len(found),
        "verified_total_bytes": total_bytes,
        "index_sha256": hashlib.sha256(raw).hexdigest(),
        "index_name": pathlib.Path(index_path).name,
        "origin_authenticated": False,
        "physical_validation_claim": False,
        "note": ("File bytes agree with their SHA-256 in the trial manifest. "
                 "This does not authenticate acquisition, labels, or physical truth."),
    }
