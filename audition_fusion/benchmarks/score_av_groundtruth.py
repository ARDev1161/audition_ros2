#!/usr/bin/env python3
"""Strict frame-level scoring of fully annotated AV hypothesis snapshots.

Ground truth is supplied externally. This tool does not synthesize labels,
interpret sound, or turn diagnostic pair hypotheses into physical identities.
"""
import argparse
import json
import math
import re
import sys


def identity(value):
    if (not isinstance(value, list) or len(value) != 2
            or not isinstance(value[0], str) or not value[0]
            or len(value[0].encode("utf-8")) > 256
            or type(value[1]) is not int or value[1] < 1):
        raise ValueError("invalid [track_id, generation]")
    return (value[0], value[1])


def pair_key(value):
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("invalid AV pair")
    audio = identity(value[:2])
    visual = identity(value[2:])
    return audio + visual


def keys(values, converter, limit, label):
    if not isinstance(values, list) or len(values) > limit:
        raise ValueError("invalid " + label + " size")
    items = [converter(x) for x in values]
    if len(items) != len(set(items)):
        raise ValueError("duplicate " + label)
    return set(items)


def matrix(tp=0, fp=0, tn=0, fn=0):
    return dict(tp=tp, fp=fp, tn=tn, fn=fn)


def add(result, truth, selected, universe_size):
    result["tp"] += len(truth & selected)
    result["fp"] += len(selected - truth)
    result["fn"] += len(truth - selected)
    result["tn"] += universe_size - len(truth | selected)


def metrics(result):
    tp, fp, tn, fn = (result[x] for x in ("tp", "fp", "tn", "fn"))
    return dict(result,
        precision=tp/(tp+fp) if tp+fp else 0.,
        recall=tp/(tp+fn) if tp+fn else 0.,
        false_positive_rate=fp/(fp+tn) if fp+tn else 0.)


def score_frames(frames):
    instant, confirmed = matrix(), matrix()
    frame_count, skipped_count, last_stamp = 0, 0, 0
    for index, row in enumerate(frames, start=1):
        if not isinstance(row, dict) or row.get("schema") != "av_eval_v1":
            raise ValueError("frame %d: invalid schema" % index)
        stamp = row.get("stamp_ns")
        if type(stamp) is not int or stamp <= last_stamp:
            raise ValueError("frame %d: non-monotonic observation time" % index)
        last_stamp = stamp
        for name in ("audio_scene_session_id", "vision_session_id"):
            value = row.get(name)
            if not isinstance(value, str) or not re.fullmatch("[a-f0-9]{32}", value):
                raise ValueError("frame %d: invalid %s" % (index, name))
        annotated = row.get("fully_annotated")
        if type(annotated) is not bool:
            raise ValueError("frame %d: annotation completeness required" % index)
        if not annotated:
            skipped_count += 1
            continue
        audios = keys(row.get("audio_tracks"), identity, 16, "audio tracks")
        visuals = keys(row.get("visual_tracks"), identity, 16, "visual tracks")
        truth = keys(row.get("truth_pairs"), pair_key, 256, "ground truth")
        universe = {a + v for a in audios for v in visuals}
        if not truth <= universe:
            raise ValueError("frame %d: truth refers to absent tracks" % index)
        predicted = row.get("hypotheses")
        if not isinstance(predicted, list) or len(predicted) > 256:
            raise ValueError("frame %d: invalid hypotheses" % index)
        all_pairs, unique, temporal = set(), set(), set()
        for item in predicted:
            if not isinstance(item, dict):
                raise ValueError("frame %d: malformed hypothesis" % index)
            key = pair_key(item.get("pair"))
            if key not in universe or key in all_pairs:
                raise ValueError("frame %d: invalid/duplicate hypothesis" % index)
            all_pairs.add(key)
            unq, conf = item.get("unique_match"), item.get("temporally_confirmed")
            if type(unq) is not bool or type(conf) is not bool:
                raise ValueError("frame %d: missing boolean pair gates" % index)
            if conf and not unq:
                raise ValueError("frame %d: confirmed ambiguous pair" % index)
            if unq:
                unique.add(key)
            if conf:
                temporal.add(key)
        add(instant, truth, unique, len(universe))
        add(confirmed, truth, temporal, len(universe))
        frame_count += 1
    if not frame_count:
        raise ValueError("no completely annotated evaluation frames")
    return {"schema":"av_eval_results_v1", "scored_frames":frame_count,
            "skipped_incomplete_frames":skipped_count,
            "instantaneous":metrics(instant), "temporal":metrics(confirmed),
            "note":"Frame-level annotated associations; not speaker identity or localization accuracy"}


def parse_jsonl(lines):
    for line_number, line in enumerate(lines, start=1):
        if len(line) > 1_000_000:
            raise ValueError("line %d exceeds size limit" % line_number)
        if line.strip():
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError("line %d: invalid JSON" % line_number) from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("annotations_jsonl", help="Fully annotated AV snapshots in av_eval_v1 schema")
    args = parser.parse_args()
    try:
        with open(args.annotations_jsonl, encoding="utf-8") as stream:
            report=score_frames(parse_jsonl(stream))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(report, sort_keys=True, indent=2))


if __name__ == "__main__":
    main()
