# M4 independent ground-truth AV scoring

`score_av_groundtruth.py` scores fully annotated, externally provided
**frame-level** AV association labels. It does not inspect raw audio/video,
infer speaker identity, estimate range or authorize robot motion.

```bash
python3 audition_fusion/benchmarks/score_av_groundtruth.py labeled_frames.jsonl
```

One JSON object per line, sorted by strictly increasing `stamp_ns`:

```json
{"schema":"av_eval_v1","stamp_ns":1000000000,"fully_annotated":true,"audio_scene_session_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","vision_session_id":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","audio_tracks":[["s1",1]],"visual_tracks":[["face1",1]],"truth_pairs":[["s1",1,"face1",1]],"hypotheses":[{"pair":["s1",1,"face1",1],"unique_match":true,"temporally_confirmed":false}]}
```

Generation is part of both track identities. `truth_pairs` is the externally
annotated common physical source pairing for that frame, not merely similar
bearings; annotators must label it independently of the algorithm outputs.
Each frame lists **all** independently observable audio and visual tracks,
including unrelated noise and silent faces. `fully_annotated=false` excludes
a frame from evaluation, rather than converting unknown associations into
false negatives or true negatives. At least one complete frame is required.

`hypotheses` holds the exact diagnostic pairing output at that frame.
Scoring uses the same ground truth and eligible audio × visual universe for
two policies: instantaneous `unique_match`, and the stricter
`temporally_confirmed` pair (which must also be unique). Results contain
TP, FP, TN, FN, micro precision/recall and false-positive rate. This is a
**frame-level**, not independent-event-level metric. Dense cross-products
can make TN and FPR look artificially favorable; report raw counts and
precision alongside rates. Track switches, class imbalance and long clips
require further dataset-specific reporting.

Input validation rejects duplicated identities, invalid session IDs,
missing truth tracks, out-of-order frames, confirmed-but-ambiguous outputs,
and unknown boolean gates. Maximum 16 audio and 16 visual tracks per frame;
larger scenes require a new explicitly tested contract, not silent truncation.
The input schema can be populated from timestamp-aligned rosbag exports and
human-authored source labels; this repository does not yet ship a rosbag
converter or an annotated real-world recording.

Do not upload or publish personally identifying recordings without explicit
consent and a retention policy. Prefer anonymized track IDs and timestamps.
Passing this deterministic evaluator does not establish M4's real-device
accuracy or positive fusion gain.
