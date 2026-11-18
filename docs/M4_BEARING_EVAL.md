# M4 planar source-location and range evaluation (external truth required)

`audition_fusion/benchmarks/score_bearing_groundtruth.py` can score **real,
independently surveyed** XY source positions against diagnostic intersections
from `fusion/bearing_candidates`. It does not triangulate, infer labels or
invent source covariance. Run:

```bash
python3 audition_fusion/benchmarks/score_bearing_groundtruth.py surveyed.jsonl \
  --maximum-position-error-m 1.0
```

JSONL example, one fully annotated observation snapshot per line:

```json
{"schema":"bearing_eval_v1","stamp_ns":1000000000,"fully_annotated":true,"scene_session_id":"11111111111111111111111111111111","frame_id":"odom","robot_xy_m":[0,0],"sources":[{"source_id":"speech","source_generation":1,"truth_xy_m":[3,4]},{"source_id":"reflection","source_generation":1,"truth_xy_m":null}],"candidates":[{"scene_session_id":"11111111111111111111111111111111","frame_id":"odom","source_id":"speech","source_generation":1,"xy_m":[3.1,4.1],"supporting_observations":3,"baseline_m":0.9}]}
```

`truth_xy_m=null` is a **fully annotated negative** (e.g. reflection, false
source or source not physically linked to this track), not missing data.
When the annotator cannot independently establish a complete source/truth
map, mark the entire snapshot `fully_annotated=false`; it is skipped,
never treated as a true negative. All active track identities must be
enumerated in `sources`, including unlocalized/reflection hypotheses.

TP/FP/TN/FN describe **whether a candidate was emitted for the correct
annotated track**, not whether the metric coordinate is accurate. For TPs,
the evaluator separately reports nearest-rank median and p95 Euclidean
position error and **absolute range error relative to the surveyed robot
origin**. Also reports how many candidate positions meet the configured
error threshold. Crucially, missing true-source candidates count as FN;
position/range percentiles among emitted candidates must always be read
alongside recall. Rejects cross-session/cross-frame predictions, bad geometry
provenance, duplicates, NaN, invalid coordinates or reversed frame times.

The annotator, not the algorithm, supplies surveyed ground truth, clock
alignment and track correspondence. No dataset ships with this tool. It
cannot validate 3D Z, covariance calibration, scene motion, multipath
mitigation or joint localization/costmap safety. Do not claim positive
fusion gain until results on the **same independent labeled scenes** beat
the separate sensing baseline and include calibration/robot-pose error.
