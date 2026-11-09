"""Ground-truth evaluation must not invent negatives from partial labels."""
import copy
import io
import json
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/"benchmarks"))
from score_av_groundtruth import score_frames, parse_jsonl


def frame(t=1000000000):
    return {
        "schema":"av_eval_v1",
        "stamp_ns":t,
        "fully_annotated":True,
        "audio_scene_session_id":"a"*32,
        "vision_session_id":"b"*32,
        "audio_tracks":[["speech",1],["fan",1]],
        "visual_tracks":[["person",1],["screen",1]],
        "truth_pairs":[["speech",1,"person",1]],
        "hypotheses":[
            {"pair":["speech",1,"person",1],"audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,"unique_match":True,"temporally_confirmed":False},
            {"pair":["fan",1,"screen",1],"audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,"unique_match":False,"temporally_confirmed":False},
        ],
    }


class ScoringTests(unittest.TestCase):
    def test_frame_level_confusion_and_partial_skip(self):
        first=frame()
        second=frame(1200000000)
        second["hypotheses"]=[
            {"pair":["speech",1,"person",1],"audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,"unique_match":True,"temporally_confirmed":True},
            {"pair":["fan",1,"screen",1],"audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,"unique_match":True,"temporally_confirmed":False},
        ]
        third=frame(1400000000)
        third["hypotheses"]=[]
        skipped=frame(1600000000)
        skipped["fully_annotated"]=False
        result=score_frames([first,second,third,skipped])
        self.assertEqual(result["scored_frames"],3)
        self.assertEqual(result["skipped_incomplete_frames"],1)
        self.assertEqual([result["instantaneous"][k] for k in ("tp","fp","tn","fn")],
                         [2,1,8,1])
        self.assertEqual([result["temporal"][k] for k in ("tp","fp","tn","fn")],
                         [1,0,9,2])
        self.assertAlmostEqual(result["instantaneous"]["precision"],2/3)

    def test_jsonl_reader(self):
        row=frame()
        source=io.StringIO("\n"+json.dumps(row)+"\n")
        self.assertEqual(list(parse_jsonl(source)),[row])
        with self.assertRaises(ValueError):
            list(parse_jsonl(['{broken json']))

    def test_reject_partial_or_invalid_annotation_without_guessing(self):
        source=frame()
        for change in [
            lambda d: d.pop("fully_annotated"),
            lambda d: d.update(fully_annotated="yes"),
            lambda d: d.update(truth_pairs=[["absent",1,"person",1]]),
            lambda d: d.update(audio_tracks=[["speech",1],["speech",1]]),
            lambda d: d.update(hypotheses=[{"pair":["speech",1,"screen",1],
                          "audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,
                          "unique_match":False,"temporally_confirmed":True}]),
            lambda d: d.update(hypotheses=[{"pair":["speech",1,"screen",1],
                          "audio_scene_session_id":"a"*32,"vision_session_id":"b"*32,
                          "unique_match":True,"temporally_confirmed":"true"}]),
            lambda d: d.update(vision_session_id="b"),
            lambda d: d["hypotheses"][0].update(vision_session_id="c"*32),
        ]:
            row=copy.deepcopy(source)
            change(row)
            with self.assertRaises(ValueError):
                score_frames([row])

    def test_replay_and_unannotated_files_rejected(self):
        row=frame()
        with self.assertRaises(ValueError):
            score_frames([row,row])
        row["fully_annotated"]=False
        with self.assertRaises(ValueError):
            score_frames([row])


if __name__=="__main__":
    unittest.main()
