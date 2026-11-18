"""Strict ground truth scoring for *planar* candidate diagnostics."""
import copy
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'benchmarks'))
from score_bearing_groundtruth import score

SESSION='1'*32


def frame(stamp=1000000000):
    return {'schema':'bearing_eval_v1','stamp_ns':stamp,'fully_annotated':True,
            'scene_session_id':SESSION,'frame_id':'odom',
            'robot_xy_m':[0.,0.],
            'sources':[{'source_id':'speaker','source_generation':1,
                        'truth_xy_m':[3.,4.]},
                       {'source_id':'reflection','source_generation':2,
                        'truth_xy_m':None}],
            'candidates':[{'scene_session_id':SESSION,'frame_id':'odom',
                           'source_id':'speaker','source_generation':1,
                           'xy_m':[3.,4.],'supporting_observations':3,
                           'baseline_m':.8}]}


class RangeScoringTests(unittest.TestCase):
    def test_presence_range_and_position_separate(self):
        a=frame()
        b=frame(1200000000)
        b['candidates']=[{'scene_session_id':SESSION,'frame_id':'odom',
                          'source_id':'reflection','source_generation':2,
                          'xy_m':[7.,2.],'supporting_observations':3,
                          'baseline_m':.7}]
        c=frame(1400000000)
        c['candidates'][0]['xy_m']=[3.6,4.8]
        result=score([a,b,c],maximum_position_error_m=.5)
        self.assertEqual([result[k] for k in ('tp','fp','tn','fn')],[2,1,2,1])
        self.assertEqual(result['position_within_tolerance'],1)
        self.assertAlmostEqual(result['position_error_m_p50'],0.)
        self.assertAlmostEqual(result['position_error_m_p95'],1.)
        self.assertAlmostEqual(result['absolute_range_error_m_p95'],1.)
        self.assertAlmostEqual(result['candidate_precision'],2/3)

    def test_annotation_incomplete_does_not_count_negatives(self):
        good=frame()
        ignored=frame(1200000000)
        ignored['fully_annotated']=False
        ignored['sources']=[]
        ignored['candidates']=[]
        result=score([good,ignored])
        self.assertEqual(result['scored_frames'],1)
        self.assertEqual(result['skipped_incomplete_frames'],1)
        with self.assertRaises(ValueError):
            score([ignored])

    def test_wrong_sessions_and_bad_coordinates_rejected(self):
        original=frame()
        for mutation in (
            lambda v:v['candidates'][0].update(scene_session_id='2'*32),
            lambda v:v['candidates'][0].update(frame_id='map'),
            lambda v:v['candidates'][0].update(xy_m=[float('nan'),0]),
            lambda v:v['candidates'][0].update(supporting_observations=1),
            lambda v:v['candidates'][0].update(baseline_m=0),
            lambda v:v['sources'][0].update(truth_xy_m='unknown'),
            lambda v:v.update(frame_id='/odom'),
            lambda v:v.update(robot_xy_m=None),
            lambda v:v['candidates'][0].update(source_id='unannotated'),
            lambda v:v['candidates'].append(copy.deepcopy(v['candidates'][0])),
        ):
            candidate=copy.deepcopy(original)
            mutation(candidate)
            with self.assertRaises(ValueError):
                score([candidate])

    def test_time_replay_and_zero_positives(self):
        a=frame()
        with self.assertRaises(ValueError):
            score([a,a])
        a['candidates']=[]
        a['sources'][0]['truth_xy_m']=None
        value=score([a])
        self.assertIsNone(value['position_error_m_p95'])
        self.assertEqual(value['tn'],2)


if __name__=='__main__':
    unittest.main()
