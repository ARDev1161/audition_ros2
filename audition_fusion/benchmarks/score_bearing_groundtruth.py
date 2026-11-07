#!/usr/bin/env python3
"""Evaluate *diagnostic* planar intersections against surveyed 2D truth.

A dataset must explicitly annotate every active source track, including
unlocalized/false-reflection tracks, before a frame is scored. This scorer
never asserts the candidate is a validated physical 3D source location.
"""
import argparse
import json
import math
import re


def finite_pair(value):
    return (isinstance(value,list) and len(value)==2
            and all(type(x) in (int,float) and math.isfinite(x) for x in value))


def identity(row):
    if (not isinstance(row,dict) or not isinstance(row.get('source_id'),str)
            or not row['source_id'] or len(row['source_id'].encode('utf8'))>256
            or type(row.get('source_generation')) is not int
            or row['source_generation']<1):
        raise ValueError('invalid source identity')
    return (row['source_id'],row['source_generation'])


def checked_entries(values,limit,name):
    if not isinstance(values,list) or len(values)>limit:
        raise ValueError('invalid '+name)
    output={}
    for entry in values:
        key=identity(entry)
        if key in output:
            raise ValueError('duplicate '+name)
        output[key]=entry
    return output


def percentile(data,fraction):
    if not data:
        return None
    sorted_values=sorted(data)
    return sorted_values[max(0,math.ceil(len(data)*fraction)-1)]


def score(frames,*,maximum_position_error_m=1.0):
    if (type(maximum_position_error_m) not in (int,float)
            or not math.isfinite(maximum_position_error_m)
            or maximum_position_error_m<=0):
        raise ValueError('invalid position error threshold')
    tp=fp=tn=fn=within_tolerance=0
    position_errors=[]
    range_errors=[]
    scored=skipped=0
    last_stamp=0
    for frame in frames:
        if (not isinstance(frame,dict) or frame.get('schema')!='bearing_eval_v1'):
            raise ValueError('invalid frame schema')
        stamp=frame.get('stamp_ns')
        if type(stamp) is not int or stamp<=last_stamp:
            raise ValueError('frame stamps not strictly increasing')
        last_stamp=stamp
        session=frame.get('scene_session_id')
        if not isinstance(session,str) or not re.fullmatch('[0-9a-f]{32}',session):
            raise ValueError('invalid scene session')
        complete=frame.get('fully_annotated')
        if type(complete) is not bool:
            raise ValueError('unknown annotation completeness')
        if not complete:
            skipped+=1
            continue
        frame_id=frame.get('frame_id')
        if (not isinstance(frame_id,str) or not frame_id or frame_id.startswith('/')
                or not finite_pair(frame.get('robot_xy_m'))):
            raise ValueError('invalid metric ground-truth coordinate frame')
        origin=frame['robot_xy_m']
        truth=checked_entries(frame.get('sources'),16,'annotated sources')
        candidates=checked_entries(frame.get('candidates'),16,'candidate positions')
        if not set(candidates).issubset(truth):
            raise ValueError('candidate missing independent source annotation')
        for item in candidates.values():
            if item.get('scene_session_id')!=session or item.get('frame_id')!=frame_id:
                raise ValueError('candidate session/frame mismatch')
            xy=item.get('xy_m')
            if not finite_pair(xy):
                raise ValueError('non-metric candidate point')
            if (type(item.get('supporting_observations')) is not int
                    or item['supporting_observations']<3
                    or type(item.get('baseline_m')) not in (int,float)
                    or not math.isfinite(item['baseline_m'])
                    or item['baseline_m']<=0):
                raise ValueError('inadequate candidate provenance')
        for key,item in truth.items():
            point=item.get('truth_xy_m')
            if point is not None and not finite_pair(point):
                raise ValueError('invalid surveyed source coordinates')
            emitted=candidates.get(key)
            if point is None:
                if emitted is None:
                    tn+=1
                else:
                    fp+=1
            elif emitted is None:
                fn+=1
            else:
                tp+=1
                pred=emitted['xy_m']
                position_error=math.dist(pred,point)
                range_error=abs(math.dist(pred,origin)-math.dist(point,origin))
                position_errors.append(position_error)
                range_errors.append(range_error)
                within_tolerance+=position_error<=maximum_position_error_m
        scored+=1
    if scored<1:
        raise ValueError('no fully annotated frames')
    return {'schema':'bearing_eval_results_v1','scored_frames':scored,
            'skipped_incomplete_frames':skipped,
            'tp':tp,'fp':fp,'tn':tn,'fn':fn,
            'candidate_precision':tp/(tp+fp) if tp+fp else 0.,
            'candidate_recall':tp/(tp+fn) if tp+fn else 0.,
            'position_error_m_p50':percentile(position_errors,.5),
            'position_error_m_p95':percentile(position_errors,.95),
            'absolute_range_error_m_p50':percentile(range_errors,.5),
            'absolute_range_error_m_p95':percentile(range_errors,.95),
            'position_within_tolerance':within_tolerance,
            'maximum_position_error_m':maximum_position_error_m,
            'notes':['truth source association is externally provided',
                     'candidate precision does not measure metric position quality',
                     'range error is only evaluated for emitted candidates',
                     'missing emitted positions count as recall failures',
                     '2D planar only, no calibrated Z/covariance/physical safety']}


def read_jsonl(lines):
    for index,line in enumerate(lines,1):
        if len(line)>1000000:
            raise ValueError('line exceeds maximum length: '+str(index))
        if line.strip():
            try:
                yield json.loads(line)
            except json.JSONDecodeError as e:
                raise ValueError('invalid JSON on line '+str(index)) from e


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('labeled_jsonl')
    parser.add_argument('--maximum-position-error-m',type=float,default=1.)
    args=parser.parse_args()
    try:
        with open(args.labeled_jsonl,encoding='utf8') as stream:
            report=score(read_jsonl(stream),
                         maximum_position_error_m=args.maximum_position_error_m)
    except (ValueError,OSError) as e:
        parser.error(str(e))
    print(json.dumps(report,sort_keys=True,indent=2))


if __name__=='__main__':
    main()
