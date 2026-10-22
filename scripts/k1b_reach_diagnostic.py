#!/usr/bin/env python3
"""Read-only FK-cloud diagnosis of native-valid K1-b fixed-palm grips.

Counterfactual wrist rolls below are geometric comparisons only. They do not
preserve native contact, natural arm posture, collision or camera acceptance.
"""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from gamepad_vla.feasibility import transform
from k1b_search import Problem
from k1_height_common import write_json


def target_distances(problem, palm_in_pad):
    out = {}
    for goal in problem.goals:
        finger = problem.fingers[goal['finger']]
        points = np.array([s['point'] for s in goal['states']])
        normals = -np.array([s['normal'] for s in goal['states']])
        target = (points-palm_in_pad[:3, 3]) @ palm_in_pad[:3, :3]
        inward = normals @ palm_in_pad[:3, :3]
        pos_dist, pos_id = finger.position_tree.query(target, workers=1)
        cone_dist, cone_id = finger.tree.query(np.c_[target, .015*inward], workers=1)
        rows = []
        for i in range(len(target)):
            rows.append(dict(state=i, target_in_palm_m=target[i].tolist(),
                min_position_distance_m=float(pos_dist[i]),
                position_nearest_normal_cos=float(finger.nrm[pos_id[i]] @ inward[i]),
                position_nearest_mode=int(pos_id[i]//len(finger.q)),
                min_weighted_point_normal_distance_m=float(cone_dist[i]),
                normal_nearest_position_error_m=float(np.linalg.norm(finger.xyz[cone_id[i]]-target[i])),
                normal_nearest_cos=float(finger.nrm[cone_id[i]] @ inward[i]),
                normal_nearest_mode=int(cone_id[i]//len(finger.q))))
        # Required stick endpoints use max; alternative trigger patches use min.
        reducer = min if goal.get('alternatives') else max
        out[goal['name']] = dict(states=rows, alternatives=bool(goal.get('alternatives')),
            required_cloud_distance_m=reducer(float(v) for v in pos_dist),
            normal_cone_required=goal['name'] not in ('LS','RS','LT','RT'))
    return out


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--root', type=Path, required=True)
    a.add_argument('--task', type=Path, default=Path('configs/tasks/fixed_dpad_causal.json'))
    a.add_argument('--output', type=Path, required=True)
    args = a.parse_args()
    output = dict(scope='100k random FK samples per finger, geometric distance not impossibility proof; native results read unchanged', sides={})
    for side in ('left','right'):
        problem = Problem(args.task, side)
        native = json.loads((args.root/f'native-{side}'/'native.json').read_text())
        entries = []
        thumb = problem.fingers['thumb']
        zero = thumb.chain.link_poses(np.zeros((1, len(thumb.lower))))
        zero_root = zero[thumb.chain.joints[0]['child']][0]
        zero_tip = zero[thumb.chain.tip_link][0]
        # Origins of the first movable finger joints are their base points.
        ring = problem.fingers['ring']
        ring_zero = ring.chain.link_poses(np.zeros((1, len(ring.lower))))
        ring_base = ring_zero[ring.chain.joints[0]['child']][0,:3,3]
        thumb_base = zero_root[:3,3]
        thumb_side = np.array([0.,np.sign(thumb_base[1]-ring_base[1]),0.])
        transverse = thumb_base-ring_base
        transverse[2] = 0.
        normal_counters = Counter()
        for index, entry in enumerate(native['accepted']):
            pad, palm, _, _, frames = problem.unpack(np.array(entry['x']))
            pp = np.linalg.inv(pad) @ palm
            bthumb = transform(thumb_base, pp)
            bring = transform(ring_base, pp)
            inward_sign = 1 if side=='left' else -1
            diagnostics = target_distances(problem, pp)
            for name, goal in diagnostics.items():
                prior = entry.get('control_diagnostics',{}).get(name,{})
                goal['existing_point_ik_solutions'] = sum(s['point_ik_solutions'] for s in prior.get('states',[]))
                goal['existing_native_rejections'] = prior.get('rejections',{})
                normal_counters.update(prior.get('rejections',{}))
            # Roll around the hand extension axis while anchoring the mean of
            # the actual ring/pinky contact points, not the arbitrary wrist.
            local_anchor = np.mean([transform(problem.fingers[f].points[0], frames[f+'_distal'])
                                    for f in ('ring','pinky')], axis=0)
            anchor = transform(local_anchor, pp)
            rolls = []
            for angle in (-90,-60,-30,0,30,60,90,180):
                candidate = pp.copy()
                candidate[:3,:3] = pp[:3,:3] @ Rotation.from_euler('z',angle,degrees=True).as_matrix()
                candidate[:3,3] = anchor-candidate[:3,:3]@local_anchor
                dd = target_distances(problem,candidate)
                thumb_dist = {g['name']:dd[g['name']]['required_cloud_distance_m']
                              for g in problem.goals if g['finger']=='thumb'}
                rolls.append(dict(roll_deg=angle, thumb_required_distances_m=thumb_dist,
                    sum_thumb_required_distance_m=sum(thumb_dist.values()),
                    finger_axis_world=(pad[:3,:3]@candidate[:3,2]).tolist(),
                    finger_pad_axis_world=(pad[:3,:3]@candidate[:3,0]).tolist(),
                    thumb_lateral_alignment_inward_cos=float(inward_sign*(candidate[:3,:3]@thumb_side)[0]),
                    thumb_inward_from_ring_base_m=float(inward_sign*(transform(thumb_base,candidate)[0]-transform(ring_base,candidate)[0]))))
            entries.append(dict(index=index,source_seed_index=entry['source_seed_index'],
                gamepad_position_m=pad[:3,3].tolist(), palm_position_m=palm[:3,3].tolist(),
                palm_axes_world=palm[:3,:3].T.tolist(),palm_axes_in_pad=pp[:3,:3].T.tolist(),
                thumb_base_in_pad_m=bthumb.tolist(),ring_base_in_pad_m=bring.tolist(),
                thumb_inward_from_ring_base_m=float(inward_sign*(bthumb[0]-bring[0])),
                thumb_lateral_alignment_inward_cos=float(inward_sign*(pp[:3,:3]@thumb_side)[0]),
                thumb_transverse_inward_m=float(inward_sign*(pp[:3,:3]@transverse)[0]),
                ring_pad_normal_world=(palm[:3,:3]@ring.normals[0]).tolist(),
                thumb_zero_tip_axis_world=(palm[:3,:3]@zero_tip[:3,2]).tolist(),
                thumb_zero_tip_position_pad_m=transform(zero_tip[:3,3],pp).tolist(),
                natural=entry['check']['natural'], controls=diagnostics, wrist_roll_counterfactuals=rolls))
        goals = {}
        for goal in problem.goals:
            name = goal['name']
            dd = np.array([e['controls'][name]['required_cloud_distance_m'] for e in entries])
            goals[name] = dict(required_cloud_distance_min_mm=1000*float(dd.min()),
                required_cloud_distance_median_mm=1000*float(np.median(dd)),
                required_cloud_distance_max_mm=1000*float(dd.max()),
                total_point_ik_solutions=sum(e['controls'][name]['existing_point_ik_solutions'] for e in entries))
        output['sides'][side] = dict(count=len(entries), thumb_base_local_m=thumb_base.tolist(),
            ring_base_local_m=ring_base.tolist(),thumb_zero_axes_local=zero_tip[:3,:3].T.tolist(),
            thumb_lateral_axis_local=thumb_side.tolist(),ring_pad_normal_local=ring.normals[0].tolist(),
            finger_joint_axes_and_origins=[dict(name=j['name'],axis=j['axis'].tolist(),origin=j['origin'].tolist()) for j in thumb.chain.joints],
            thumb_inward_count=sum(e['thumb_inward_from_ring_base_m']>0 for e in entries),
            thumb_lateral_inward_count=sum(e['thumb_lateral_alignment_inward_cos']>0 for e in entries),
            thumb_lateral_inward_cos_range=[min(e['thumb_lateral_alignment_inward_cos'] for e in entries),max(e['thumb_lateral_alignment_inward_cos'] for e in entries)],
            thumb_inward_range_mm=[1000*min(e['thumb_inward_from_ring_base_m'] for e in entries),1000*max(e['thumb_inward_from_ring_base_m'] for e in entries)],
            native_rejections=dict(normal_counters),goals=goals,candidates=entries)
        write_json(args.output,output)
        print(json.dumps(dict(side=side,count=len(entries),thumb_inward=output['sides'][side]['thumb_inward_count'],goals=goals)),flush=True)


if __name__=='__main__': main()
