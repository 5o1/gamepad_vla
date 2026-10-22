#!/usr/bin/env python3
"""Pair K1 single-side witnesses; choose a placement using reviewer decisions 1/3."""
from __future__ import annotations
import argparse
from collections import Counter
from itertools import product
import json
from pathlib import Path
import time

import numpy as np
from gamepad_vla.feasibility import CollisionScene
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from k1_height_common import (CameraGate, MeshGeometry, with_support, set_placement,
    support_point, SUPPORT_NAME, defer_pad_contacts, write_json, reject_reason)


def rank_pair(pair):
    return (-len(pair['controls']), not pair['natural_pass'],0. if pair['natural_pass'] else pair['natural_deviation'],
            pair['wrist_deviation_deg'],-pair['min_camera_margin_px'])


def selected_arm(e, alt):
    return {**e,**alt}


def combined_gate(active, other, other_side):
    """Union of already projected exact meshes/points; moving pad comes from active state."""
    margins = {**active['group_margins_px'],
               **{k:v for k,v in other['group_margins_px'].items() if k!='gamepad_mesh'}}
    distances = {**active['centroid_distances_m'],other_side:other['centroid_distances_m'][other_side]}
    return dict(passed=True,margin_px=min(margins.values()),group_margins_px=margins,
                centroid_distances_m=distances,reasons=[])



def coupling_check(scene):
    """Only new pair couplings: hand/hand, cross hand/arm and arm/arm.

    Fixtures, whole-mesh table clearance, native pad contacts and camera gates
    are already certified for each retained single-side witness/IK alternative.
    """
    scene.mj.mj_forward(scene.model,scene.data)
    hand_bad=[];world_bad=[]
    for c in scene.data.contact:
        a,b=map(int,c.geom);groups={scene.groups[a],scene.groups[b]}
        if c.dist >= -1e-6:continue
        if groups not in ({'HL','HR'},{'HL','AR'},{'HR','AL'},{'AL','AR'}):continue
        row=dict(a=scene.model.body(scene.model.geom_bodyid[a]).name,
                 b=scene.model.body(scene.model.geom_bodyid[b]).name,
                 geom_a=scene.model.geom(a).name,geom_b=scene.model.geom(b).name,distance_m=float(c.dist))
        (hand_bad if groups=={'HL','HR'} else world_bad).append(row)
    return world_bad,hand_bad


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left',type=Path,required=True);parser.add_argument('--right',type=Path,required=True)
    parser.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--static-audit',type=Path,required=True)
    args=parser.parse_args();start=time.monotonic();args.output.mkdir(parents=True,exist_ok=True)
    audit=json.loads(args.static_audit.read_text())
    assert audit['passed'],'static hand/pad/support audit must pass before pairing'
    static_bad={(e['left_index'],e['right_index']) for e in audit['rejected_static_hand_pairs']}
    data={s:json.loads((getattr(args,s)/'world.json').read_text()) for s in ('left','right')}
    if any(not d['placements'] for d in data.values()):
        write_json(args.output/'scan.json',dict(stop='no relative grips; user stop condition',placements=[]));return
    cfg=load_task(args.task);spec=load_spec(cfg['workcell']);support=support_point()
    scene=CollisionScene(with_support(spec,data['left']['placements'][-1]['pose'],support),cfg['resource_root'],args.output,moving_fixtures=(SUPPORT_NAME,))
    defer_pad_contacts(scene)
    geo=MeshGeometry(scene);camera=CameraGate(next(c for c in spec.cameras if c.name=='camera_3'),geo)
    reports=[];relative_pair_cache={};all_witnesses={}
    total_goals=sum(len(data[s]['goals']) for s in data)
    for li,ri in zip(data['left']['placements'],data['right']['placements']):
        assert li['name']==ri['name']
        set_placement(scene,li['pose'],support)
        candidate_pairs={};rejects=Counter();checked_rest_pairs=0;checked_ik_pairs=0
        for le,re in product(li['witnesses'],ri['witnesses']):
            checked_rest_pairs+=1
            if (le['palm_index'],re['palm_index']) in static_bad:
                rejects['static_palm_palm_collision']+=1;continue
            restkey=(le['palm_index'],le['baseline_index'],re['palm_index'],re['baseline_index'])
            if relative_pair_cache.get(restkey) is False:
                rejects['hand_hand_collision']+=1;continue
            arms=list(product(le['arm_alternatives'],re['arm_alternatives']))
            arms.sort(key=lambda a:(not(a[0]['natural']['natural'] and a[1]['natural']['natural']),
                                    max(a[0]['natural']['deviation'],a[1]['natural']['deviation']),
                                    max(a[0]['natural']['wrist_deviation_deg'],a[1]['natural']['wrist_deviation_deg'])))
            upper=len(le['controls'])+len(re['controls'])
            best=None
            for la,ra in arms:
                checked_ik_pairs+=1
                pair={'left':selected_arm(le,la),'right':selected_arm(re,ra)}
                for side,e in pair.items():
                    scene.set_arm(side,e['arm_q']);scene.set_hand(side,pose_to_matrix(e['palm_world']),e['resting_hand_q'])
                scene.set_pad(li['pose'],{})
                worldbad,handbad=coupling_check(scene)
                if restkey not in relative_pair_cache:
                    relative_pair_cache[restkey]=not handbad
                if not relative_pair_cache[restkey]:
                    rejects['hand_hand_collision']+=1;break
                if worldbad:
                    rejects[reject_reason(worldbad,{'ring','pinky'})]+=1;continue
                controls=[];accepted_states=[];worst_margin=min(le['camera']['margin_px'],re['camera']['margin_px'])
                for side,e in pair.items():
                    other_side='right' if side=='left' else 'left';other=pair[other_side]
                    for control in e['controls']:
                        goal=next(g for g in data[side]['goals'] if g['name']==control)
                        states=[s for s in e['states'] if s['control']==control];good=[]
                        for st in states:
                            scene.set_hand(side,pose_to_matrix(e['palm_world']),st['hand_q'])
                            scene.set_hand(other_side,pose_to_matrix(other['palm_world']),other['resting_hand_q'])
                            scene.set_pad(li['pose'],st['gamepad_q'])
                            worldbad,handbad=coupling_check(scene)
                            statekey=(*restkey,side,control,st['state'])
                            if statekey not in relative_pair_cache:relative_pair_cache[statekey]=not handbad
                            passed=not worldbad and relative_pair_cache[statekey]
                            good.append(passed)
                            if passed:
                                gg=combined_gate(st['camera'],other['camera'],other_side)
                                accepted_states.append({**st,'side':side,'camera':gg})
                            else:rejects['operation:'+reject_reason(worldbad+handbad,{'ring','pinky'})]+=1
                        passed=any(good) if goal.get('alternatives') else bool(good) and len(good)==len(goal['states']) and all(good)
                        if passed:
                            controls.append(side+':'+control)
                            worst_margin=min(worst_margin,*[s['camera']['margin_px'] for s in accepted_states
                                                            if s['side']==side and s['control']==control])
                        scene.set_hand(side,pose_to_matrix(e['palm_world']),e['resting_hand_q'])
                cand=dict(left=pair['left'],right=pair['right'],controls=controls,accepted_states=accepted_states,
                    natural_pass=bool(la['natural']['natural'] and ra['natural']['natural']),
                    natural_deviation=max(la['natural']['deviation'],ra['natural']['deviation']),
                    wrist_deviation_deg=max(la['natural']['wrist_deviation_deg'],ra['natural']['wrist_deviation_deg']),
                    min_camera_margin_px=worst_margin)
                if best is None or rank_pair(cand)<rank_pair(best):best=cand
                # Starts are ordered by natural preference. When all single-side controls survive,
                # later starts cannot improve the coverage tier or the natural/wrist preference.
                if len(controls)==upper:break
            if best is not None:
                key=(le['palm_index'],re['palm_index'])
                if key not in candidate_pairs or rank_pair(best)<rank_pair(candidate_pairs[key]):candidate_pairs[key]=best
        values=list(candidate_pairs.values())
        best=min(values,key=rank_pair) if values else None
        maxcontrols=max((len(v['controls']) for v in values),default=0)
        full=sum(len(v['controls'])==total_goals for v in values)
        if best:
            all_witnesses[li['name']]=best
        # Location score uses the true triangle surface-area centre, not the root or bounding box.
        scene.set_pad(li['pose'],{});scene.mj.mj_forward(scene.model,scene.data)
        centroid=geo.centroid('pad');uv,_=camera.project(centroid[None])
        centre_error=float(np.linalg.norm(uv[0]-[camera.k.cx,camera.k.cy]))
        report={k:li[k] for k in ('name','height_mm','offset_mm','pose','optical_axis_plane_intersection_m')}
        report.update(full_control_pairs=full,max_controls=maxcontrols,grip_pairs=len(values),
            single={s:{k:r[k] for k in ('relative_grip_palms','arm_ik_palms','mesh_clearance_30mm_palms',
                         'camera_grip_palms','per_control','full_hand_palms','rejected_palms')}
                    for s,r in [('left',li),('right',ri)]},
            checked_rest_pairs=checked_rest_pairs,checked_ik_pairs=checked_ik_pairs,pair_rejections=dict(rejects),
            gamepad_centroid_world_m=centroid.tolist(),gamepad_centroid_pixel=uv[0].tolist(),centre_error_px=centre_error,
            natural_pass=best['natural_pass'] if best else False,
            natural_deviation=best['natural_deviation'] if best else None,
            wrist_deviation_deg=best['wrist_deviation_deg'] if best else None,
            min_camera_margin_px=best['min_camera_margin_px'] if best else None,
            best_controls=best['controls'] if best else [],
            natural={s:best[s]['natural'] for s in ('left','right')} if best else None)
        reports.append(report)
        if len(reports)%49==0:
            write_json(args.output/'progress.json',reports)
            print(json.dumps(dict(stage='pair_height',height_mm=li['height_mm'],
                pairs=sum(r['grip_pairs'] for r in reports[-49:]),max_controls=max(r['max_controls'] for r in reports[-49:]))),flush=True)
    anyfull=any(r['full_control_pairs'] for r in reports)
    tierkey='full_control_pairs' if anyfull else 'max_controls'
    tier=max((r[tierkey] for r in reports if r['grip_pairs']),default=0)
    eligible=[r for r in reports if r['grip_pairs'] and r[tierkey]==tier]
    natural=[r for r in eligible if r['natural_pass']]
    natural_available=bool(natural)
    if natural:eligible=natural
    elif eligible:
        deviation=min(r['natural_deviation'] for r in eligible)
        eligible=[r for r in eligible if abs(r['natural_deviation']-deviation)<1e-9]
    choice=min(eligible,key=lambda r:(r['height_mm'],r['wrist_deviation_deg'],r['centre_error_px'],
                                       -r['min_camera_margin_px'])) if eligible else None
    out=dict(placements=reports,coverage_key=tierkey,best_tier=tier,total_required_controls=total_goals,
             natural_available_at_best_tier=natural_available,choice=choice,
             acceptable_heights_mm=sorted({r['height_mm'] for r in reports if r['grip_pairs'] and r[tierkey]==tier and r['natural_pass']}),
             geometric_best_tier_heights_mm=sorted({r['height_mm'] for r in reports if r['grip_pairs'] and r[tierkey]==tier}),
             elapsed_seconds=time.monotonic()-start,
             stop='all camera/world candidates failed; user stop condition' if choice is None else None)
    write_json(args.output/'scan.json',out)
    if choice:write_json(args.output/'selected_pair.json',dict(placement=choice,**all_witnesses[choice['name']]))
    write_json(args.output/'best_pairs.json',all_witnesses)
    print(json.dumps(dict(stage='pairs_done',seconds=out['elapsed_seconds'],choice=choice)),flush=True)


if __name__=='__main__':main()
