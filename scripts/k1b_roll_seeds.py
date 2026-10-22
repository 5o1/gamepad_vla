#!/usr/bin/env python3
"""Generate natural-arm IK seeds by rolling native-valid grip palm frames.

The original pad placement and finger joints are retained. Rotated palms only
provide new optimization initial values; these outputs have no native-contact,
collision, clearance or camera acceptance certificate.
"""
from __future__ import annotations
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from gamepad_vla.feasibility import Finger, Pad, mesh_for_geom, transform
from gamepad_vla.kinematics import SerialChain
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix, matrix_to_pose
from k1_natural_reference import ArmFeatures
from k1_height_common import write_json
from search_gamepad_height import multistart_ik


ROLLS_DEG = (-90., -60., -30., 30., 60., 90.)


def natural_dict(metrics, row):
    return {key: bool(value[row]) if np.asarray(value).dtype.kind=='b' else float(value[row])
            for key,value in metrics.items() if np.asarray(value).ndim==1}


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--root',type=Path,required=True)
    a.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    a.add_argument('--output',type=Path,required=True)
    args = a.parse_args()
    cfg = load_task(args.task)
    spec = load_spec(cfg['workcell'])
    padmodel = Pad(cfg['resource_root'])
    padmodel.state({})
    # Match Problem.pad_centre exactly: all original compiled gamepad meshes.
    whole = trimesh.util.concatenate([mesh_for_geom(padmodel.model,padmodel.data,g)
        for g in range(padmodel.model.ngeom) if padmodel.model.geom_dataid[g]>=0])
    pad_centroid = np.array(whole.centroid)
    for side in ('left','right'):
        arm = getattr(spec,side+'_arm')
        hand = getattr(spec,side+'_hand')
        feature = ArmFeatures(arm,hand)
        rng = np.random.default_rng(8821)
        gripfingers = {name:Finger(hand.urdf,name,1,rng) for name in ('ring','pinky')}
        thumbchain = SerialChain(hand.urdf,'thumb_distal')
        thumbzero = thumbchain.link_poses(np.zeros((1,len(thumbchain.active))))
        thumbbase = thumbzero[thumbchain.joints[0]['child']][0,:3,3]
        ringchain = gripfingers['ring'].chain
        ringzero = ringchain.link_poses(np.zeros((1,len(ringchain.active))))
        ringbase = ringzero[ringchain.joints[0]['child']][0,:3,3]
        thumbside = np.array([0.,np.sign(thumbbase[1]-ringbase[1]),0.])
        inward_sign = 1 if side=='left' else -1
        native = json.loads((args.root/f'native-{side}'/'native.json').read_text())
        seeds = []
        diagnostics = []
        for index,entry in enumerate(native['accepted']):
            x = np.array(entry['x'])
            palm = feature.frames(x[4:11])['palm'][0]
            padpose = pose_to_matrix(entry['check']['gamepad_pose'])
            padcentre = transform(pad_centroid,padpose)
            handq = entry['check']['hand_q']
            contact_local = []
            for name,finger in gripfingers.items():
                q = np.array([handq[n] for n in finger.chain.active])
                frame = finger.chain.forward([q])[0]
                contact_local.append(transform(finger.points[0],frame))
            midpoint_local = np.mean(contact_local,axis=0)
            midpoint_world = transform(midpoint_local,palm)
            targets = []
            for angle in ROLLS_DEG:
                target = palm.copy()
                target[:3,:3] = palm[:3,:3]@Rotation.from_euler('z',angle,degrees=True).as_matrix()
                target[:3,3] = midpoint_world-target[:3,:3]@midpoint_local
                targets.append(target)
            targets = np.array(targets)
            # Retain all starts, then select among accurate AND natural solutions.
            solutions = multistart_ik(replace(arm,home_q=tuple(x[4:11])),hand,targets,
                                      seed=9311+index+1000*(side=='right'),iterations=100)
            metrics = [feature.measure(s[0],np.tile(padcentre,(len(targets),1))) for s in solutions]
            for row,angle in enumerate(ROLLS_DEG):
                starts = []
                valid = []
                for start,(q,accurate,pe,re) in enumerate(solutions):
                    natural = natural_dict(metrics[start],row)
                    starts.append(dict(start=start,q=q[row].tolist(),accurate=bool(accurate[row]),
                        position_error_m=float(pe[row]),rotation_error_rad=float(re[row]),natural=natural))
                    if accurate[row] and natural['natural']:
                        valid.append(start)
                record = dict(source_index=index,roll_deg=angle,target_palm_world=list(matrix_to_pose(targets[row])),
                    contact_midpoint_world_m=midpoint_world.tolist(),starts=starts)
                if valid:
                    selected = min(valid,key=lambda k:(float(metrics[k]['deviation'][row]),
                                                      float(metrics[k]['wrist_deviation_deg'][row])))
                    q,_,pe,re = solutions[selected]
                    xx = x.copy()
                    xx[4:11] = q[row]
                    actual = feature.frames(q[row])['palm'][0]
                    pp = np.linalg.inv(padpose)@actual
                    seed = dict(x=xx.tolist(),source_index=index,roll_deg=angle,selected_start=selected,
                        natural=natural_dict(metrics[selected],row),
                        ik_error=dict(position_m=float(pe[row]),rotation_rad=float(re[row])),
                        thumb_lateral_alignment_inward_cos=float(inward_sign*(pp[:3,:3]@thumbside)[0]))
                    seeds.append(seed)
                    record['selected_start'] = selected
                diagnostics.append(record)
            print(json.dumps(dict(stage='rolled_source',side=side,source_index=index,seeds=len(seeds))),flush=True)
            write_json(args.output/f'{side}.json',dict(seeds=seeds,diagnostics=diagnostics,
                gamepad_centroid_local_m=pad_centroid.tolist(),rolls_deg=ROLLS_DEG,
                scope='Optimization seeds only. Accurate six-start K0 IK and actual-centroid natural hard gates; no native/contact/clearance/camera acceptance.'))
        print(json.dumps(dict(stage='rolled_done',side=side,sources=len(native['accepted']),targets=len(diagnostics),seeds=len(seeds))),flush=True)


if __name__=='__main__': main()
