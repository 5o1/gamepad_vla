#!/usr/bin/env python3
"""Search a small table-plane translation for a four-direction reachable fixed pad.

Keeps AR5 pose, gamepad height and yaw unchanged. This is kinematic selection;
run the contact baseline on its output before learning or claiming feasibility.
"""
import argparse
import json
import os
from pathlib import Path
from dataclasses import replace

import numpy as np
from scipy.spatial import cKDTree

from gamepad_vla.kinematics import SerialChain
from gamepad_vla.tasks.fixed_gamepad import load_task, ThumbIK
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path('configs/tasks/fixed_dpad.json'))
    parser.add_argument('--inspection',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--radius',type=float,default=.04)
    parser.add_argument('--samples',type=int,default=400000)
    args=parser.parse_args()
    config=load_task(args.config)
    cell=load_spec(config['workcell'])
    hand=np.array(json.loads(args.inspection.read_text())['hand_world'])
    chain=SerialChain(cell.left_hand.urdf,config['thumb_link'])
    lo,hi=chain.active_limits
    q=np.random.default_rng(0).uniform(lo,hi,(args.samples,len(lo)))
    frames=chain.forward(q)
    points=(frames[:,:3,:3]@config['thumb_contact_point_m']+frames[:,:3,3])@hand[:3,:3].T+hand[:3,3]
    normals=(frames[:,:3,:3]@config['thumb_contact_normal'])@hand[:3,:3].T
    pad=pose_to_matrix(cell.gamepad.root_pose)
    normal=pad[:3,2]
    aligned=normals@normal < -np.cos(np.deg2rad(30))
    tree=cKDTree(points[aligned])
    centre=pad[:3,:3]@config['dpad_centre_m']+pad[:3,3]
    pattern=np.array([centre+config['hover_height_m']*normal]+[
        centre+config['press_radius_m']*v-config['press_depth_m']*normal
        for v in (pad[:3,1],-pad[:3,1],-pad[:3,0],pad[:3,0])])
    a=np.linspace(-args.radius,args.radius,61)
    offsets=np.array([(x,y,0) for x in a for y in a])
    distance,_=tree.query(pattern[None,:,:]+offsets[:,None,:])
    order=np.argsort(distance.max(axis=1))[:8]
    results=[]
    for i in order:
        pose=np.array(cell.gamepad.root_pose);pose[:3]+=offsets[i]
        candidate=replace(cell,gamepad=replace(cell.gamepad,root_pose=tuple(pose)))
        ik=ThumbIK(config,candidate,hand)
        results.append(dict(pose=pose.tolist(),ik_error_m=ik.errors,offset_m=offsets[i].tolist()))
    best=min(results,key=lambda r:max(r['ik_error_m']))
    raw=json.loads(args.config.read_text())
    # Write portable workspace references, retaining other task options.
    raw['workcell']=os.path.relpath(config['workcell'],args.output.parent.resolve())
    raw['gamepad_root_pose']=best['pose']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(raw,indent=2)+'\n')
    args.output.with_suffix('.search.json').write_text(json.dumps(dict(best=best,candidates=results),indent=2)+'\n')
    print(json.dumps(best),flush=True)


if __name__=='__main__':
    main()
