#!/usr/bin/env python3
"""Sample fixed-palm grasps without training; run on the configured compute node.

Example: PYTHONPATH=src python scripts/check_two_hand_feasibility.py \
  --side left --output "$GAMEPAD_VLA_WORKSPACE/runs/k0/left"
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from gamepad_vla.feasibility import (
    Pad, Finger, CollisionScene, FINGERS, fixtures, palm_candidates, local_targets,
    arm_ik, POINT_TOL,
)
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix, matrix_to_pose


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    p.add_argument('--side',choices=['left','right'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=100000,help='FK samples per finger')
    p.add_argument('--palms',type=int,default=4096)
    p.add_argument('--shell-samples',type=int,default=40000)
    p.add_argument('--seed',type=int,default=21)
    p.add_argument('--reuse-kinematics',type=Path,help='Reuse saved FK/IK arrays from the same sampled run after a collision-only change')
    p.add_argument('--arm-seeds',type=int,default=6)
    p.add_argument('--arm-iterations',type=int,default=100)
    args=p.parse_args();start=time.monotonic()
    args.output.mkdir(parents=True,exist_ok=True)
    cfg=load_task(args.task);spec=load_spec(cfg['workcell']);side=args.side
    hand,arm=getattr(spec,side+'_hand'),getattr(spec,side+'_arm')
    rng=np.random.default_rng(args.seed+(side=='right'))
    pad=Pad(cfg['resource_root']);goals=pad.targets(side)
    fingers={name:Finger(hand.urdf,name,args.samples,rng) for name in FINGERS}
    points,normals=pad.grips(side,args.shell_samples,args.seed)
    if not len(points):raise ValueError('empty rear grip patch')
    print(json.dumps(dict(stage='assets',grip_surface_samples=len(points),goals=[g['name'] for g in goals])),flush=True)
    if args.reuse_kinematics:
        cached=np.load(args.reuse_kinematics)
        palms,ringq,ringids,pinkq,pe,pdot,grip_ids,coverage=[cached[k] for k in
            ('palms','ringq','ringids','pinkq','pinkerror','pinkdot','grip_ids','coverage')]
        results={g['name']:{k:cached[g['name']+'_'+k] for k in ('q','mode','error','dot','ok')} for g in goals}
        print(json.dumps(dict(stage='reused_kinematics',grips=len(grip_ids),palms=len(palms))),flush=True)
    else:
        palms,ringq,ringids=palm_candidates(fingers['ring'],points,normals,args.palms,rng)
        # Try shell points nearest the pinky workspace's centre, rather than assuming symmetry.
        pc=fingers['pinky'].xyz.mean(axis=0)
        centre=np.einsum('nij,j->ni',palms[:,:3,:3],pc)+palms[:,:3,3]
        from scipy.spatial import cKDTree
        _,neighbors=cKDTree(points).query(centre,k=min(24,len(points)))
        # Each palm has a different set of candidate shell points.
        lp=np.einsum('nji,ntj->nti',palms[:,:3,:3],points[neighbors]-palms[:,None,:3,3])
        ln=np.einsum('nji,ntj->nti',palms[:,:3,:3],-normals[neighbors])
        dist,_=fingers['pinky'].nearest_seed(lp.reshape(-1,3),ln.reshape(-1,3),normal_cone=False)
        best=dist.reshape(len(palms),-1).argmin(axis=1)
        dest=lp[np.arange(len(palms)),best];n=ln[np.arange(len(palms)),best]
        pinkq,pinkmode,pe,pdot,pok=fingers['pinky'].fit(dest,n,normal_cone=False)
        # A 90-degree pad is the only allowed grip mode for ring and pinky.
        grip_ids=np.where(pok)[0]
        print(json.dumps(dict(stage='grip_ik',passed=len(grip_ids),total=len(palms))),flush=True)
        results={};coverage=np.zeros((len(palms),len(goals)),bool)
        for gi,g in enumerate(goals):
            f=fingers[g['finger']]
            gp=np.array([s['point'] for s in g['states']]);gn=-np.array([s['normal'] for s in g['states']])
            tp,tn=local_targets(palms[grip_ids],gp,gn)
            dist,_=f.nearest_seed(tp.reshape(-1,3),tn.reshape(-1,3),normal_cone=g['name'] not in ('LS','RS','LT','RT'))
            # Broad phase is not acceptance: every survivor is refined and measured.
            likely=dist<.012
            q=np.zeros((len(dist),len(f.lower)));m=np.zeros(len(dist),int)
            error=np.full(len(dist),np.inf);dot=np.full(len(dist),-1.)
            ok=np.zeros(len(dist),bool)
            if likely.any():
                q[likely],m[likely],error[likely],dot[likely],ok[likely]=f.fit(tp.reshape(-1,3)[likely],tn.reshape(-1,3)[likely],normal_cone=g['name'] not in ('LS','RS','LT','RT'))
            ok=ok.reshape(len(grip_ids),-1)
            covered=ok.any(axis=1) if g.get('alternatives') else ok.all(axis=1)
            coverage[grip_ids,gi]=covered
            results[g['name']]=dict(q=q.reshape(len(grip_ids),-1,q.shape[-1]),mode=m.reshape(ok.shape),
                                      error=error.reshape(ok.shape),dot=dot.reshape(ok.shape),ok=ok)
            print(json.dumps(dict(stage='control_ik',control=g['name'],covered=int(covered.sum()))),flush=True)
        np.savez_compressed(args.output/'kinematics.npz',palms=palms,ringq=ringq,ringids=ringids,pinkq=pinkq,
                            pinkerror=pe,pinkdot=pdot,grip_ids=grip_ids,coverage=coverage,
                            **{name+'_'+k:v for name,rr in results.items() for k,v in rr.items()})
    scene=CollisionScene(spec,cfg['resource_root'],args.output)
    fixture_results=[];witnesses={}
    # Include grip-only samples as evidence even if no assigned control survives IK.
    candidates=grip_ids
    rng=np.random.default_rng(args.seed+1000+(side=='right'))
    for fx in fixtures(cfg['gamepad_root_pose']):
        worldpad=pose_to_matrix(fx['pose']);worldpalms=worldpad@palms[candidates]
        if len(candidates):
            aq,aok,ape,are=arm_ik(arm,hand,worldpalms,rng,seeds=args.arm_seeds,iterations=args.arm_iterations)
        else:
            aq=np.zeros((0,7));aok=np.zeros(0,bool);ape=are=np.zeros(0)
        counts={g['name']:0 for g in goals};combos={};examples=[];rejects={};valid_grips=0
        for progress,row in enumerate(np.where(aok)[0]):
            if progress%200==0:
                print(json.dumps(dict(stage='collision_progress',fixture=fx['name'],checked=progress,total=int(aok.sum()))),flush=True)
            pi=int(candidates[row]);localrow=int(np.searchsorted(grip_ids,pi))
            qgrip={**dict(zip(fingers['ring'].chain.active,ringq[pi])),**dict(zip(fingers['pinky'].chain.active,pinkq[pi]))}
            scene.set_arm(side,aq[row]);palm=worldpalms[row]
            baseline=None
            # Search resting postures for the other three fingers, with grip fixed.
            rest_candidates=[]
            for j in range(12):
                q=qgrip.copy()
                for name in ('thumb','index','middle'):
                    f=fingers[name]
                    if j==0:values=f.lower
                    elif j==1:values=f.upper
                    else:values=f.q[(pi*13+j*997)%len(f.q)]
                    q.update(zip(f.chain.active,values))
                rest_candidates.append(q)
            # Prioritize a known reachable thumb posture as another resting option.
            for gi,g in enumerate(goals):
                if coverage[pi,gi]:
                    q=rest_candidates[0].copy();rr=results[g['name']]
                    j=int(np.where(rr['ok'][localrow])[0][0]);q.update(zip(fingers[g['finger']].chain.active,rr['q'][localrow,j]))
                    rest_candidates.insert(0,q)
            for q in rest_candidates:
                scene.set_pad(fx['pose'],{});scene.set_hand(side,palm,q)
                clear,bad,grips,minimum=scene.check(side)
                if clear and grips=={'ring','pinky'}:
                    baseline=q;break
                # Resting the active fingers cannot repair a fixed palm/grip/arm collision.
                if any(not any('_'+f+'_' in b for f in ('thumb','index','middle') for b in (v['a'],v['b'])) for v in bad):
                    break
            if baseline is None:
                reason=bad[0]['a']+'|'+bad[0]['b'] if bad else 'grip_contact_not_detected'
                rejects[reason]=rejects.get(reason,0)+1
                continue
            valid_grips+=1
            covered=[];states=[]
            for gi,g in enumerate(goals):
                if not coverage[pi,gi]:continue
                rr=results[g['name']];good=[]
                for j,state in enumerate(g['states']):
                    if not rr['ok'][localrow,j]:good.append(False);continue
                    q=baseline.copy();q.update(zip(fingers[g['finger']].chain.active,rr['q'][localrow,j]))
                    scene.set_pad(fx['pose'],state['gamepad_q']);scene.set_hand(side,palm,q)
                    clear,bad,grips,minimum=scene.check(side,g['part'],g['finger'])
                    accepted=clear and grips=={'ring','pinky'} and scene.last_active_contact
                    good.append(accepted)
                    if accepted:states.append(dict(control=g['name'],state=j,hand_q=q,gamepad_q=state['gamepad_q'],min_contact_m=minimum,
                                                  point_error_m=float(rr['error'][localrow,j]),normal_cos=float(rr['dot'][localrow,j]),active_contact_distance_m=scene.last_active_distance))
                passed=any(good) if g.get('alternatives') else all(good)
                if passed:covered.append(g['name']);counts[g['name']]+=1
            key=','.join(covered) if covered else '(grip only)';combos[key]=combos.get(key,0)+1
            witness=dict(palm_index=pi,palm_in_pad=list(matrix_to_pose(palms[pi])),palm_world=list(matrix_to_pose(palm)),
                         arm_q=aq[row].tolist(),arm_position_error_m=float(ape[row]),arm_rotation_error_rad=float(are[row]),
                         resting_hand_q=baseline,controls=covered,states=states)
            examples.append(witness)
        examples.sort(key=lambda x:len(x['controls']),reverse=True)
        witnesses[fx['name']]=examples
        full=sum(len(e['controls'])==len(goals) for e in examples)
        fr=dict(**fx,sampled_palms=len(palms),grip_kinematic=int(len(grip_ids)),arm_ik=int(aok.sum()),
                collision_free_grips=valid_grips,full_hand_feasible=full,per_control=counts,
                control_combinations=combos,best_controls=examples[0]['controls'] if examples else [],rejections=rejects)
        fixture_results.append(fr)
        print(json.dumps(dict(stage='fixture',**fr)),flush=True)
    out=dict(side=side,phase='K0 static necessary feasibility; no dynamics or force validation',
             parameters=dict(fk_samples_per_finger=args.samples,palm_samples=args.palms,shell_samples=args.shell_samples,
                             grip_surface_samples=len(points),seed=args.seed+(side=='right'),normal_cone_deg=30,
                             point_tolerance_m=POINT_TOL,grip_penetration_m=.001,active_penetration_numerical_m=.0001,
                             arm_position_tolerance_m=.0005,arm_rotation_tolerance_rad=.005,
                             arm_seeds=args.arm_seeds,arm_iterations=args.arm_iterations,arm_seed=args.seed+1000+(side=='right'),rest_postures=12,
                             grip_patch='signed x >25mm, y<5mm, z<-5mm, outward nz<-.3; shell only',
                             grip_fixed=True,lookahead=0),
             goals=goals,grip_kinematic=int(len(grip_ids)),kinematic_per_control={g['name']:int(coverage[:,i].sum()) for i,g in enumerate(goals)},
             kinematic_full=int(coverage.all(axis=1).sum()),fixtures=fixture_results,
             elapsed_seconds=time.monotonic()-start,inertia_corrections=scene.corrections,
             limitations=['finite sampling does not prove impossibility','static contact geometry does not prove force or continuous path feasibility',
                          'own-hand self-collision and connected own hand/arm mounting seam excluded',
                          'single-side runs use opposite arm parked at spec.home_q; paired verification required',
                          'support fixture geometry is not yet modeled; central open-grip support is a design proposal',
                          'right mount symmetric assumed; E2 unknown'])
    (args.output/'result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    (args.output/'witnesses.json').write_text(json.dumps(witnesses,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(stage='done',seconds=out['elapsed_seconds'])),flush=True)


if __name__=='__main__':main()
