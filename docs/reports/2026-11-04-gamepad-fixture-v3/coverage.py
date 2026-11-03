#!/usr/bin/env python3
"""Bounded Task Z native replay at V3, CPU only; no broad new pose search.

Retarget saved accepted palm frames by 7-DoF arm IK, retain the original contact
modes and finger states, then rerun Z's exact native predicates. This establishes
retained-coverage lower bounds, not a new global reachability optimum.
"""
from __future__ import annotations
import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def run(a):
    started=time.time();deadline=started+a.seconds;root=Path(os.environ['GAMEPAD_VLA_WORKSPACE']);old=root/'runs/gamepad-placement-x-20261103';zr=root/'runs/reach-coverage-z-20261104'
    sys.path.insert(0,str(zr));import solve as z
    import natural_grip_fixed as fixed
    from gamepad_vla.workstation.spec import pose_to_matrix
    from k1_height_common import write_json
    raw=json.loads(a.spec.read_text());proxies=[f for f in raw['fixtures'] if f['name'].startswith('v3_proxy_column')]
    fixed.POST=max(proxies,key=lambda f:np.prod(f['half_extents']))['name']
    task=json.loads((old/'final/rank-1/task.json').read_text());task['workcell']=str(a.spec.resolve());a.output.mkdir(parents=True,exist_ok=True);write_json(a.output/'task.json',task)
    p=z.ZProblem(a.output/'task.json',a.side);native=z.ZNative(p,a.output/'model')
    oldW=pose_to_matrix(json.loads((old/'final/rank-1/spec.json').read_text())['gamepad']['root_pose']);delta=p.fixed_pad@np.linalg.inv(oldW)
    source=json.loads((zr/'solve/d050-h220-t+10'/a.side/'result.json').read_text());results={};retarget_cache={}
    def tick():
        if time.time()>deadline:raise TimeoutError('V3 25-minute shared replay ceiling')
    def retarget(x):
        tick();x=np.asarray(x).copy();q=x[4:11];key=tuple(q)
        if key not in retarget_cache:
            desired=delta@p.feature.frames(q)['palm'][0]
            def residual(arm):
                tick();actual=p.feature.frames(arm)['palm'][0]
                return np.r_[(actual[:3,3]-desired[:3,3])*1000,Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).as_rotvec()*100,(arm-q)*.00001]
            fit=least_squares(residual,np.clip(q,p.lower[4:11]+1e-9,p.upper[4:11]-1e-9),bounds=(p.lower[4:11],p.upper[4:11]),max_nfev=80,ftol=1e-11,xtol=1e-11,gtol=1e-11)
            actual=p.feature.frames(fit.x)['palm'][0];retarget_cache[key]=(fit.x,dict(position_error_mm=float(np.linalg.norm(actual[:3,3]-desired[:3,3])*1000),rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).magnitude()))))
        x[:4]=p.fixed4;x[4:11]=retarget_cache[key][0];return x,retarget_cache[key][1]
    for mode in ('strict','realistic'):
        original=source[mode];anchor,info=retarget(original['x']);hold=native.evaluate(anchor);rows=[];groups={}
        for saved in original['states']:
            tick()
            if saved['control'] not in original['controls']:continue
            g=next(g for g in p.goals if g['name']==saved['control']);state=g['states'][saved['state']]
            x,fitinfo=retarget(saved['x'])
            if mode=='strict':x[4:11]=anchor[4:11]
            check=native.evaluate(x,gamepad_q=state['gamepad_q'],goal=g,state=state,solution=dict(mode=saved['mode']))
            motion=z.motion(p,x,anchor)
            if mode=='realistic' and (motion['centroid_displacement_mm']>20.+1e-6 or motion['rotation_deg']>15.+1e-6):check['passed']=False;check['reasons'].append('palm_adjustment_limit')
            row=dict(control=saved['control'],state=saved['state'],x=x.tolist(),retarget=fitinfo,motion=motion,passed=check['passed'],reasons=check['reasons'],check=check)
            rows.append(row);groups.setdefault(saved['control'],[]).append(row)
            write_json(a.output/'progress.json',dict(side=a.side,mode=mode,states=rows,elapsed_seconds=time.time()-started))
        controls=[]
        for name,items in groups.items():
            g=next(g for g in p.goals if g['name']==name)
            required=1 if g.get('alternatives') else len(g['states'])
            success={r['state'] for r in items if r['passed']}
            if len(success)>=required:controls.append(name)
        if not hold['passed']:controls=[]
        results[mode]=dict(source_controls=original['controls'],retained_controls=controls,count=len(controls),denominator=8,anchor=anchor.tolist(),anchor_retarget=info,hold_check=hold,states=rows,
            scope='Retarget and exact native replay of Z accepted states only; unsearched controls remain unproven, not unreachable.')
        write_json(a.output/'result.json',dict(side=a.side,results=results,completed=False,elapsed_seconds=time.time()-started,gpu=False))
    write_json(a.output/'result.json',dict(side=a.side,results=results,completed=True,elapsed_seconds=time.time()-started,gpu=False,
        method='Z predicates and original modes, natural bounds, strict common arm or realistic palm<=20mm/SO3<=15deg; V3 contact boxes and all station collision fixtures',
        source='reach-coverage-z-20261104/d050-h220-t+10',spec='configs/workcell/placement_trials/gamepad_fixture_v3.json'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--side',choices=('left','right'),required=True);p.add_argument('--seconds',type=float,default=1500);a=p.parse_args();run(a)
