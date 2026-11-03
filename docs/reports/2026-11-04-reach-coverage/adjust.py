"""Final Z realistic search: centroid limit only, bounded pose multi-starts."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import solve as z
from refine import neutralize, natural_project
from k1b_contact_fit import fit_goal_modes
from k1_height_common import write_json
from gamepad_vla.feasibility import transform


def admissible(p,x,base):
    m=z.motion(p,x,base)
    return m['centroid_displacement_mm']<=20.0000001 and m['rotation_deg']<=15.0000001


def arm_seed(p,base,goal,si,angle):
    base=np.array(base);palm=p.feature.frames(base[4:11])['palm'][0]
    pp=np.linalg.inv(p.fixed_pad)@palm;state=goal['states'][si];finger=p.fingers[goal['finger']]
    target=(np.array(state['point'])-pp[:3,3])@pp[:3,:3]
    normal=-np.array(state['normal'])@pp[:3,:3]
    cone=goal['name'] not in ('LS','RS','LT','RT')
    _,idx=finger.tree.query(np.r_[target,.015*normal]) if cone else finger.position_tree.query(target)
    delta=palm[:3,:3]@(target-finger.xyz[idx])
    norm=np.linalg.norm(delta)
    if norm>0:delta*=min(.018/norm,1.)
    rotation=palm[:3,:3]@Rotation.from_euler('yz',angle,degrees=True).as_matrix()
    centroid=transform(p.feature.palm_point_local,palm)+delta
    targetroot=centroid-rotation@p.feature.palm_point_local
    lo,hi=p.feature.chain.active_limits
    def fun(q):
        z.tick();f=p.feature.frames(q)['palm'][0]
        return np.r_[(f[:3,3]-targetroot)*1000,
            Rotation.from_matrix(rotation@f[:3,:3].T).as_rotvec()*50,
            p.natural_residual(q)*1000,(q-base[4:11])*.00001]
    fit=least_squares(fun,base[4:11],bounds=(lo,hi),max_nfev=45,
        ftol=1e-9,xtol=1e-9,gtol=1e-9)
    x=base.copy();x[4:11]=fit.x
    return natural_project(p,x)


def sample_state(p,native,base,goal,si):
    diagnostics=[];state=goal['states'][si]
    for angle in [(0,0),(0,10),(0,-10),(10,0),(-10,0)]:
        z.tick();x=arm_seed(p,base,goal,si,angle)
        if not admissible(p,x,base):continue
        pp=np.linalg.inv(p.fixed_pad)@p.feature.frames(x[4:11])['palm'][0]
        # A single state keeps the original target and gamepad_q unchanged.
        g=dict(goal,states=[state]);fits=fit_goal_modes(p.fingers[g['finger']],g,pp)
        rejected=Counter()
        for solution in fits['states'][0]['solutions']:
            v=x.copy();v[11+np.arange(len(p.names))[p.slices[g['finger']]]]=solution['q']
            c=native.evaluate(v,gamepad_q=state['gamepad_q'],goal=goal,state=state,solution=solution)
            if c['passed']:
                return dict(control=goal['name'],state=si,x=v.tolist(),mode=solution['mode'],
                    check=c,motion=z.motion(p,v,base),gamepad_q=state['gamepad_q']),diagnostics
            rejected.update(c['reasons'])
        row,attempts=z.state_fit(p,native,x,goal,si,False,nfev=40,starts=3)
        diagnostics.append(dict(seed_rotation_yz_deg=angle,native_rejections=dict(rejected),joint_attempts=attempts))
        if row['check']['passed'] and admissible(p,row['x'],base):
            row.update(control=goal['name'],motion=z.motion(p,row['x'],base));return row,diagnostics
    return None,diagnostics


def improve(p,native,base,initial,checkpoint):
    out={k:initial[k] for k in ('controls','states','diagnostics','radial')}
    out['controls']=list(initial['controls']);out['states']=list(initial['states'])
    out['diagnostics']=dict(initial['diagnostics']);x=np.array(base['x'])
    for goal in p.goals:
        name=goal['name']
        if name in out['controls']:continue
        winners={s['state']:s for s in out['states'] if s['control']==name}
        attempts=[]
        for si in range(len(goal['states'])):
            if si in winners:continue
            winner,diag=sample_state(p,native,x,goal,si)
            attempts.append(dict(state=si,pose_multistarts=diag,passed=winner is not None))
            if winner:
                winners[si]=winner
                if goal.get('alternatives'):break
            elif not goal.get('alternatives') and name in ('LS','RS'):break
        passed=bool(winners) if goal.get('alternatives') else len(winners)==len(goal['states'])
        out['states']=[s for s in out['states'] if s['control']!=name]+list(winners.values())
        out['diagnostics'][name]=dict(passed=passed,centroid_pose_multistarts=attempts,
            previous_diagnostics=out['diagnostics'].get(name))
        if passed:out['controls'].append(name)
        write_json(checkpoint,dict(base_label=base.get('label',base.get('base_label')),
            x=base['x'],check=base['check'],**out))
        print(json.dumps(dict(event='centroid_control',side=p.side,control=name,passed=passed)),flush=True)
    return dict(base_label=base.get('label',base.get('base_label')),x=base['x'],check=base['check'],**out)


def run(a):
    started=time.time();z.DEADLINE=min(a.deadline,started+a.seconds)
    folder=a.run/'solve'/a.point/a.side
    while True:
        z.tick()
        path=folder/'result.json'
        if path.exists():
            r=json.loads(path.read_text())
            if 'stage2_completed' in r:break
        time.sleep(1)
    write_json(folder/'stage2-result.json',r)
    p=z.ZProblem(a.source/'final'/f'rank-{a.rank}'/'task.json',a.side)
    native=z.ZNative(p,folder/'centroid-model')
    candidates=json.loads((folder/'candidates-progress.json').read_text())
    valid=[c for c in candidates if c['check']['passed']]
    def rank(c):return (len(c['controls']),len(c['check']['grips']),-c['check']['natural']['wrist_deviation_deg'])
    bases=[dict(r['realistic'],label=r['realistic'].get('base_label',r['realistic'].get('label')))]
    for c in sorted(valid,key=rank,reverse=True):
        if all(z.motion(p,c['x'],b['x'])['centroid_displacement_mm']>3 or
            z.motion(p,c['x'],b['x'])['rotation_deg']>3 for b in bases):bases.append(c)
        if len(bases)>=3:break
    results=[r['realistic']];complete=True
    try:
        # Search existing diverse shared holds first. Every accepted state stays
        # within 20 mm of its own hold centroid and 15 deg of its orientation.
        for index,base in enumerate(bases):
            if base['x']==r['realistic']['x']:initial=r['realistic']
            else:initial=z.controls(p,native,np.array(base['x']),flexible=True,strict=base)
            result=improve(p,native,base,initial,folder/f'centroid-base-{index}.json');results.append(result)
            write_json(folder/'centroid-progress.json',results)
        # One complementary-hold midpoint tests the realistic base objective.
        pair=max(((u,v) for u in valid for v in valid),key=lambda pair:len(set(pair[0]['controls'])|set(pair[1]['controls'])))
        if len(set(pair[0]['controls'])|set(pair[1]['controls']))>len(r['strict']['controls']):
            mid=(np.array(pair[0]['x'])+pair[1]['x'])/2
            mid=neutralize(p,native,mid);check=native.evaluate(mid)
            if check['passed']:
                strict=z.controls(p,native,mid,refine=False)
                base=dict(x=mid.tolist(),check=check,label='complementary-hold-midpoint',**strict)
                candidates.append(base);write_json(folder/'candidates-progress.json',candidates)
                if rank(base)>rank(r['strict']):r['strict']=base
                initial=z.controls(p,native,mid,flexible=True,strict=base)
                result=improve(p,native,base,initial,folder/'centroid-base-midpoint.json');results.append(result)
                write_json(folder/'centroid-progress.json',results)
    except TimeoutError:
        complete=False
        if (folder/'centroid-progress.json').exists():results=json.loads((folder/'centroid-progress.json').read_text())
        results += [json.loads(f.read_text()) for f in folder.glob('centroid-base-*.json')]
    results.append(dict(r['strict'],base_label=r['strict']['label']))
    r.update(realistic=max(results,key=rank),stage3_completed=complete,stage3_elapsed_seconds=time.time()-started,
        candidates=len(candidates),valid_candidates=sum(c['check']['passed'] for c in candidates),
        realistic_displacement_definition='G20 hand_base_link collision mesh surface-area centroid; root displacement diagnostic only')
    write_json(folder/'result.json',r)
    print(json.dumps(dict(event='centroid_done',point=a.point,side=a.side,completed=complete,
        strict=r['strict']['controls'],realistic=r['realistic']['controls'],elapsed=time.time()-started)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','source'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--point',required=True);p.add_argument('--side',required=True);p.add_argument('--rank',type=int,required=True)
    p.add_argument('--seconds',type=float,default=1000);p.add_argument('--deadline',type=float,required=True)
    run(p.parse_args())
