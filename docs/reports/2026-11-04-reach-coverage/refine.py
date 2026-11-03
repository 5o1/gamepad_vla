"""Second Z pass: explicit shared-arm/multiple-finger-state optimization."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares, minimize
import solve as z
from gamepad_vla.feasibility import transform
from k1_height_common import write_json


def natural_project(p,x):
    """Correct penalty-boundary numerical overshoot with hard SLSQP bounds."""
    x=np.array(x);anchor=x[4:11].copy()
    if p.feature.measure(anchor,p.world_centre)['natural'][0]:return x
    lo,hi=p.feature.chain.active_limits
    def inequalities(q):
        m=p.feature.measure(q,p.world_centre)
        u=m['upper_tilt_deg'][0];f=m['forearm_pitch_deg'][0];h=m['forearm_heading_deg'][0]
        return np.array([u-10,35-u,f+10,25-f,30-h,
            (m['elbow']-m['shoulder'])[0,0]*100])-1e-7
    fit=minimize(lambda q:float(np.sum((q-anchor)**2)),anchor,method='SLSQP',bounds=list(zip(lo,hi)),
        constraints=[dict(type='ineq',fun=inequalities)],options=dict(maxiter=60,ftol=1e-12))
    x[4:11]=fit.x
    return x


def neutralize(p,native,x):
    """Hold arm fixed and find an unpenetrating neutral finger configuration."""
    x=natural_project(p,x)
    if native.evaluate(x)['passed']:return x
    anchor=x.copy()
    def fun(h):
        z.tick();v=anchor.copy();v[11:]=h
        return np.r_[native.collision_vector(v)*6000,(h-anchor[11:])*.00001]
    fit=least_squares(fun,np.clip(x[11:],p.lo+1e-8,p.hi-1e-8),bounds=(p.lo,p.hi),
        diff_step=1e-4,max_nfev=65,x_scale=.4,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    x[11:]=fit.x;return x


def joint_common(p,native,base,goals,nfev=70):
    x=np.array(base);pp=np.linalg.inv(p.fixed_pad)@p.feature.frames(x[4:11])['palm'][0]
    seeds=[];records=[];off=len(x)-4
    for goal in goals:
        finger=p.fingers[goal['finger']];count=len(finger.q)
        indices=range(len(goal['states'])) if not goal.get('alternatives') else [0]
        for si in indices:
            state=goal['states'][si]
            target=(np.array(state['point'])-pp[:3,3])@pp[:3,:3]
            normal=-np.array(state['normal'])@pp[:3,:3]
            cone=goal['name'] not in ('LS','RS','LT','RT')
            _,nearest=(finger.tree.query(np.r_[target,.015*normal]) if cone else finger.position_tree.query(target))
            mode=int(nearest)//count;q=finger.q[int(nearest)%count]
            seeds.extend(q);records.append((goal,state,finger,mode,slice(off,off+len(q)),cone))
            off+=len(q)
    lo=np.r_[p.lower[4:],*[f.lower for g,s,f,m,i,c in records]]
    hi=np.r_[p.upper[4:],*[f.upper for g,s,f,m,i,c in records]]
    seed=np.r_[x[4:],seeds]
    def residual(v):
        z.tick();b=np.r_[p.fixed4,v[:len(x)-4]];palm=p.feature.frames(b[4:11])['palm'][0]
        pp=np.linalg.inv(p.fixed_pad)@palm;errors=[]
        for goal,state,finger,mode,section,cone in records:
            fr=pp@finger.chain.forward(v[section][None])[0]
            point=transform(finger.points[mode],fr);n=fr[:3,:3]@finger.normals[mode]
            weight=1/np.sqrt(1 if goal.get('alternatives') else len(goal['states']))
            errors.extend((point-(np.array(state['point'])-.000045*np.array(state['normal'])))*1000*weight)
            errors.append(max(0.,np.cos(np.deg2rad(29.8))+n@np.array(state['normal']))*15*weight if cone else 0.)
        return np.r_[errors,p.natural_residual(b[4:11])*1000,
            native.collision_vector(b)*6000,(v-seed)*.00001,
            z.soft_residual(p,b,x)[:6]*.05,
            p.feature.measure(b[4:11],p.world_centre)['wrist_deviation_deg']*.00000028]
    fit=least_squares(residual,np.clip(seed,lo+1e-8,hi-1e-8),bounds=(lo,hi),x_scale=.35,
        diff_step=1e-4,max_nfev=nfev,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    b=np.r_[p.fixed4,fit.x[:len(x)-4]]
    return neutralize(p,native,b),dict(nfev=fit.nfev,cost=fit.cost,explicit_joint_variables=len(seed),
        required_operation_states=len(records),controls=[g['name'] for g in goals])


def run(a):
    start=time.time();z.DEADLINE=min(start+a.seconds,a.deadline)
    folder=a.run/'solve'/a.point/a.side
    # Wait for the first pass; its independent timeout still applies.
    while not (folder/'result.json').exists():
        z.tick();time.sleep(1)
    previous=json.loads((folder/'result.json').read_text())
    write_json(folder/'stage1-result.json',previous)
    candidates=json.loads((folder/'candidates-progress.json').read_text())
    p=z.ZProblem(a.source/'final'/f'rank-{a.rank}'/'task.json',a.side);p.natural_weight=40
    native=z.ZNative(p,folder/'refine-model')
    def rank(r):return (len(r['controls']),len(r['check']['grips']),-r['check']['natural']['wrist_deviation_deg'])
    best=max([r for r in candidates if r['check']['passed']],key=rank)
    def record(x,label,info):
        nonlocal best
        z.tick();check=native.evaluate(x)
        row=dict(x=np.asarray(x).tolist(),label=label,check=check,optimizer=info,controls=[])
        if check['passed']:row.update(z.controls(p,native,np.array(x),refine=True))
        candidates.append(row);write_json(folder/'candidates-progress.json',candidates)
        print(json.dumps(dict(event='refine_candidate',point=a.point,side=a.side,label=label,
            controls=row['controls'],reasons=check['reasons'],elapsed=time.time()-start)),flush=True)
        if check['passed'] and rank(row)>rank(best):best=row
    complete=True
    try:
        for label,goals in [('all',p.goals),('thumb',[g for g in p.goals if g['finger']=='thumb'])]:
            x,info=joint_common(p,native,best['x'],goals)
            record(x,'explicit-shared-'+label,info)
        for goal in p.goals:
            if goal['name'] in best['controls']:continue
            row,diag=z.state_fit(p,native,np.array(best['x']),goal,0,'free',nfev=75,starts=3)
            if row['check']['passed']:
                x=neutralize(p,native,np.array(row['x']))
                record(x,'free-arm-operation-'+goal['name'],dict(attempts=diag))
        realistic_candidates=[previous['realistic']] if 'realistic' in previous else []
        choices=sorted([r for r in candidates if r['check']['passed']],key=rank,reverse=True)
        for base in choices[:2]:
            result=z.controls(p,native,np.array(base['x']),flexible=True,strict=base)
            realistic_candidates.append(dict(base_label=base['label'],x=base['x'],check=base['check'],**result))
            write_json(folder/'refine-practical-progress.json',realistic_candidates)
    except TimeoutError:
        complete=False
        realistic_candidates=[previous['realistic']] if 'realistic' in previous else []
        if (folder/'refine-practical-progress.json').exists():realistic_candidates=json.loads((folder/'refine-practical-progress.json').read_text())
    result=dict(previous,strict=best,realistic=max(realistic_candidates,key=rank),
        candidates=len(candidates),valid_candidates=sum(r['check']['passed'] for r in candidates),
        stage2_completed=complete,stage2_elapsed_seconds=time.time()-start,
        stage2_deadline_epoch=z.DEADLINE)
    write_json(folder/'result.json',result)
    print(json.dumps(dict(event='refine_done',point=a.point,side=a.side,completed=complete,
        strict=best['controls'],realistic=result['realistic']['controls'],elapsed=time.time()-start)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','source'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--point',required=True);p.add_argument('--side',required=True);p.add_argument('--rank',type=int,required=True)
    p.add_argument('--seconds',type=float,default=1200);p.add_argument('--deadline',type=float,required=True)
    run(p.parse_args())
