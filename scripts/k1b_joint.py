#!/usr/bin/env python3
"""K1-b common-placement two-side block optimization with native acceptance.

One root/tilt is shared, each side has its own actual arm FK and fixed grip.
Control variables are eliminated by per-mode constrained IK and native replay;
FK reach energies guide outer variables without being counted as coverage.
"""
from __future__ import annotations
import argparse,json,time
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from k1_height_common import write_json
from k1b_search import Problem,MIRROR,cpu_seconds
from k1b_native import Native

SIDES=('left','right')
_GLOBAL=None


def reach(p,x,mode):
    if mode=='none':return np.zeros(0)
    pad,palm,_,_,_=p.unpack(x);tohand=np.linalg.inv(palm)@pad;values=[]
    for g in p.goals:
        if mode=='thumb' and g['finger']!='thumb':continue
        if mode=='shoulder' and g['finger']=='thumb':continue
        points=np.array([s['point'] for s in g['states']]);normals=-np.array([s['normal'] for s in g['states']])
        target=points@tohand[:3,:3].T+tohand[:3,3];normal=normals@tohand[:3,:3].T
        d,_=p.fingers[g['finger']].nearest_seed(target,normal,normal_cone=g['name'] not in ('LS','RS','LT','RT'))
        worst=d.min() if g.get('alternatives') else d.max()
        # Saturated per-goal residual approximates an uncovered-goal count.
        # Native contact, original tolerance and all states decide real coverage.
        values.append(.002*np.tanh(max(0,worst-.002)/.008))
    return np.array(values)


class Joint:
    def __init__(self,task,caches=None,priority=1.):
        self.p={s:Problem(task,s,caches[s] if caches else None) for s in SIDES}
        for problem in self.p.values():problem.constraint_priority=priority
        self.lower=np.r_[self.p['left'].lower,self.p['right'].lower[4:]]
        self.upper=np.r_[self.p['left'].upper,self.p['right'].upper[4:]]
        self.n=len(self.p['left'].lower)

    def split(self,z):
        return {'left':z[:self.n],'right':np.r_[z[:4],z[self.n:]]}

    def residual(self,z,mode='none',regularize=None):
        xs=self.split(z)
        return np.concatenate([np.r_[self.p[s].residual(xs[s],self.split(regularize)[s] if regularize is not None else None),reach(self.p[s],xs[s],mode)] for s in SIDES])

    def summaries(self,z):return {s:self.p[s].summary(x) for s,x in self.split(z).items()}


def load_candidates(path):
    data=json.loads(path.read_text())
    return data.get('accepted',[])


def seeds(j,left,right,count):
    allseeds=[]
    # Mirror actual natural-arm human-grip successes, not the rejected K0 palms.
    for source,side in [(left,'left'),(right,'right')]:
        for e in source:
            x=np.array(e['x']);other=x.copy();other[4:11]*=MIRROR
            for heightdelta,tilt_delta in [(0,0),(-.01,0),(.01,0),(0,np.deg2rad(5))]:
                shared=x[:4].copy();shared[1]=.6202;shared[2]+=heightdelta;shared[3]+=tilt_delta
                l=x[4:] if side=='left' else other[4:];r=other[4:] if side=='left' else x[4:]
                z=np.r_[shared,l,r];allseeds.append(np.clip(z,j.lower+1e-8,j.upper-1e-8))
    pairs=sorted([(float(np.linalg.norm((np.array(l['x'])[:4]-np.array(r['x'])[:4])/[.08,.08,.08,.3])),l,r) for l in left for r in right],key=lambda t:t[0])
    for _,l,r in pairs[:count]:
        lx,rx=np.array(l['x']),np.array(r['x']);z=np.r_[(lx[:4]+rx[:4])/2,lx[4:],rx[4:]]
        allseeds.append(np.clip(z,j.lower+1e-8,j.upper-1e-8))
    # Alternate mirrored and nearest-pair seeds instead of taking only one hand.
    rng=np.random.default_rng(7203);order=rng.permutation(len(allseeds))
    return [allseeds[i] for i in order[:count]]


def init(task,caches,priority):
    global _GLOBAL
    _GLOBAL=Joint(task,caches,priority)


def solve(item):
    i,z,mode,nfev=item;j=_GLOBAL;t=time.monotonic();c=cpu_seconds()
    scale=np.r_[.08,.08,.08,.3,np.ones(len(z)-4)*.5]
    result=least_squares(lambda v:j.residual(v,mode,z),z,bounds=(j.lower,j.upper),method='trf',x_scale=scale,
        max_nfev=nfev,ftol=1e-7,xtol=1e-7,gtol=1e-7)
    return dict(seed_index=i,mode=mode,z=result.x.tolist(),cost=float(result.cost),nfev=result.nfev,
                sides=j.summaries(result.x),elapsed_seconds=time.monotonic()-t,cpu_seconds=cpu_seconds()-c)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True)
    p.add_argument('--rolled-left',type=Path);p.add_argument('--rolled-right',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--starts',type=int,default=64)
    p.add_argument('--constraint-priority',type=float,default=1.)
    p.add_argument('--workers',type=int,default=8);p.add_argument('--nfev',type=int,default=160)
    p.add_argument('--polish',type=int,default=12);p.add_argument('--controls-limit',type=int,default=12)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True);start=time.monotonic()
    j=Joint(args.task,priority=args.constraint_priority);left,right=load_candidates(args.left),load_candidates(args.right)
    if args.rolled_left:left+=json.loads(args.rolled_left.read_text())['seeds']
    if args.rolled_right:right+=json.loads(args.rolled_right.read_text())['seeds']
    if not left and not right:
        write_json(args.output/'result.json',dict(stop='no individual native natural grips; user stop condition',accepted=[]));return
    zs=seeds(j,left,right,args.starts)
    cache={s:j.p[s].surface_cache for s in SIDES};checkpoints=args.output/'checkpoints';checkpoints.mkdir(exist_ok=True)
    proposals=[json.loads(f.read_text()) for f in sorted(checkpoints.glob('*.json'))];done={r['seed_index'] for r in proposals}
    modes=['none','all','thumb','shoulder']
    with ProcessPoolExecutor(max_workers=args.workers,initializer=init,initargs=(args.task,cache,args.constraint_priority)) as pool:
        futures=[pool.submit(solve,(i,z,modes[i%4],args.nfev)) for i,z in enumerate(zs) if i not in done]
        for future in as_completed(futures):
            row=future.result();proposals.append(row);write_json(checkpoints/f"{row['seed_index']:04d}.json",row)
            if len(proposals)%4==0:
                near=sum(all(v['natural']['natural'] and max(v['grip_point_error_m'])<=.00025 for v in r['sides'].values()) for r in proposals)
                print(json.dumps(dict(stage='joint_proposal',done=len(proposals),point_natural_both=near)),flush=True)
    write_json(args.output/'proposals.json',proposals)
    native={s:Native(j.p[s],args.output/(s+'-model')) for s in SIDES}
    proposals.sort(key=lambda r:(not all(e['natural']['natural'] for e in r['sides'].values()),max(max(e['grip_point_error_m']) for e in r['sides'].values()),r['cost']))
    accepted=[];attempts=[];polishes=0
    for row in proposals:
        z=np.array(row['z']);xs=j.split(z);checks={s:native[s].evaluate(xs[s]) for s in SIDES}
        initial=checks
        near=all(e['natural']['deviation']<1.25 and max(e['grip_point_error_m'])<.003 for e in row['sides'].values())
        if not all(c['passed'] for c in checks.values()) and near and polishes<args.polish:
            polishes+=1
            result=least_squares(lambda v:np.r_[j.residual(v,row['mode'],z),*[native[s].penalty(j.split(v)[s]) for s in SIDES]],
                z,bounds=(j.lower,j.upper),x_scale=np.r_[.08,.08,.08,.3,np.ones(len(z)-4)*.5],diff_step=1e-4,
                max_nfev=45,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            z=result.x;xs=j.split(z);checks={s:native[s].evaluate(xs[s]) for s in SIDES}
        attempts.append(dict(seed_index=row['seed_index'],mode=row['mode'],initial=initial,final=checks))
        if all(c['passed'] for c in checks.values()):
            e=dict(z=z.tolist(),seed_index=row['seed_index'],sides={s:dict(x=xs[s].tolist(),check=checks[s],controls=[],states=[]) for s in SIDES})
            accepted.append(e)
        print(json.dumps(dict(stage='joint_native',checked=len(attempts),accepted=len(accepted),polishes=polishes)),flush=True)
    # Independent recordings, one shared pad placement. Controls remain at each
    # side's one fixed palm and ring/pinky q; only the assigned finger changes.
    for e in accepted[:args.controls_limit]:
        for side in SIDES:
            entry=e['sides'][side];entry['controls'],entry['states'],entry['control_diagnostics']=native[side].controls(np.array(entry['x']))
    tested=[e for e in accepted if all('control_diagnostics' in e['sides'][s] for s in SIDES)]
    tested.sort(key=lambda e:(-sum(len(e['sides'][s]['controls']) for s in SIDES),e['z'][2],
        max(e['sides'][s]['check']['natural']['wrist_deviation_deg'] for s in SIDES),
        sum(j.p[s].summary(np.array(e['sides'][s]['x']))['centre_error_px'] for s in SIDES),
        -min(e['sides'][s]['check']['camera']['margin_px'] for s in SIDES)))
    out=dict(accepted=accepted,attempts=attempts,choice=tested[0] if tested else None,starts=len(zs),polishes=polishes,
        elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds(),constraint_priority=args.constraint_priority,
        stop='no shared-placement natural native grip; user stop condition' if not accepted else None,
        semantics='one pad pose; two independently recorded fixed palms; all natural/collision/table/camera conditions hard; no unnatural fallback')
    write_json(args.output/'result.json',out)
    if tested:write_json(args.output/'selected.json',tested[0])
    print(json.dumps(dict(stage='joint_done',accepted=len(accepted),controls={s:tested[0]['sides'][s]['controls'] for s in SIDES} if tested else None,cpu_seconds=out['cpu_seconds'])),flush=True)


if __name__=='__main__':main()
