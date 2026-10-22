#!/usr/bin/env python3
"""Expanded K1-b common-pose search, checkpointed native replay per start.

Adds a height/tilt lattice and perturbations of an actual common native grip.
All proxy losses remain search guides; Native decides acceptance and controls.
"""
from __future__ import annotations
import argparse,json,os,time
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from k1b_joint import Joint,seeds,load_candidates,SIDES
from k1b_native import Native
from k1_height_common import write_json
from k1b_search import cpu_seconds

_J=None
_N=None
_OUT=None


def initialize(task,cache,out):
    global _J,_N,_OUT
    _J=Joint(task,cache,30.);_N=None;_OUT=Path(out)


def solve(item):
    global _N
    index,z,mode,nfev,repair=item;j=_J;t=time.monotonic();c=cpu_seconds()
    scale=np.r_[.08,.08,.08,.3,np.ones(len(z)-4)*.5]
    r=least_squares(lambda v:j.residual(v,mode,z),z,bounds=(j.lower,j.upper),
        x_scale=scale,max_nfev=nfev,ftol=1e-7,xtol=1e-7,gtol=1e-7)
    z=r.x;summary=j.summaries(z)
    row=dict(seed_index=index,mode=mode,z=z.tolist(),cost=float(r.cost),nfev=r.nfev,sides=summary)
    near=all(s['natural']['deviation']<1.05 and max(s['grip_point_error_m'])<.001 for s in summary.values())
    if near:
        if _N is None:_N={s:Native(j.p[s],_OUT/'models'/str(os.getpid())/s) for s in SIDES}
        xs=j.split(z);checks={s:_N[s].evaluate(xs[s]) for s in SIDES};row['initial_native']=checks
        if not all(s['passed'] for s in checks.values()) and repair:
            # Native collision residuals do not change the strict pass predicates.
            q=least_squares(lambda v:np.r_[j.residual(v,mode,z),*[_N[s].penalty(j.split(v)[s]) for s in SIDES]],
                z,bounds=(j.lower,j.upper),x_scale=scale,diff_step=1e-4,max_nfev=repair,
                ftol=1e-8,xtol=1e-8,gtol=1e-8)
            z=q.x;xs=j.split(z);checks={s:_N[s].evaluate(xs[s]) for s in SIDES}
            row['repair_nfev']=q.nfev
        row['z']=z.tolist();row['native']=checks
        if all(s['passed'] for s in checks.values()):
            entry=dict(z=z.tolist(),seed_index=index,sides={s:dict(x=xs[s].tolist(),check=checks[s]) for s in SIDES})
            for s in SIDES:
                side=entry['sides'][s]
                side['controls'],side['states'],side['control_diagnostics']=_N[s].controls(xs[s])
            row['accepted']=entry
    row.update(cpu_seconds=cpu_seconds()-c,wall_seconds=time.monotonic()-t)
    return row


def seed_bank(j,args):
    rng=np.random.default_rng(8307);left,right=load_candidates(args.left),load_candidates(args.right)
    for side,entries in [('left',left),('right',right)]:
        path=getattr(args,'rolled_'+side)
        if path:entries.extend(json.loads(path.read_text())['seeds'])
    base=seeds(j,left,right,args.starts)
    common=[]
    if args.common:
        for e in json.loads(args.common.read_text()).get('common_successes',[]):
            common.append(np.r_[e['sides']['left']['x'],e['sides']['right']['x'][4:]])
    output=[];meta=[]
    # Half paired/mirrored natural-human seeds, half explicit 60..220mm / 0..30deg coverage.
    for i in range(args.starts):
        if i<len(base)//2:
            z=base[i].copy();kind='paired_or_mirrored_human'
        else:
            k=i-len(base)//2;z=(common[k%len(common)] if common and k%2==0 else base[k%len(base)]).copy()
            height=.060+.020*(k%9);theta=np.deg2rad(5*((k//9)%7))
            z[2:4]=height,theta
            # Calibrated optical-axis intersection, adjusted for geometric centroid.
            p=j.p['left'];pad=p.unpack(np.r_[z[:4],z[4:j.n]])[0]
            centre=pad[:3,:3]@p.pad_centre+pad[:3,3]
            axis=p.cf[:3,2];hit=p.cf[:3,3]+axis*((centre[2]-p.cf[2,3])/axis[2])
            z[:2]=hit[:2]-(pad[:3,:3]@p.pad_centre)[:2]+rng.choice([-.06,-.04,-.02,0,.02,.04,.06],2)
            kind='height_tilt_lattice_common' if common and k%2==0 else 'height_tilt_lattice_pair'
        output.append(np.clip(z,j.lower+1e-8,j.upper-1e-8));meta.append(kind)
    return output,meta


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    for name in ('left','right','rolled-left','rolled-right','common'):p.add_argument('--'+name,type=Path,required=name in ('left','right'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--starts',type=int,default=1024)
    p.add_argument('--workers',type=int,default=46);p.add_argument('--nfev',type=int,default=240)
    p.add_argument('--repair-nfev',type=int,default=40)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True);start=time.monotonic();j=Joint(args.task,priority=30.)
    zs,kinds=seed_bank(j,args);checkpoints=args.output/'checkpoints';checkpoints.mkdir(exist_ok=True)
    write_json(args.output/'seed-bank.json',dict(z=[z.tolist() for z in zs],kind=kinds,seed=8307,
        height_lattice_mm=list(range(60,221,20)),tilt_lattice_deg=list(range(0,31,5))))
    rows=[json.loads(f.read_text()) for f in sorted(checkpoints.glob('*.json'))];done={r['seed_index'] for r in rows}
    with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize,
            initargs=(args.task,{s:j.p[s].surface_cache for s in SIDES},args.output)) as pool:
        futures=[pool.submit(solve,(i,z,['all','thumb','shoulder','none'][i%4],args.nfev,args.repair_nfev))
            for i,z in enumerate(zs) if i not in done]
        for future in as_completed(futures):
            row=future.result();rows.append(row);write_json(checkpoints/f"{row['seed_index']:04d}.json",row)
            if len(rows)%4==0:
                accepted=[r['accepted'] for r in rows if 'accepted' in r]
                print(json.dumps(dict(stage='expanded',done=len(rows),accepted=len(accepted),
                    best_coverage=max((sum(len(e['sides'][s]['controls']) for s in SIDES) for e in accepted),default=0))),flush=True)
                write_json(args.output/'progress.json',dict(done=len(rows),accepted=accepted))
    accepted=[r['accepted'] for r in rows if 'accepted' in r]
    accepted.sort(key=lambda e:(-sum(len(e['sides'][s]['controls']) for s in SIDES),e['z'][2],
        max(e['sides'][s]['check']['natural']['wrist_deviation_deg'] for s in SIDES),
        sum(j.p[s].summary(np.array(e['sides'][s]['x']))['centre_error_px'] for s in SIDES),
        -min(e['sides'][s]['check']['camera']['margin_px'] for s in SIDES)))
    write_json(args.output/'result.json',dict(accepted=accepted,choice=accepted[0] if accepted else None,
        starts=len(zs),workers=args.workers,nfev=args.nfev,repair_nfev=args.repair_nfev,
        constraint_priority=30.,elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds()))


if __name__=='__main__':main()
