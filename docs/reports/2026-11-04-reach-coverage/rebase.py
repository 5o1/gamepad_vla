"""Reuse accepted realistic poses as final common-hold candidates, CPU only."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import solve as z
from refine import neutralize
from k1_height_common import write_json


def run(a):
    started=time.time();z.DEADLINE=min(a.deadline,started+1100)
    folder=a.run/'solve'/a.point/a.side
    while True:
        z.tick();r=json.loads((folder/'result.json').read_text())
        if 'stage3_completed' in r:break
        time.sleep(1)
    write_json(folder/'stage3-result.json',r)
    p=z.ZProblem(a.source/'final'/f'rank-{a.rank}'/'task.json',a.side)
    native=z.ZNative(p,folder/'rebase-model')
    candidates=json.loads((folder/'candidates-progress.json').read_text())
    def rank(c):return (len(c['controls']),len(c['check']['grips']),-c['check']['natural']['wrist_deviation_deg'])
    best=r['strict'];complete=True
    states=[s for s in r['realistic']['states'] if s['control'] in r['realistic']['controls'] and
        s['control'] not in r['strict']['controls']]
    states.sort(key=lambda s:s['motion']['centroid_displacement_mm'])
    seeds=[]
    for s in states:
        if not any(s['control']==v['control'] for v in seeds):seeds.append(s)
    try:
        for s in seeds[:3]:
            z.tick();x=np.array(r['realistic']['x']);x[4:11]=s['x'][4:11]
            x=neutralize(p,native,x);check=native.evaluate(x)
            row=dict(x=x.tolist(),check=check,label='realistic-witness-'+s['control'],controls=[])
            if check['passed']:
                row.update(z.controls(p,native,x,refine=True))
                goal=next(g for g in p.goals if g['name']==s['control'])
                # Reuse a known operation witness when this control has one
                # required state (or alternative trigger patches). It must have
                # the identical arm to the new hold, and is natively replayed.
                if s['control'] not in row['controls'] and (len(goal['states'])==1 or goal.get('alternatives')):
                    state=goal['states'][s['state']]
                    v=np.array(s['x']);v[4:11]=x[4:11]
                    c=native.evaluate(v,gamepad_q=state['gamepad_q'],goal=goal,state=state,solution=dict(mode=s['mode']))
                    if c['passed']:
                        row['controls'].append(s['control'])
                        row['states'].append(dict(s,x=v.tolist(),check=c,gamepad_q=state['gamepad_q'],
                            motion=dict(centroid_displacement_mm=0.,root_displacement_mm=0.,rotation_deg=0.)))
                        row['diagnostics'][s['control']]=dict(passed=True,replayed_realistic_witness=True)
            candidates.append(row);write_json(folder/'candidates-progress.json',candidates)
            if check['passed'] and rank(row)>rank(best):best=row
            print(json.dumps(dict(event='rebase_candidate',point=a.point,side=a.side,
                label=row['label'],controls=row['controls'],reasons=check['reasons'])),flush=True)
    except TimeoutError:complete=False
    realistic=max([r['realistic'],dict(best,base_label=best['label'])],key=rank)
    r.update(strict=best,realistic=realistic,stage4_completed=complete,stage4_elapsed_seconds=time.time()-started,
        candidates=len(candidates),valid_candidates=sum(c['check']['passed'] for c in candidates))
    write_json(folder/'result.json',r)
    print(json.dumps(dict(event='rebase_done',point=a.point,side=a.side,completed=complete,
        strict=best['controls'],realistic=realistic['controls'],elapsed=time.time()-started)),flush=True)


def all_jobs(a):
    root=a.run;code=a.source/'code';workspace=Path(os.environ['GAMEPAD_VLA_WORKSPACE'])
    env=dict(os.environ,GAMEPAD_VLA_WORKSPACE=str(workspace),CUDA_VISIBLE_DEVICES='',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        PYTHONPATH=f'{workspace}/runs/natural-grip-s-20261023/python_overlay:{code}/src:{code}/scripts')
    points=['d050-h220-t+10','d075-h190-t+10','d025-h250-t+10','d050-h190-t+10']
    deadline=json.loads((root/'budget.json').read_text())['compute_started_epoch']+3000
    write_json(root/'rebase-budget.json',dict(started_epoch=time.time(),deadline_epoch=deadline,per_point_side_seconds=1100))
    jobs=[]
    for rank,point in enumerate(points,1):
        for side in ('left','right'):
            i=len(jobs);log=(root/f'rebase-{point}-{side}.log').open('w')
            cmd=['timeout','1200','taskset','-c',f'{i*4}-{i*4+3}',str(workspace/'envs/fixed-gamepad-mujoco-py311/bin/python'),
                '-u',str(root/'rebase.py'),'--source',str(a.source),'--run',str(root),'--point',point,
                '--rank',str(rank),'--side',side,'--deadline',str(deadline)]
            proc=subprocess.Popen(cmd,cwd=code,env=env,stdout=log,stderr=subprocess.STDOUT);jobs.append((point,side,proc,log,cmd))
    write_json(root/'rebase-pids.json',[dict(point=p,side=s,pid=c.pid,command=cmd) for p,s,c,l,cmd in jobs])
    for p,s,c,l,cmd in jobs:c.wait();l.close()
    write_json(root/'rebase-exit.json',[dict(point=p,side=s,returncode=c.returncode) for p,s,c,l,cmd in jobs])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('run','source'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--all',action='store_true');p.add_argument('--point');p.add_argument('--side');p.add_argument('--rank',type=int)
    p.add_argument('--deadline',type=float)
    a=p.parse_args()
    if a.all:all_jobs(a)
    else:run(a)
