"""CPU continuation, bounded by the original total compute window."""
import json
import os
from pathlib import Path
import subprocess
import time

root=Path(os.environ['GAMEPAD_VLA_WORKSPACE'],'runs/reach-coverage-z-20261104')
source=root.parent/'gamepad-placement-x-20261103';workspace=Path(os.environ['GAMEPAD_VLA_WORKSPACE']);code=source/'code'
env=dict(os.environ,GAMEPAD_VLA_WORKSPACE=str(workspace),CUDA_VISIBLE_DEVICES='',
    OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
    PYTHONPATH=f'{workspace}/runs/natural-grip-s-20261023/python_overlay:{code}/src:{code}/scripts')
points=['d050-h220-t+10','d075-h190-t+10','d025-h250-t+10','d050-h190-t+10']
deadline=json.loads((root/'budget.json').read_text())['compute_started_epoch']+2700
write=dict(started_epoch=time.time(),deadline_epoch=deadline,per_point_side_seconds=1400)
(root/'refine-budget.json').write_text(json.dumps(write,indent=2)+'\n')
jobs=[]
for rank,point in enumerate(points,1):
    for side in ('left','right'):
        i=len(jobs);log=(root/f'refine-{point}-{side}.log').open('w')
        cmd=['timeout','1500','taskset','-c',f'{i*4}-{i*4+3}',str(workspace/'envs/fixed-gamepad-mujoco-py311/bin/python'),
            '-u',str(root/'refine.py'),'--source',str(source),'--run',str(root),'--point',point,
            '--rank',str(rank),'--side',side,'--seconds','1400','--deadline',str(deadline)]
        p=subprocess.Popen(cmd,cwd=code,env=env,stdout=log,stderr=subprocess.STDOUT);jobs.append((point,side,p,log,cmd))
(root/'refine-pids.json').write_text(json.dumps([dict(point=p,side=s,pid=c.pid,command=cmd) for p,s,c,l,cmd in jobs],indent=2)+'\n')
for point,side,p,log,cmd in jobs:p.wait();log.close()
(root/'refine-exit.json').write_text(json.dumps([dict(point=point,side=side,returncode=p.returncode) for point,side,p,log,cmd in jobs],indent=2)+'\n')
