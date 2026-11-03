"""CPU process supervisor; eight independently bounded point/side jobs."""
import json
import os
from pathlib import Path
import subprocess
import time

root=Path(os.environ['GAMEPAD_VLA_WORKSPACE'],'runs/reach-coverage-z-20261104')
source=root.parent/'gamepad-placement-x-20261103'
workspace=Path(os.environ['GAMEPAD_VLA_WORKSPACE'])
code=source/'code'
env=dict(os.environ, GAMEPAD_VLA_WORKSPACE=str(workspace), CUDA_VISIBLE_DEVICES='',
    OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
    PYTHONPATH=f'{workspace}/runs/natural-grip-s-20261023/python_overlay:{code}/src:{code}/scripts')
points=['d050-h220-t+10','d075-h190-t+10','d025-h250-t+10','d050-h190-t+10']
started=time.time()
budget=dict(compute_started_epoch=1793716029.,supervisor_started_epoch=started,
    solve_deadline_epoch=min(started+2400,1793716029.+2700),
    compute_deadline_epoch=1793716029.+3600,
    per_point_side_seconds=2100,workers=8,threads_per_compile=4,gpu=False)
(root/'budget.json').write_text(json.dumps(budget,indent=2)+'\n')
jobs=[]
for rank,point in enumerate(points,1):
    for side in ('left','right'):
        index=len(jobs);cpus=f'{index*4}-{index*4+3}'
        cmd=['timeout','2250','taskset','-c',cpus,str(workspace/'envs/fixed-gamepad-mujoco-py311/bin/python'),
            '-u',str(root/'solve.py'),'--source',str(source),'--output',str(root/'solve'),
            '--point',point,'--rank',str(rank),'--side',side,'--seconds','2100',
            '--deadline',str(budget['solve_deadline_epoch'])]
        log=(root/f'{point}-{side}.log').open('w')
        proc=subprocess.Popen(cmd,cwd=code,env=env,stdout=log,stderr=subprocess.STDOUT)
        jobs.append((point,side,proc,log,cmd))
(root/'pids.json').write_text(json.dumps([dict(point=p,side=s,pid=pr.pid,command=c) for p,s,pr,l,c in jobs],indent=2)+'\n')
for p,s,proc,log,cmd in jobs:
    proc.wait();log.close()
(root/'exit.json').write_text(json.dumps([dict(point=p,side=s,returncode=pr.returncode) for p,s,pr,l,c in jobs],indent=2)+'\n')
print('All point/side processes finished',time.time()-started,flush=True)
