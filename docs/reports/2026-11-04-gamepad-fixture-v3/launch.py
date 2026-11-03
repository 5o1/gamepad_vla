"""Isolated V3 CPU supervisor. Run on the configured compute workspace."""
import argparse,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);a=p.parse_args();w=a.workspace
run=w/'runs/fixture-v3-20261104';code=run/'code';report=code/'docs/reports/2026-11-04-gamepad-fixture-v3';spec=code/'configs/workcell/placement_trials/gamepad_fixture_v3.json'
started=time.time();deadline=started+1720
resources=json.loads((w/'runs/reach-coverage-z-20261104/render/config.json').read_text())['resources']
env=dict(os.environ,GAMEPAD_VLA_WORKSPACE=str(w),CUDA_VISIBLE_DEVICES='',GAMEPAD_VLA_RENDER_DEVICE='',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',VK_ICD_FILENAMES='/usr/share/vulkan/icd.d/lvp_icd.x86_64.json',XBOX_GAMEPAD_PHYSICS_RESOURCES=resources)
oldcode=w/'runs/gamepad-placement-x-20261103/code'
env['PYTHONPATH']=f'{w}/runs/natural-grip-s-20261023/python_overlay:{code}/src:{oldcode}/scripts'
jobs=[]
commands=[('left',[str(w/'envs/fixed-gamepad-mujoco-py311/bin/python'),str(report/'coverage.py'),'--spec',str(spec),'--output',str(run/'coverage/left'),'--side','left','--seconds','1500']),
 ('right',[str(w/'envs/fixed-gamepad-mujoco-py311/bin/python'),str(report/'coverage.py'),'--spec',str(spec),'--output',str(run/'coverage/right'),'--side','right','--seconds','1500']),
 ('render',[str(w/'envs/sapien-3.0.3-py310/bin/python'),str(report/'render_evidence.py'),'capture','--spec',str(spec),'--run',str(run/'render'),'--projection',str(report/'projection.json')])]
for j,(name,cmd) in enumerate(commands):
 log=(run/(name+'.log')).open('w');command=['timeout','1700','nice','-n','10','taskset','-c',f'{j*4}-{j*4+3}',*cmd]
 proc=subprocess.Popen(command,cwd=code,env=env,stdout=log,stderr=subprocess.STDOUT);jobs.append((name,proc,log,command))
(run/'budget.json').write_text(json.dumps(dict(started_epoch=started,wall_ceiling_seconds=1720,worker_seconds=1500,cpu=True,gpu=False,commands=[dict(name=n,pid=p.pid,command=c) for n,p,l,c in jobs]),indent=2)+'\n')
for name,proc,log,cmd in jobs:proc.wait();log.close()
(run/'exit.json').write_text(json.dumps(dict(elapsed_seconds=time.time()-started,jobs=[dict(name=n,returncode=p.returncode) for n,p,l,c in jobs]),indent=2)+'\n')
