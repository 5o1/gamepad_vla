"""One targeted Z-method correction of rejected retargeted states, CPU only."""
import argparse,json,os,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--side',required=True);a=p.parse_args();w=a.workspace;run=w/'runs/fixture-v3-20261104';folder=run/'coverage'/a.side
sys.path.insert(0,str(w/'runs/reach-coverage-z-20261104'));import solve as z
import natural_grip_fixed as fixed
from k1_height_common import write_json
source=json.loads((folder/'result.json').read_text());write_json(folder/'initial-replay.json',source)
spec=json.loads((run/'code/configs/workcell/placement_trials/gamepad_fixture_v3.json').read_text());boxes=[f for f in spec['fixtures'] if f['name'].startswith('v3_proxy_column')];fixed.POST=max(boxes,key=lambda f:np.prod(f['half_extents']))['name']
problem=z.ZProblem(folder/'task.json',a.side);native=z.ZNative(problem,folder/'refine-model')
started=time.time();z.DEADLINE=min(started+600,json.loads((run/'budget.json').read_text())['started_epoch']+1720)
repaired={}
try:
 for mode,r in source['results'].items():
  for row in r['states']:
   z.tick()
   if row['passed']:continue
   g=next(g for g in problem.goals if g['name']==row['control']);state=g['states'][row['state']];x=np.array(row['x']);attempts=[]
   if row['reasons']==['natural_hard']:
    q=x[4:11].copy();desired=problem.feature.frames(q)['palm'][0]
    def residual(arm):
     z.tick();actual=problem.feature.frames(arm)['palm'][0];pitch=problem.feature.measure(arm,problem.world_centre)['forearm_pitch_deg'][0]
     return np.r_[(actual[:3,3]-desired[:3,3])*1000,Rotation.from_matrix(actual[:3,:3]@desired[:3,:3].T).as_rotvec()*100,problem.natural_residual(arm)*80,max(0.,pitch-24.90)*.25,(arm-q)*.000001]
    fit=least_squares(residual,q,bounds=(problem.lower[4:11],problem.upper[4:11]),max_nfev=100,ftol=1e-11,xtol=1e-11,gtol=1e-11);x[4:11]=fit.x
    # Use the original contact mode, reconstructed from Z's saved witness.
    original=json.loads((w/'runs/reach-coverage-z-20261104/solve/d050-h220-t+10'/a.side/'result.json').read_text())
    witness=next(s for s in original[mode]['states'] if s['control']==row['control'] and s['state']==row['state'])
    c=native.evaluate(x,gamepad_q=state['gamepad_q'],goal=g,state=state,solution=dict(mode=witness['mode']))
    candidate=dict(x=x.tolist(),check=c,motion=z.motion(problem,x,np.array(r['anchor'])),nfev=fit.nfev,method='redundant-arm IK with identical desired palm and natural-range interior proposal')
   else:
    key=(row['control'],row['state'],tuple(np.round(np.array(r['anchor'])[4:11],9)))
    if key in repaired:candidate=repaired[key]
    else:
     candidate,attempts=z.state_fit(problem,native,x,g,row['state'],mode=='realistic',nfev=45,starts=2)
     candidate['method']='Z contact-mode fit with original native collision penalty';repaired[key]=candidate
   motion=z.motion(problem,np.asarray(candidate['x']),np.asarray(r['anchor']));candidate['motion']=motion;accepted=candidate['check']['passed'] and (mode=='strict' or (motion['centroid_displacement_mm']<=20+1e-6 and motion['rotation_deg']<=15+1e-6))
   row['targeted_refinement']=dict(method=candidate['method'],nfev=candidate['nfev'],accepted=accepted,attempts=attempts)
   if accepted:row.update(x=candidate['x'],check=candidate['check'],motion=motion,passed=True,reasons=[])
   groups={}
   for s in r['states']:groups.setdefault(s['control'],[]).append(s)
   r['retained_controls']=[name for name,items in groups.items() if len({v['state'] for v in items if v['passed']}) >= (1 if next(g for g in problem.goals if g['name']==name).get('alternatives') else len(next(g for g in problem.goals if g['name']==name)['states']))];r['count']=len(r['retained_controls'])
   write_json(folder/'result.json',source)
except TimeoutError:source['targeted_refinement_budget_exhausted']=True
source['targeted_refinement_seconds']=time.time()-started;source['compute_wall_seconds_from_supervisor']=time.time()-json.loads((run/'budget.json').read_text())['started_epoch'];write_json(folder/'result.json',source)
