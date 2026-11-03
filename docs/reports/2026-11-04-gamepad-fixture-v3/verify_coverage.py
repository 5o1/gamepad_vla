"""Final native replay of the saved V3 states after final mesh/spec export."""
import argparse,json,os,sys,time,hashlib,importlib.metadata
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--side',required=True);a=p.parse_args();w=a.workspace;run=w/'runs/fixture-v3-20261104';folder=run/'coverage'/a.side
sys.path.insert(0,str(w/'runs/reach-coverage-z-20261104'));import solve as z
import natural_grip_fixed as fixed
from k1_height_common import write_json
source=json.loads((folder/'result.json').read_text());write_json(folder/'before-final-replay.json',source)
path=run/'code/configs/workcell/placement_trials/gamepad_fixture_v3.json';spec=json.loads(path.read_text());boxes=[f for f in spec['fixtures'] if f['name'].startswith('v3_proxy_column')];fixed.POST=max(boxes,key=lambda f:np.prod(f['half_extents']))['name']
problem=z.ZProblem(folder/'task.json',a.side);native=z.ZNative(problem,folder/'final-model');deadline=json.loads((run/'budget.json').read_text())['started_epoch']+1720
for mode,r in source['results'].items():
 anchor=np.asarray(r['anchor']);r['hold_check']=native.evaluate(anchor)
 for row in r['states']:
  if time.time()>deadline:raise TimeoutError('Original bounded CPU window ended')
  g=next(g for g in problem.goals if g['name']==row['control']);state=g['states'][row['state']];x=np.asarray(row['x']);contact_mode=int(row['check']['active_contact']['mode'])
  c=native.evaluate(x,gamepad_q=state['gamepad_q'],goal=g,state=state,solution=dict(mode=contact_mode));motion=z.motion(problem,x,anchor)
  if mode=='strict' and float(abs(x[4:11]-anchor[4:11]).max())>1e-10:raise RuntimeError('Strict state changed common arm')
  if mode=='realistic' and (motion['centroid_displacement_mm']>20+1e-6 or motion['rotation_deg']>15+1e-6):c['passed']=False;c['reasons'].append('palm_adjustment_limit')
  row.update(check=c,passed=c['passed'],reasons=c['reasons'],motion=motion)
 groups={}
 for row in r['states']:groups.setdefault(row['control'],[]).append(row)
 r['retained_controls']=[name for name,items in groups.items() if r['hold_check']['passed'] and len({v['state'] for v in items if v['passed']}) >= (1 if next(g for g in problem.goals if g['name']==name).get('alternatives') else len(next(g for g in problem.goals if g['name']==name)['states']))];r['count']=len(r['retained_controls'])
expected=json.loads((run/'code/docs/reports/2026-11-04-gamepad-fixture-v3/asset_provenance.json').read_text())['collision_resources']
resource_root=Path(json.loads((folder/'task.json').read_text())['resource_root'])
differences=[r['path'] for r in expected if hashlib.sha256((resource_root/r['path']).read_bytes()).hexdigest()!=r['sha256']]
if differences:raise RuntimeError('Final CPU resources differ from0.4.1/Z: '+str(differences))
source.update(collision_resource_files_verified=len(expected),collision_resources_match_asset_v041_and_Z=True,final_native_replay_completed=True,final_scene_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),compute_wall_seconds_from_supervisor=time.time()-json.loads((run/'budget.json').read_text())['started_epoch'],runtime_versions={n:importlib.metadata.version(n) for n in ('mujoco','xbox-gamepad-physics')})
write_json(folder/'result.json',source)
