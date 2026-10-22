#!/usr/bin/env python3
"""Audit MuJoCo's skipped degree-of-freedom-less pairs without changing K0.

mocap roots do not automatically contact static/mocap objects. This checks those
pairs using native convex geom distances or provable disjoint original-mesh
AABBs; no positive contact is invented from a missing native contact record.
"""
import argparse,json
from pathlib import Path
import numpy as np
from gamepad_vla.feasibility import CollisionScene
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from k1_height_common import (relative_check,with_support,support_point,MeshGeometry,write_json,SUPPORT_NAME,placements)

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
cfg=load_task('configs/tasks/fixed_dpad_causal.json');spec=load_spec(cfg['workcell'])
rel={s:json.loads((getattr(a,s)/'relative.json').read_text()) for s in ('left','right')}
# Extended DOWNWARD column encloses the real column volume in every relative
# placement that can pass the 30mm table gate. Its top is exactly the same shell point.
pose=list(spec.gamepad.root_pose);pose[2]=1.20
scene=CollisionScene(with_support(spec,pose,support_point()),cfg['resource_root'],a.output.parent,force_static_hand_pairs=True)
geo=MeshGeometry(scene);scene.set_pad(pose,{});scene.mj.mj_forward(scene.model,scene.data)
column=scene.model.geom(SUPPORT_NAME).id
padvertices=geo.points('pad');padbounds=np.array([padvertices.min(axis=0),padvertices.max(axis=0)])
def bounds(points):return np.array([points.min(axis=0),points.max(axis=0)])
def boxgap(left,right):return float(np.linalg.norm(np.maximum(0,np.maximum(left[0]-right[1],right[0]-left[1]))))
def static(g):
 bid=scene.model.geom_bodyid[g];weld=scene.model.body_weldid[bid]
 return scene.model.body_dofnum[weld]==0
out={'sides':{},'static_hand_pairs':[],'static_arm_column_aabb_gaps_m':{}}
rootmeshes={}
for side,letter in [('left','L'),('right','R')]:
 rows=[];rootmeshes[side]=[]
 for e in rel[side]['bank']:
  palm=pose_to_matrix(pose)@pose_to_matrix(e['palm_in_pad'])
  basemin=.50;native_static_bad=[]
  for b in e['baselines']:
   for state in [dict(hand_q=b['resting_hand_q'])]+[s for ss in b['operations'].values() for s in ss]:
    scene.set_hand(side,palm,state['hand_q']);scene.mj.mj_forward(scene.model,scene.data)
    # Explicit pairs bypass the automatic no-DOF filter. Keep only root/pad
    # violations here; target/grip dynamic contacts were already checked.
    for c in scene.data.contact:
     g,h=map(int,c.geom)
     if {scene.groups[g],scene.groups[h]}!={'H'+letter,'pad'}:continue
     hg=g if scene.groups[g]=='H'+letter else h
     if static(hg) and c.dist < -1e-6:
      native_static_bad.append(dict(hand_geom=scene.model.geom(hg).name,distance_m=float(c.dist)))
    for g in geo.groups['H'+letter]:
     basemin=min(basemin,float(scene.mj.mj_geomDistance(scene.model,scene.data,g,column,.5,None)))
  # Palm-root meshes do not depend on finger joints.
  staticgeoms=[g for g in geo.groups['H'+letter] if static(g)]
  pts=np.concatenate([geo.vertices(g) for g in staticgeoms]);bb=bounds(pts)
  padgap=boxgap(bb,padbounds)
  rows.append(dict(palm_index=e['palm_index'],whole_hand_extended_column_distance_m=basemin,
                   static_hand_pad_aabb_gap_m=padgap,static_geom_count=len(staticgeoms),native_static_hand_pad_violations=native_static_bad))
  rootmeshes[side].append(dict(index=e['palm_index'],palm=palm,q=e['baselines'][0]['resting_hand_q'],geoms=staticgeoms,bounds=bb))
 out['sides'][side]=rows
for l in rootmeshes['left']:
 for r in rootmeshes['right']:
  gap=boxgap(l['bounds'],r['bounds'])
  if gap<=1e-9:
   for side,e in [('left',l),('right',r)]:scene.set_hand(side,e['palm'],e['q'])
   scene.mj.mj_forward(scene.model,scene.data)
   gap=min(float(scene.mj.mj_geomDistance(scene.model,scene.data,g,h,.5,None)) for g in l['geoms'] for h in r['geoms'])
  out['static_hand_pairs'].append(dict(left_index=l['index'],right_index=r['index'],distance_lower_bound_m=gap))
places=placements(spec,rel['left']['shell_low_m'])
lo=np.min([p['pose'][:3] for p in places],axis=0);hi=np.max([p['pose'][:3] for p in places],axis=0)
columnbounds=np.array([[lo[0]-.012,lo[1]-.010,spec.table.pose[2]+spec.table.half_extents[2]],
                       [hi[0]+.012,hi[1]+.010,hi[2]+support_point()[2]]])
for letter in ('L','R'):
 for g in geo.groups['A'+letter]:
  if static(g):out['static_arm_column_aabb_gaps_m'][scene.model.geom(g).name]=boxgap(bounds(geo.vertices(g)),columnbounds)
out['rejected_static_hand_pairs']=[r for r in out['static_hand_pairs'] if r['distance_lower_bound_m'] < -1e-6]
out['passed']=all(r['whole_hand_extended_column_distance_m']>=-1e-6 and not r['native_static_hand_pad_violations'] for rows in out['sides'].values() for r in rows) and all(v>0 for v in out['static_arm_column_aabb_gaps_m'].values())
write_json(a.output,out)
assert out['passed'],'static-pair audit failed; world counts must be filtered before selection'
print('static palm/pad and support audited; intersecting static palm pairs exported for mandatory pairing rejection')
