#!/usr/bin/env python3
"""Measure sampled support heights and mesh envelopes of saved static grasps."""
import argparse
import json
from pathlib import Path
import numpy as np,mujoco as mj,trimesh
from gamepad_vla.feasibility import Pad,mesh_for_geom,transform
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import pose_to_matrix,load_spec
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runs-root',type=Path,required=True)
parser.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
root=args.runs_root;cfg=load_task(args.task);spec=load_spec(cfg['workcell']);pad=Pad(cfg['resource_root']);pad.state({});shell=pad.mesh('shell')
p,f=trimesh.sample.sample_surface(shell,40000,seed=19);n=shell.face_normals[f]
mask=(abs(p[:,0])<.01)&(abs(p[:,1])<.012)&(n[:,2]<-.3)
idx=np.where(mask)[0];center=idx[np.argmin(np.linalg.norm(p[idx,:2],axis=1))];support=p[center]
base=json.loads((root/'left/result.json').read_text())
out=dict(shell_bbox_m=[shell.bounds[0].tolist(),shell.bounds[1].tolist()],support_point_m=support.tolist(),support_outward_normal=n[center].tolist(),support_sample_count=int(mask.sum()),fixture_support_heights=[])
tabletop=spec.table.pose[2]+spec.table.half_extents[2]
for fx in base['fixtures']:
 h=float(transform(support,pose_to_matrix(fx['pose']))[2]-tabletop)
 out['fixture_support_heights'].append(dict(fixture=fx['name'],root_height_above_table_m=fx['pose'][2]-tabletop,sampled_support_contact_height_above_table_m=h))
out['hand_envelopes']=[]
for side,letter in [('left','L'),('right','R')]:
 w=json.loads((root/side/'witnesses.json').read_text())['b_raise40'][0]
 model=mj.MjModel.from_xml_path(str(root/side/(letter+'H.urdf')));data=mj.MjData(model)
 groups=[w['resting_hand_q']]+[s['hand_q'] for s in w['states'] if s['control'] in w['controls']]
 bounds=[]
 import xml.etree.ElementTree as ET
 xml=ET.parse(root/side/(letter+'H.urdf')).getroot()
 for q in groups:
  for j in xml.findall('joint'):
   if j.get('type')=='fixed':continue
   name=j.get('name');m=j.find('mimic')
   value=q.get(name,0.) if m is None else q.get(m.get('joint'),0.)*float(m.get('multiplier','1'))+float(m.get('offset','0'))
   data.qpos[model.joint(name).qposadr[0]]=value
  mj.mj_forward(model,data)
  vertices=np.concatenate([mesh_for_geom(model,data,g).vertices for g in range(model.ngeom)])
  vertices=transform(vertices,pose_to_matrix(w['palm_in_pad']));bounds.append([vertices.min(axis=0),vertices.max(axis=0)])
 b=np.array(bounds);out['hand_envelopes'].append(dict(side=side,fixture='b_raise40',palm_index=w['palm_index'],controls=w['controls'],bbox_m=[b[:,0].min(axis=0).tolist(),b[:,1].max(axis=0).tolist()]))
out['scope']='Sampled central rear-shell support point and exact mesh vertex envelopes of one accepted grip per side; no clamp mesh or load/stability validation; not a full-control motion envelope.'
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(out,indent=2)+'\n')
