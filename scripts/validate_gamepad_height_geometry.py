#!/usr/bin/env python3
"""Small K1 geometry checks using native collisions, calibration and AR5 FK."""
import argparse
from pathlib import Path
import numpy as np
import mujoco as mj
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from k1_natural_reference import ArmFeatures
from k1_height_common import write_json,CameraGate

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
args=p.parse_args()
spec=load_spec('configs/workcell/station_g20_v4_4.json')
# Same collision strategy as the full scan, without expensive robot/pad assets.
s=mj.MjSpec()
b=s.worldbody.add_body(name='column_mover',mocap=True)
b.add_geom(name='column',type=mj.mjtGeom.mjGEOM_BOX,size=[.010,.012,.090],contype=1,conaffinity=1)
b=s.worldbody.add_body(name='probe_mover');b.add_freejoint(name='probe_free')
b.add_geom(name='probe',type=mj.mjtGeom.mjGEOM_SPHERE,size=[.004,0,0],contype=1,conaffinity=1)
m=s.compile();d=mj.MjData(m);results=[]
for height in (.079118,.179118):
 for xy in ((-.060,-.060),(.060,.060)):
  centre=np.array([*xy,height/2]);m.geom_size[m.geom('column').id]=[.010,.012,height/2]
  d.mocap_pos[m.body('column_mover').mocapid[0]]=centre
  for offset,expected in (([0,0,0],True),([.050,0,0],False)):
   d.qpos[:3]=centre+offset;d.qpos[3:7]=[1,0,0,0]
   mj.mj_forward(m,d)
   contact=any(c.dist<0 for c in d.contact)
   assert contact==expected
   results.append(dict(column_height_m=height,xy_m=xy,probe_offset_m=offset,native_contact=contact))
cam=next(c for c in spec.cameras if c.name=='camera_3');frame=pose_to_matrix(cam.pose_optical)
gate=CameraGate(cam,None)
world=np.array([[0,0,1],[.1,.2,1]])@frame[:3,:3].T+frame[:3,3]
uv,z=gate.project(world)
expected=np.array([[cam.intrinsics.cx,cam.intrinsics.cy],
                   [cam.intrinsics.cx+.1*cam.intrinsics.fx,cam.intrinsics.cy+.2*cam.intrinsics.fy]])
assert np.allclose(uv,expected,atol=1e-8) and np.allclose(z,1)
features={}
for side in ('left','right'):
 arm,hand=getattr(spec,side+'_arm'),getattr(spec,side+'_hand');f=ArmFeatures(arm,hand)
 r=f.frames(np.array(arm.home_q));e=r['elbow'][0];pp=r['joint_origins'][0];aa=r['joint_axes'][0]
 onaxis=float(np.linalg.norm(np.cross(e-pp[3],aa[3])))
 proj=np.eye(3)-aa[4:7,:,None]*aa[4:7,None,:]
 gradient=np.sum(np.einsum('kij,kj->ki',proj,r['wrist'][0]-pp[4:7]),axis=0)
 assert onaxis<1e-8 and np.linalg.norm(gradient)<1e-8
 metrics=f.measure(arm.home_q,np.array(spec.gamepad.root_pose[:3]))
 features[side]=dict(elbow_on_j4_error_m=onaxis,wrist_ls_gradient_m=gradient.tolist(),
                    shoulder_axis_gap_m=float(r['shoulder_axis_gap_m'][0]),
                    palm_surface_centroid_local_m=f.palm_point_local.tolist(),
                    home_metrics={k:bool(v[0]) if np.asarray(v).dtype.kind=='b' else float(v[0])
                                  for k,v in metrics.items() if np.asarray(v).ndim==1})
top=spec.table.pose[2]+spec.table.half_extents[2]
write_json(args.output,dict(native_moving_column_checks=results,projection_probe_pixels=uv.tolist(),
    arm_feature_checks=features,baseline_geometry=dict(tabletop_z_m=top,
        gamepad_root_world_m=spec.gamepad.root_pose[:3],gamepad_root_height_m=spec.gamepad.root_pose[2]-top,
        arm_axis_base_height_m=spec.left_arm.base_pose[2]-top,
        ego_optical_height_m=cam.pose_optical[2]-top,ego_axis_world=frame[:3,2].tolist(),
        fov_deg=[float(np.degrees(2*np.arctan(cam.intrinsics.width/(2*cam.intrinsics.fx)))),
                 float(np.degrees(2*np.arctan(cam.intrinsics.height/(2*cam.intrinsics.fy))))]),
    status='passed'))
print('native column 8 checks, optical projection 2 points, FK feature identities: passed')
