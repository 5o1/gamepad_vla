#!/usr/bin/env python3
"""AR5 axis-line features and FK reference heights, reviewer decision 3.

Shoulder: midpoint of closest J1/J2 points. Wrist: least-squares point nearest
J5/J6/J7 axes (null directions tied to the mean of their origins). Elbow: point
on J4 nearest the shoulder-wrist line. No link-origin substitute is used.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
import xml.etree.ElementTree as ET
from gamepad_vla.kinematics import rpy_to_matrix
from scipy.optimize import minimize

from gamepad_vla.kinematics import SerialChain
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from k1_height_common import write_json


def unit(v):
    return v/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),1e-12)


def closest_lines(p,a,q,b):
    c=np.sum(a*b,axis=-1);d=q-p
    ad=np.sum(a*d,axis=-1);bd=np.sum(b*d,axis=-1);den=1-c*c
    safe=np.maximum(den,1e-12)
    t=np.where(den>1e-10,(ad-c*bd)/safe,ad/2)
    u=np.where(den>1e-10,(c*ad-bd)/safe,-bd/2)
    return p+t[...,None]*a,q+u[...,None]*b


class ArmFeatures:
    def __init__(self,arm,hand):
        self.arm,self.hand=arm,hand
        self.chain=SerialChain(arm.urdf,arm.tip_link)
        self.joints=[j for j in self.chain.joints if j['name'] in self.chain.active]
        assert len(self.joints)==7
        z=self.chain.link_poses(np.zeros((1,7)))
        zero=np.eye(4) if self.chain.root_link==arm.base_link else z[arm.base_link][0]
        self.world_from_design=pose_to_matrix(arm.base_pose)@np.linalg.inv(zero)
        self.mount=pose_to_matrix(hand.mount)
        root=ET.parse(hand.urdf).getroot()
        collision=root.find("link[@name='hand_base_link']/collision")
        mesh=collision.find('geometry/mesh')
        geometry=trimesh.load(Path(hand.urdf).parent/mesh.get('filename'),force='mesh')
        geometry.vertices*=np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
        point=np.array(geometry.centroid)
        origin=collision.find('origin')
        if origin is not None:
            point=point@rpy_to_matrix([float(v) for v in origin.get('rpy','0 0 0').split()]).T
            point+=[float(v) for v in origin.get('xyz','0 0 0').split()]
        self.palm_point_local=point

    def frames(self,q):
        poses=self.chain.link_poses(np.atleast_2d(q))
        world={k:self.world_from_design@v for k,v in poses.items()}
        origins=np.stack([world[j['child']][:,:3,3] for j in self.joints],axis=1)
        axes=np.stack([world[j['child']][:,:3,:3]@j['axis'] for j in self.joints],axis=1)
        p1,p2=closest_lines(origins[:,0],axes[:,0],origins[:,1],axes[:,1])
        shoulder=(p1+p2)/2
        pp,aa=origins[:,4:7],axes[:,4:7]
        projectors=np.eye(3)-aa[:,:,:,None]*aa[:,:,None,:]
        mat=projectors.sum(axis=1);rhs=np.einsum('nkij,nkj->ni',projectors,pp)
        anchor=pp.mean(axis=1)
        wrist=anchor+np.einsum('nij,nj->ni',np.linalg.pinv(mat),rhs-np.einsum('nij,nj->ni',mat,anchor))
        elbow,_=closest_lines(origins[:,3],axes[:,3],shoulder,unit(wrist-shoulder))
        palm=world[self.arm.tip_link]@self.mount
        return dict(shoulder=shoulder,elbow=elbow,wrist=wrist,palm=palm,
                    shoulder_axis_gap_m=np.linalg.norm(p1-p2,axis=1),joint_origins=origins,joint_axes=axes)

    def measure(self,q,padcentres):
        f=self.frames(q);upper=unit(f['elbow']-f['shoulder']);fore=unit(f['wrist']-f['elbow'])
        tilt=np.degrees(np.arccos(np.clip(-upper[:,2],-1,1)))
        pitch=np.degrees(np.arcsin(np.clip(fore[:,2],-1,1)))
        target=np.atleast_2d(padcentres)-f['elbow']
        target=unit(target[:,:2]);horizontal=unit(fore[:,:2])
        heading=np.degrees(np.arccos(np.clip(np.sum(target*horizontal,axis=1),-1,1)))
        # Interior elbow angle: vectors elbow->shoulder and elbow->wrist.
        elbow=np.degrees(np.arccos(np.clip(np.sum(-upper*fore,axis=1),-1,1)))
        # G20 hand root +z follows the extended finger chains in the source URDF.
        wrist=np.degrees(np.arccos(np.clip(np.sum(fore*f['palm'][:,:3,2],axis=1),-1,1)))
        tdev=np.abs(tilt-20)/np.where(tilt<20,10.,15.)
        pdev=np.abs(pitch-10)/np.where(pitch<10,20.,15.)
        deviation=np.maximum.reduce([tdev,pdev,heading/30])
        forward=upper[:,0]>0
        # Retain a finite diagnostic score for a backwards upper arm, but never
        # let deviation<=1 mislabel that forbidden direction as natural.
        deviation=np.where(forward,deviation,np.maximum(deviation,1+np.abs(upper[:,0])))
        natural=forward&(tilt>=10)&(tilt<=35)&(pitch>=-10)&(pitch<=25)&(heading<=30)
        return dict(upper_tilt_deg=tilt,forearm_pitch_deg=pitch,forearm_heading_deg=heading,
                    elbow_angle_deg=elbow,wrist_deviation_deg=wrist,deviation=deviation,natural=natural,
                    upper_forward=forward,shoulder=f['shoulder'],elbow=f['elbow'],wrist=f['wrist'],
                    palm_position=f['palm'][:,:3,:3]@self.palm_point_local+f['palm'][:,:3,3],shoulder_axis_gap_m=f['shoulder_axis_gap_m'])


def solve_reference(features,padcentre,tilt,pitch,previous=None,warm_starts=()):
    """Constrained nearest-home FK reference; deterministic or corner-warm starts."""
    lo,hi=features.chain.active_limits;home=np.asarray(features.arm.home_q)
    rng=np.random.default_rng(2203)
    starts=([np.asarray(q) for q in warm_starts] if len(warm_starts) else [home]+[rng.uniform(lo,hi) for _ in range(5)])
    if previous is not None:starts[1]=previous
    targetupper=np.array([np.sin(np.deg2rad(tilt)),0,-np.cos(np.deg2rad(tilt))])
    targetpitch=np.sin(np.deg2rad(pitch))
    def values(q):
        f=features.frames(q);up=unit(f['elbow']-f['shoulder'])[0];fore=unit(f['wrist']-f['elbow'])[0]
        towards=unit((np.asarray(padcentre)-f['elbow'][0])[:2])
        return up,fore,towards
    def eq(q):
        up,fore,towards=values(q)
        return np.array([up[0]-targetupper[0],up[1],fore[2]-targetpitch,
                         fore[0]*towards[1]-fore[1]*towards[0]])
    def ineq(q):
        up,fore,towards=values(q)
        return np.array([-up[2],np.dot(fore[:2],towards)])
    best=None;failures=[]
    for i,seed in enumerate(starts):
        result=minimize(lambda q:float(np.sum((q-home)**2)),seed,method='SLSQP',
            bounds=list(zip(lo,hi)),constraints=[dict(type='eq',fun=eq),dict(type='ineq',fun=ineq)],
            options=dict(maxiter=250,ftol=1e-10))
        residual=float(np.max(np.abs(eq(result.x))));valid=residual<1e-5 and np.min(ineq(result.x))>0
        if not valid:
            failures.append(dict(start=i,message=str(result.message),constraint_residual=residual));continue
        if best is None or result.fun<best['home_squared_distance_rad2']:
            m=features.measure(result.x,np.array(padcentre));f=features.frames(result.x)
            best=dict(q=result.x.tolist(),home_squared_distance_rad2=float(result.fun),start=i,
                constraint_residual=residual,optimizer_success=bool(result.success),
                upper_tilt_target_deg=tilt,forearm_pitch_target_deg=pitch,
                wrist_world_m=f['wrist'][0].tolist(),palm_world_m=(f['palm'][0,:3,:3]@features.palm_point_local+f['palm'][0,:3,3]).tolist(),
                shoulder_world_m=f['shoulder'][0].tolist(),elbow_world_m=f['elbow'][0].tolist(),
                angles={k:float(m[k][0]) for k in ('upper_tilt_deg','forearm_pitch_deg','forearm_heading_deg','elbow_angle_deg','wrist_deviation_deg')})
    return dict(solution=best,failed_starts=failures,
        scope=f'nearest-home among {len(starts)} local constrained FK starts; no global optimum or collision certificate')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--selected',type=Path,required=True)
    p.add_argument('--left-relative',type=Path,required=True);p.add_argument('--right-relative',type=Path,required=True)
    p.add_argument('--spec',type=Path,default=Path('configs/workcell/station_g20_v4_4.json'))
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();pair=json.loads(args.selected.read_text());spec=load_spec(args.spec)
    top=spec.table.pose[2]+spec.table.half_extents[2]
    centre=np.array(pair['placement']['gamepad_centroid_world_m']);out={}
    for side in ('left','right'):
        features=ArmFeatures(getattr(spec,side+'_arm'),getattr(spec,side+'_hand'))
        rel=json.loads((getattr(args,side+'_relative')/'relative.json').read_text())
        shell_low=rel['shell_low_m']
        offsets=[float((pose_to_matrix(e['palm_in_pad'])[:3,:3]@features.palm_point_local+pose_to_matrix(e['palm_in_pad'])[:3,3])[2]-shell_low) for e in rel['bank']]
        median=float(np.median(offsets))
        selected_frame=pose_to_matrix(pair[side]['palm_in_pad'])
        selected_offset=(selected_frame[:3,:3]@features.palm_point_local+selected_frame[:3,3])[2]-shell_low
        reference=solve_reference(features,centre,20,10)
        prev=np.array(reference['solution']['q']) if reference['solution'] else None
        corners=[solve_reference(features,centre,tilt,pitch,prev) for tilt in (10,35) for pitch in (-10,25)]
        if reference['solution'] is None:
            warm=[c['solution']['q'] for c in corners if c['solution']]
            if warm:
                initial=reference
                reference=solve_reference(features,centre,20,10,warm_starts=warm)
                reference['initial_failed_starts']=initial['failed_starts']
                reference['warm_start_source']='already converged tolerance-corner FK configurations'
        for ref in [reference]+corners:
            if ref['solution']:
                s=ref['solution'];s['wrist_height_above_table_m']=s['wrist_world_m'][2]-top
                s['palm_height_above_table_m']=s['palm_world_m'][2]-top
                s['reference_shell_clearance_m']=s['palm_height_above_table_m']-median
                s['selected_grasp_reference_shell_clearance_m']=s['palm_height_above_table_m']-selected_offset
        heights=[r['solution']['reference_shell_clearance_m'] for r in corners if r['solution']]
        out[side]=dict(palm_height_above_shell_bottom_offsets_m=offsets,median_offset_m=median,
            selected_grasp_offset_m=float(selected_offset),reference=reference,corners=corners,
            corner_shell_clearance_range_m=[min(heights),max(heights)] if heights else None,
            palm_definition='G20 hand_base_link original mesh surface-area centroid; finger extension axis root +z from URDF',palm_point_local_m=features.palm_point_local.tolist(),
            median_population='all K0-sampled palms with native rear-shell grip and whole-hand pad clearance; one value per palm')
    write_json(args.output,out);print(json.dumps(out),flush=True)


if __name__=='__main__':main()
