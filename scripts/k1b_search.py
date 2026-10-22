#!/usr/bin/env python3
"""K1-b: natural-arm-first, human-style, bounded multi-start grip optimization.

Approximate distances guide optimization only. A positive result must pass the
unmodified K0 native contact predicates and K1-a mesh/camera/natural gates.
Compute-node use only. No dynamics, training, hardware or environment mutation.
"""
from __future__ import annotations
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import replace
import json,os,resource,time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh

from gamepad_vla.feasibility import Pad,Finger,FINGERS,CollisionScene,transform
from gamepad_vla.kinematics import SerialChain,rpy_to_matrix
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix,matrix_to_pose,FixtureSpec
from k1_natural_reference import ArmFeatures
from k1_height_common import write_json,EXCLUDED,MeshGeometry,CameraGate,support_point,SUPPORT_NAME

MIRROR=np.array([-1,1,-1,1,-1,1,-1])
YAW=Rotation.from_euler('z',-90,degrees=True).as_matrix()


def cpu_seconds():
    a=resource.getrusage(resource.RUSAGE_SELF);b=resource.getrusage(resource.RUSAGE_CHILDREN)
    return a.ru_utime+a.ru_stime+b.ru_utime+b.ru_stime


def make_pose(x,shell_yz,top):
    px,py,h,tilt=x[:4]
    s,c=np.sin(tilt),np.cos(tilt)
    low=float(np.min(shell_yz[:,0]*s+shell_yz[:,1]*c))
    f=np.eye(4);f[:3,:3]=YAW@Rotation.from_euler('x',tilt).as_matrix()
    f[:3,3]=[px,py,top+h-low]
    return f


def column_box(spec,pose):
    f=pose_to_matrix(pose);p=support_point();centre=transform([0,0,p[2]],f)
    top=spec.table.pose[2]+spec.table.half_extents[2]
    # Bounding vertical box contains the later exact sloped-top prism. It is
    # intentionally conservative in screening; final uses the exact mesh.
    crest=float(centre[2]+.012*np.abs(f[2,1]))
    return FixtureSpec(SUPPORT_NAME,'box',(*centre[:2],(crest+top)/2,*matrix_to_pose(np.block([[YAW,np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]))[3:]),
        True,'design','K1-b screening: vertical 20x24mm envelope of the sloped-top central post.',(.010,.012,(crest-top)/2))


def with_column(spec,pose):
    return replace(spec,gamepad=replace(spec.gamepad,root_pose=tuple(pose)),fixtures=(*spec.fixtures,column_box(spec,pose)))


class RobotSurface:
    """Original URDF meshes with a small deterministic point set for proposals."""
    def __init__(self,urdf,root_link,arm=None,per_link=28):
        self.arm=arm;self.root_link=root_link;self.meshes={};self.areas={};self.centres={}
        root=ET.parse(urdf).getroot()
        for link in root.findall('link'):
            name=link.get('name');pieces=[]
            for col in link.findall('collision'):
                el=col.find('geometry/mesh')
                if el is None:continue
                mesh=trimesh.load(Path(urdf).parent/el.get('filename'),force='mesh')
                mesh.vertices*=np.array([float(v) for v in el.get('scale','1 1 1').split()])
                origin=col.find('origin')
                if origin is not None:
                    rr=rpy_to_matrix([float(v) for v in origin.get('rpy','0 0 0').split()])
                    mesh.vertices=mesh.vertices@rr.T+np.array([float(v) for v in origin.get('xyz','0 0 0').split()])
                pieces.append(mesh)
            if not pieces:continue
            mesh=trimesh.util.concatenate(pieces);v=np.array(mesh.vertices)
            ids=np.unique(np.r_[np.linspace(0,len(v)-1,min(per_link,len(v))).astype(int),v.argmin(axis=0),v.argmax(axis=0)])
            self.meshes[name]=v[ids];self.areas[name]=mesh.area;self.centres[name]=np.array(mesh.centroid)
        self.chains={}
        if arm:
            self.chain=SerialChain(urdf,arm.tip_link)
            z=self.chain.link_poses(np.zeros((1,7)))
            zero=np.eye(4) if self.chain.root_link==arm.base_link else z[arm.base_link][0]
            self.base=pose_to_matrix(arm.base_pose)@np.linalg.inv(zero)
        else:
            self.chains={f:SerialChain(urdf,f+'_distal') for f in FINGERS}

    def hand_frames(self,qdict):
        poses={self.root_link:np.eye(4)}
        for f,c in self.chains.items():
            q=np.array([qdict.get(n,0.) for n in c.active])[None]
            poses.update({k:v[0] for k,v in c.link_poses(q).items()})
        return poses

    def points(self,frames,world):
        out=[];labels=[];centres=[];areas=[]
        for name,v in self.meshes.items():
            if name not in frames:continue
            frame=world@frames[name];out.append(transform(v,frame));labels.extend([name]*len(v))
            centres.append(transform(self.centres[name],frame));areas.append(self.areas[name])
        return np.concatenate(out),labels,np.average(centres,axis=0,weights=areas)

    def arm_points(self,q):
        frames={self.root_link:np.eye(4),**{k:v[0] for k,v in self.chain.link_poses(np.asarray(q)[None]).items()}}
        return self.points(frames,self.base)[0]


class Problem:
    def __init__(self,task,side,surface_cache=None):
        self.cfg=load_task(task);self.spec=load_spec(self.cfg['workcell']);self.side=side;self.constraint_priority=1.
        self.arm=getattr(self.spec,side+'_arm');self.hand=getattr(self.spec,side+'_hand')
        self.feature=ArmFeatures(self.arm,self.hand);self.top=self.spec.table.pose[2]+self.spec.table.half_extents[2]
        rng=np.random.default_rng(4101+(side=='right'))
        self.fingers={f:Finger(self.hand.urdf,f,100000,rng) for f in FINGERS}
        self.names=[n for f in FINGERS for n in self.fingers[f].chain.active]
        self.slices={};off=0
        for f in FINGERS:
            self.slices[f]=slice(off,off+len(self.fingers[f].lower));off+=len(self.fingers[f].lower)
        self.lo=np.concatenate([self.fingers[f].lower for f in FINGERS]);self.hi=np.concatenate([self.fingers[f].upper for f in FINGERS])
        self.hand_surface=RobotSurface(self.hand.urdf,self.hand.root_link)
        self.arm_surface=RobotSurface(self.arm.urdf,self.arm.base_link,self.arm,per_link=18)
        if surface_cache is None:
            pad=Pad(self.cfg['resource_root']);pad.state({});shell=pad.mesh('shell')
            p,ids=trimesh.sample.sample_surface(shell,40000,seed=19);n=shell.face_normals[ids]
            sign=-1 if side=='left' else 1
            keep=(sign*p[:,0]>.025)&(p[:,1]<.005)&(p[:,2]<-.005)&(n[:,2]<-.3)
            self.grip_p,self.grip_n=p[keep],n[keep]
            meshes=[]
            for g in range(pad.model.ngeom):
                if pad.model.geom_dataid[g]>=0:
                    from gamepad_vla.feasibility import mesh_for_geom
                    meshes.append(mesh_for_geom(pad.model,pad.data,g))
            whole=trimesh.util.concatenate(meshes)
            allp,allids=trimesh.sample.sample_surface(whole,60000,seed=31)
            self.object_p,self.object_n=allp,whole.face_normals[allids]
            self.shell_yz=np.array(shell.vertices[:,1:3]);self.pad_centre=np.array(whole.centroid)
            self.pad_bbox=np.array([[x,y,z] for x in whole.bounds[:,0] for y in whole.bounds[:,1] for z in whole.bounds[:,2]])
            self.goals=[g for g in pad.targets(side) if g['name'] not in EXCLUDED]
        else:
            for key,value in surface_cache.items():setattr(self,key,value)
        self.grip_tree=cKDTree(self.grip_p);self.object_tree=cKDTree(self.object_p)
        cam=next(c for c in self.spec.cameras if c.name=='camera_3');self.camera=cam;self.cf=pose_to_matrix(cam.pose_optical)
        alo,ahi=self.feature.chain.active_limits
        self.lower=np.r_[.16,.42,.060,0.,alo,self.lo];self.upper=np.r_[.68,.82,.220,np.pi/6,ahi,self.hi]
        self.surface_cache={k:getattr(self,k) for k in ('grip_p','grip_n','object_p','object_n','shell_yz','pad_centre','pad_bbox','goals')}

    def unpack(self,x):
        pad=make_pose(x,self.shell_yz,self.top);qarm=x[4:11]
        af=self.feature.frames(qarm);palm=af['palm'][0]
        qdict=dict(zip(self.names,x[11:]));frames=self.hand_surface.hand_frames(qdict)
        return pad,palm,qarm,qdict,frames

    def project(self,p):
        xyz=(p-self.cf[:3,3])@self.cf[:3,:3];z=np.maximum(xyz[:,2],.02);k=self.camera.intrinsics
        return np.c_[k.fx*xyz[:,0]/z+k.cx,k.fy*xyz[:,1]/z+k.cy],xyz[:,2]

    def residual(self,x,regularize=None,exact_native=None):
        pad,palm,qarm,qdict,frames=self.unpack(x);inv=np.linalg.inv(pad)
        ring=[];normal=[]
        for finger in ('ring','pinky'):
            f=self.fingers[finger];fr=frames[finger+'_distal']
            ring.append(transform(transform(f.points[0],fr),palm));normal.append(palm[:3,:3]@fr[:3,:3]@f.normals[0])
        gp=transform(np.array(ring),inv);_,ids=self.grip_tree.query(gp)
        targets=self.grip_p[ids]-.00015*self.grip_n[ids]
        contact=(gp-targets).ravel()*3*self.constraint_priority
        normal=np.array(normal)@pad[:3,:3]
        normals=(normal+self.grip_n[ids]).ravel()*.0005
        points,labels,centre=self.hand_surface.points(frames,palm)
        lp=transform(points,inv);dist,near=self.object_tree.query(lp)
        signed=np.einsum('ij,ij->i',lp-self.object_p[near],self.object_n[near])
        allowance=np.array([.0005 if name in ('ring_distal','pinky_distal') else -.0015 for name in labels])
        penetration=np.where(dist<.022,np.maximum(0,-signed-allowance),0.)*3
        arm_points=self.arm_surface.arm_points(qarm)
        table=np.maximum(0,self.top+.031-np.r_[points[:,2],arm_points[:,2]])*5
        # Analytic vertical post-envelope distance, a proposal penalty only.
        support=transform([0,0,support_point()[2]],pad);crest=support[2]+.012*abs(pad[2,1])
        cp=(points-np.array([support[0],support[1],(crest+self.top)/2]))@YAW
        size=np.array([.010,.012,(crest-self.top)/2]);delta=np.abs(cp)-size
        sd=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0)
        column=np.maximum(0,.001-sd)*3
        pc=transform(self.pad_centre,pad);m=self.feature.measure(qarm,pc)
        up=m['upper_tilt_deg'][0];pitch=m['forearm_pitch_deg'][0];heading=m['forearm_heading_deg'][0]
        natural=.05*self.constraint_priority*np.array([max(0,(10.2-up)/10),max(0,(up-34.8)/15),max(0,(-9.8-pitch)/20),max(0,(pitch-24.8)/15),max(0,(heading-29.8)/30),max(0,-((m['elbow']-m['shoulder'])[0,0])/.1)])
        # Original distal mesh caps, rather than the grip pad representative.
        tips=[]
        for finger in FINGERS:
            v=self.hand_surface.meshes[finger+'_distal'];cap=v[v[:,2]>=v[:,2].max()-.00015].mean(axis=0)
            tips.append(transform(transform(cap,frames[finger+'_distal']),palm))
        camera_points=np.r_[transform(self.pad_bbox,pad),np.array(tips),palm[None,:3,3]]
        uv,depth=self.project(camera_points);k=self.camera.intrinsics
        cm=np.minimum.reduce([uv[:,0],k.width-1-uv[:,0],uv[:,1],k.height-1-uv[:,1]])
        fov=np.r_[np.maximum(0,44-cm)/k.fx,np.maximum(0,.03-depth)]*3
        distance=np.maximum(0,.303-np.linalg.norm(np.array([centre,pc])-self.cf[:3,3],axis=1))*3
        # Centring, low height and wrist neutrality are weak search preferences;
        # final lexicographic selection uses verified coverage and hard gates.
        cuv,_=self.project(pc[None]);soft=np.r_[(cuv[0]-[k.cx,k.cy])/k.fx*.0005,
              (x[2]-.06)*.001,m['wrist_deviation_deg'][0]/180*.0005]
        reg=np.zeros(len(x)) if regularize is None else (x-regularize)*.00002
        return np.r_[contact,normals,penetration,table,column,natural,fov,distance,soft,reg]

    def summary(self,x):
        pad,palm,qarm,qdict,frames=self.unpack(x);pc=transform(self.pad_centre,pad)
        natural=self.feature.measure(qarm,pc)
        gp=[]
        for finger in ('ring','pinky'):
            gp.append(transform(transform(self.fingers[finger].points[0],frames[finger+'_distal']),palm))
        lp=transform(np.array(gp),np.linalg.inv(pad));distance,ids=self.grip_tree.query(lp)
        points,_,centre=self.hand_surface.points(frames,palm);armpts=self.arm_surface.arm_points(qarm)
        cuv,_=self.project(pc[None]);k=self.camera.intrinsics
        return dict(side=self.side,x=x.tolist(),gamepad_pose=list(matrix_to_pose(pad)),palm_world=list(matrix_to_pose(palm)),
            arm_q=qarm.tolist(),hand_q=qdict,grip_point_error_m=distance.tolist(),
            natural={key:bool(v[0]) if np.asarray(v).dtype.kind=='b' else float(v[0]) for key,v in natural.items() if np.asarray(v).ndim==1},
            approximate_table_clearance_m=float(min(points[:,2].min(),armpts[:,2].min())-self.top),
            centre_error_px=float(np.linalg.norm(cuv[0]-[k.cx,k.cy])),height_mm=float(x[2]*1000),tilt_deg=float(np.degrees(x[3])))


def natural_cloud(problem,seed=5101,count=100000):
    prior=json.loads(Path('docs/reports/2026-10-22-gamepad-height/natural-reference.json').read_text())['left']
    refs=[prior['reference']['solution']]+[v['solution'] for v in prior['corners']]
    qs=np.array([v['q'] for v in refs if v]);rng=np.random.default_rng(seed)
    q=qs[rng.integers(len(qs),size=count)].copy()
    q[:,:4]+=rng.normal(size=(count,4))*[.18,.28,.32,.30]
    lo,hi=problem.feature.chain.active_limits
    q[:,4:]=rng.uniform(lo[4:],hi[4:],(count,3))
    if problem.side=='right':q*=MIRROR
    q=np.clip(q,lo,hi)
    target=np.tile([.46,.6202,1.0],(len(q),1));m=problem.feature.measure(q,target)
    good=m['natural'];natural=q[good]
    f=problem.feature.frames(natural);r=f['palm'][:,:3,:3]
    human=(problem.fingers['ring'].normals[0,0]*r[:,2,0]>.35)&(r[:,0,2]>.20)&(np.abs(r[:,2,2])<.75)
    chosen=natural[human]
    if len(chosen)==0:raise RuntimeError('natural arm cloud has no human-facing wrist seeds')
    summary=dict(raw_samples=count,natural_hard_count=int(good.sum()),human_facing_seed_count=len(chosen),
        seed=seed,mirror_signs=MIRROR.tolist(),
        human_seed_only_filter='source-derived finger pad normal +local x has world z>.35; fingers local+z world x>.2 and abs(world z)<.75; final acceptance does not add this filter',
        natural_ranges={k:[float(v[good].min()),float(v[good].max())] for k,v in m.items() if np.asarray(v).ndim==1 and np.asarray(v).dtype.kind!='b'})
    return chosen,summary


def generate_seeds(problem,cloud,count,seed):
    rng=np.random.default_rng(seed);seeds=[]
    order=rng.permutation(len(cloud))
    for i in range(max(count*10,1000)):
        aq=cloud[order[i%len(order)]];palm=problem.feature.frames(aq)['palm'][0]
        q=np.clip(problem.lo+.30*(problem.hi-problem.lo),problem.lo,problem.hi)
        for f in ('ring','pinky'):
            sl=problem.slices[f];q[sl]=np.array([rng.uniform(-.18,.18),rng.uniform(.25,1.15),rng.uniform(.25,1.50)])
            q[sl]=np.clip(q[sl],problem.lo[sl],problem.hi[sl])
        for f in ('thumb','index','middle'):
            sl=problem.slices[f];q[sl]=problem.lo[sl]+rng.uniform(.1,.7)*(problem.hi[sl]-problem.lo[sl])
        qdict=dict(zip(problem.names,q));frames=problem.hand_surface.hand_frames(qdict)
        points=[transform(problem.fingers[f].points[0],frames[f+'_distal']) for f in ('ring','pinky')]
        localhand=np.mean(points,axis=0)
        target=np.array([(-1 if problem.side=='left' else 1)*.052,-.027,-.020])
        _,idx=problem.grip_tree.query(target);target=problem.grip_p[idx]
        tilt=np.deg2rad([0,10,20,30][i%4]);rot=YAW@Rotation.from_euler('x',tilt).as_matrix()
        root=transform(localhand,palm)-rot@target
        low=np.min(problem.shell_yz[:,0]*np.sin(tilt)+problem.shell_yz[:,1]*np.cos(tilt));height=root[2]+low-problem.top
        x=np.r_[root[:2],height,tilt,aq,q]
        if not (.01<height<.30):continue
        x=np.clip(x,problem.lower+1e-8,problem.upper-1e-8)
        cost=float(np.linalg.norm(problem.residual(x)))
        seeds.append((cost,x))
        if len(seeds)>=count*4:break
    seeds.sort(key=lambda v:v[0])
    # Take ranked seeds across distinct arm states and four tilts, rather than
    # repeating one random rear-shell anchor as in K0.
    return [x for _,x in seeds[:count]]


_WORKER=None

def init_worker(task,side,cache):
    global _WORKER
    _WORKER=Problem(task,side,cache)


def optimize_seed(item):
    index,x,maxn=item;p=_WORKER;start=time.monotonic();cpu0=cpu_seconds()
    result=least_squares(lambda z:p.residual(z,x),x,bounds=(p.lower,p.upper),method='trf',
        x_scale=np.r_[.08,.08,.08,.3,np.ones(7)*.5,np.ones(len(p.lo))*.5],
        max_nfev=maxn,ftol=1e-7,xtol=1e-7,gtol=1e-7)
    out=p.summary(result.x);out.update(seed_index=index,optimizer_cost=float(result.cost),nfev=result.nfev,
        optimizer_success=bool(result.success),elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds()-cpu0)
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    parser.add_argument('--side',choices=['left','right'],required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--starts',type=int,default=256)
    parser.add_argument('--workers',type=int,default=4);parser.add_argument('--max-nfev',type=int,default=140)
    parser.add_argument('--cloud',type=int,default=100000)
    args=parser.parse_args();start=time.monotonic();args.output.mkdir(parents=True,exist_ok=True)
    p=Problem(args.task,args.side);cloud,summary=natural_cloud(p,count=args.cloud)
    np.savez_compressed(args.output/'natural-cloud.npz',arm_q=cloud)
    write_json(args.output/'cloud-summary.json',summary)
    seeds=generate_seeds(p,cloud,args.starts,6201+(args.side=='right'))
    write_json(args.output/'seed-summary.json',dict(count=len(seeds),requested=args.starts,kind='natural-arm-first plus palm-grip human placement'))
    checkpoints=args.output/'checkpoints';checkpoints.mkdir(exist_ok=True)
    outputs=[json.loads(f.read_text()) for f in sorted(checkpoints.glob('*.json'))]
    completed={r['seed_index'] for r in outputs}
    with ProcessPoolExecutor(max_workers=args.workers,initializer=init_worker,initargs=(args.task,args.side,p.surface_cache)) as pool:
        jobs=[pool.submit(optimize_seed,(i,x,args.max_nfev)) for i,x in enumerate(seeds) if i not in completed]
        for future in as_completed(jobs):
            row=future.result();outputs.append(row)
            write_json(checkpoints/f"{row['seed_index']:04d}.json",row)
            if len(outputs)%8==0:
                write_json(args.output/'progress.json',outputs)
                good=sum(r['natural']['natural'] and max(r['grip_point_error_m'])<=.00025 for r in outputs)
                print(json.dumps(dict(stage='optimized',side=args.side,done=len(outputs),point_and_natural=good,cpu_seconds=cpu_seconds()+sum(r['cpu_seconds'] for r in outputs))),flush=True)
            if cpu_seconds()+sum(r['cpu_seconds'] for r in outputs)>17500:raise RuntimeError('per-side 5 CPU-hour phase cap reached; global budget must stop/review')
    outputs.sort(key=lambda r:(not r['natural']['natural'],max(r['grip_point_error_m']),r['optimizer_cost']))
    write_json(args.output/'proposals.json',dict(side=args.side,cloud=summary,starts=len(seeds),workers=args.workers,max_nfev=args.max_nfev,
        proposals=outputs,elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds(),
        acceptance='optimizer proposals only; no candidate is accepted without full native replay'))
    print(json.dumps(dict(stage='done',side=args.side,proposals=len(outputs),elapsed_seconds=time.monotonic()-start,cpu_seconds=cpu_seconds())),flush=True)


if __name__=='__main__':main()
