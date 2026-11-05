"""AE B/C bounded CPU replay; no hardware, CAD, or production parameter changes.

Use existing production SAPIEN scene/loaders and operator snapshot cameras.
Clearance uses actual cooked SAPIEN mesh triangles, not fixture box proxies.
"""
import argparse
import gc
import json
import os
from pathlib import Path
import time
import numpy as np

HERE = Path(__file__).parent


def write(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, indent=2, default=lambda x: x.item() if isinstance(x, np.generic) else x.tolist())+'\n')


def targets():
    return json.loads((HERE/'hand_states.json').read_text())


def set_pose(b, row, *, drives=True, gamepad=True):
    from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
    side=row['side'];prefix='right_' if side=='right' else ''
    values=dict(zip(getattr(b.spec,side+'_arm').joint_names,row['arm_q']))
    values.update(zip([prefix+n for n in G20_JOINT_NAMES],row['hand_q']))
    robot=b.robots[side]
    robot.set_qpos(np.array([values[n] for n in b.joint_order[side]]));robot.set_qvel(np.zeros(len(values)))
    if drives:
        for name,v in values.items():b.joints[side][name].set_drive_target(float(v))
    if gamepad:
        q={j.name:float(row['gamepad_q'].get(j.name,0)) for j in b.gamepad.articulation.get_active_joints()}
        b.gamepad.set_joint_positions(q);b.gamepad.articulation.set_qvel(np.zeros(len(q)))


def shape_mesh(actor, shape):
    T=(actor.get_entity_pose()*shape.local_pose).to_transformation_matrix() if hasattr(actor,'get_entity_pose') else (actor.get_pose()*shape.local_pose).to_transformation_matrix()
    if hasattr(shape,'get_vertices'):
        v=np.asarray(shape.get_vertices())*shape.get_scale();f=np.asarray(shape.get_triangles(),int)
    else:
        import trimesh
        if 'Box' not in type(shape).__name__:raise TypeError(type(shape))
        mesh=trimesh.creation.box(extents=np.asarray(shape.half_size)*2);v=np.asarray(mesh.vertices);f=np.asarray(mesh.faces)
    return v@T[:3,:3].T+T[:3,3],f


def box_distance(amin,amax,bmin,bmax):
    return np.linalg.norm(np.maximum(np.maximum(amin-bmax,bmin-amax),0),axis=-1)


def triangle_pair_distance(A,B):
    """Exact nonintersecting triangle distance incl edge/edge, vectorized pairs."""
    from trimesh.triangles import closest_point
    n=len(A);best=np.full(n,np.inf)
    for i in range(3):
        best=np.minimum(best,np.linalg.norm(A[:,i]-closest_point(B,A[:,i]),axis=1))
        best=np.minimum(best,np.linalg.norm(B[:,i]-closest_point(A,B[:,i]),axis=1))
    for i in range(3):
        p=A[:,i];u=A[:,(i+1)%3]-p
        for j in range(3):
            q=B[:,j];v=B[:,(j+1)%3]-q;w=p-q
            a=np.einsum('ij,ij->i',u,u);bb=np.einsum('ij,ij->i',u,v);c=np.einsum('ij,ij->i',v,v)
            d=np.einsum('ij,ij->i',u,w);e=np.einsum('ij,ij->i',v,w);den=a*c-bb*bb
            # Interior candidate; clamped boundary candidates handled separately.
            s=np.divide(bb*e-c*d,den,out=np.zeros(n),where=abs(den)>1e-24)
            t=np.divide(a*e-bb*d,den,out=np.zeros(n),where=abs(den)>1e-24)
            interior=(s>=0)&(s<=1)&(t>=0)&(t<=1)&(abs(den)>1e-24)
            dist=np.linalg.norm(w+s[:,None]*u-t[:,None]*v,axis=1)
            best=np.minimum(best,np.where(interior,dist,np.inf))
            for se in (0.,1.):
                te=np.clip(np.divide(e+se*bb,c,out=np.zeros(n),where=c>0),0,1)
                best=np.minimum(best,np.linalg.norm(w+se*u-te[:,None]*v,axis=1))
            for te in (0.,1.):
                se=np.clip(np.divide(te*bb-d,a,out=np.zeros(n),where=a>0),0,1)
                best=np.minimum(best,np.linalg.norm(w+se[:,None]*u-te*v,axis=1))
    # Segment/triangle crossing (edges of A through B and vice versa).
    for X,Y in ((A,B),(B,A)):
        e1=Y[:,1]-Y[:,0];e2=Y[:,2]-Y[:,0]
        for i in range(3):
            origin=X[:,i];direction=X[:,(i+1)%3]-origin
            h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h);ok=abs(det)>1e-20
            inv=np.divide(1,det,out=np.zeros(n),where=ok);s=origin-Y[:,0]
            u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1)
            v=inv*np.einsum('ij,ij->i',direction,q);t=inv*np.einsum('ij,ij->i',e2,q)
            best=np.where(ok&(u>=0)&(v>=0)&(u+v<=1)&(t>=0)&(t<=1),0,best)
    return best


def mesh_distance(av,af,bv,bf,cap=.02):
    if box_distance(av.min(0),av.max(0),bv.min(0),bv.max(0))>=cap:return cap
    A=av[af];B=bv[bf];alo,ahi=A.min(1),A.max(1);blo,bhi=B.min(1),B.max(1)
    candidates=np.where(box_distance(av.min(0),av.max(0),blo,bhi)<cap)[0]
    best=cap
    for k in candidates:
        ids=np.where(box_distance(alo,ahi,blo[k],bhi[k])<best)[0]
        if len(ids):best=min(best,float(triangle_pair_distance(A[ids],np.broadcast_to(B[k],(len(ids),3,3))).min()))
        if best<1e-9:break
    return best


def clearance(spec, out):
    from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
    from gamepad_vla.workstation.fixture_collision import sapien_shapes
    b=CollectionSapienBackend(spec,work_dir=out/'backend',environment_collisions=True,gamepad_collisions=True)
    fixed=[]
    for a in b._fixed_actors:
        if a.name.startswith('v3_visual_'):
            for s in sapien_shapes(a):fixed.append((a.name,*shape_mesh(a,s)))
    assert len(fixed)==14,(len(fixed),[a for a,_,_ in fixed])
    result=dict(fixture_meshes=14,distance_cap_mm=20.,method='Exact cooked SAPIEN triangle-pair vertex/face, edge/edge and edge/triangle intersection; AABB-pruned at 20 mm. Distances >=20 mm reported as lower bounds. PhysX contact manifolds measured separately at each saved pose.',states=[])
    for row in targets():
        t=time.monotonic();b.reset();set_pose(b,row)
        shapes=[]
        for link in b.robots[row['side']].get_links():
            for s in sapien_shapes(link):
                shapes.append((link.name,*shape_mesh(link,s)))
        per_fixture=[]
        for name,bv,bf in fixed:
            best=.02;linkname=None
            for ln,av,af in shapes:
                if box_distance(av.min(0),av.max(0),bv.min(0),bv.max(0))>=best:continue
                distance=mesh_distance(av,af,bv,bf,best)
                if distance<best:best=distance;linkname=ln
            per_fixture.append(dict(fixture=name,robot_link=linkname,distance_mm=best*1000,capped=best>=.02-1e-12))
        b.scene.step();contacts=[];fx={a for a in b._fixed_actors if a.name.startswith('v3_visual_')};robot={l.entity for l in b.robots[row['side']].get_links()}
        for c in b.scene.get_contacts():
            a,d=[x.entity for x in c.bodies]
            if (a in fx and d in robot) or (d in fx and a in robot):
                contacts.append(dict(bodies=[a.name,d.name],minimum_separation_mm=min(float(p.separation)*1000 for p in c.points),point_count=len(c.points)))
        result['states'].append(dict(**{k:row[k] for k in ('label','side','mode','control','state')},minimum_distance_mm=min(r['distance_mm'] for r in per_fixture),nearest=sorted(per_fixture,key=lambda x:x['distance_mm'])[:5],per_fixture=per_fixture,contacts=contacts,elapsed_seconds=time.monotonic()-t))
        write(out/'clearance.json',result);print('clearance',row['label'],result['states'][-1]['minimum_distance_mm'],len(contacts),flush=True)
    b.close()


if __name__=='__main__':
    from gamepad_vla.workstation.spec import load_spec
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    clearance(load_spec(a.spec),a.out)
