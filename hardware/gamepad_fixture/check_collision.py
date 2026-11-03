#!/usr/bin/env python3
"""Independent signed CAD error and protected-space checks for collision hulls."""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
import trimesh
from convex_collision import HERE,ROOT,solid,mesh,union
from asset_geometry import ControllerAsset,SHELL_NAMES
sys.path.insert(0,str(ROOT/'src'))
from gamepad_vla.workstation.fixture_collision import convex_objects

def nearest(source,points):
    distances=[]
    for i in range(0,len(points),2000):distances.append(trimesh.proximity.closest_point(source,points[i:i+2000])[1])
    return np.concatenate(distances) if distances else np.empty(0)

def sample(m,density=4,seed=1):
    points,_=trimesh.sample.sample_surface(m,max(100,int(np.ceil(m.area*density))),seed=seed)
    return np.vstack([points,m.vertices,m.triangles_center])

def outside_distances(reference,points):
    # Trimesh uses the nearest outward face for points projecting into a face,
    # and exact ray parity for edge/vertex cases. This is the same closed-solid
    # sign test without ray-casting every nonzero-distance sample.
    outside=[]
    for start in range(0,len(points),2000):
        distance=trimesh.proximity.signed_distance(reference,points[start:start+2000])
        outside.extend((-distance[distance < -1e-5]).tolist())
    return np.asarray(outside)

def summary(v):return dict(max_mm=float(v.max(initial=0)),p95_mm=float(np.percentile(v,95)) if len(v) else 0.,samples=len(v))

def protected():
    asset=ControllerAsset();shapes={}
    for n,m in asset.sweep_envelopes(17).items():shapes['sweep_'+n]=solid(m)
    # Existing fixture design's grip-back keepouts, verbatim.
    for side,lo,hi in [('left',[-80,-60,-48],[-30,3,-12]),('right',[30,-60,-48],[80,3,-12])]:
        b=trimesh.creation.box(np.asarray(hi)-lo);b.apply_translation((np.asarray(hi)+lo)/2);shapes['grip_'+side]=solid(b)
    # Reuse Task Z's displaced control targets. Protect a 1 mm contact patch
    # extruded outward 3 mm, in addition to the entire control sweep. Full-hand
    # accepted-witness checks are performed separately by the native replay.
    goals=ROOT/'docs/reports/2026-11-04-fixture-convex-collision/control_targets.json'
    if goals.exists():
        for side,groups in json.loads(goals.read_text()).items():
            for group in groups:
                for index,state in enumerate(group['states']):
                    n=np.array(state['normal']);n/=np.linalg.norm(n)
                    q=trimesh.creation.cylinder(radius=1.,height=3.,sections=32)
                    T=trimesh.geometry.align_vectors([0,0,1],n);T[:3,3]=np.array(state['point'])*1000+n*1.5
                    q.apply_transform(T);shapes[f'approach_{side}_{group["name"]}_{index}']=solid(q)
    return shapes,asset

def check(directory,density=4):
    start=time.monotonic();keep,asset=protected();rows={};all_out=[];all_in=[];protected_rows=[];contacts={}
    mf=json.loads((HERE/'out/manifest.json').read_text())
    for src in sorted(directory.glob('*.obj')):
        name=src.stem;cad=trimesh.load(HERE/'out/assembly_mesh'/(name+'.stl'),force='mesh')
        hulls=[trimesh.Trimesh(v*1000,f,process=False) for v,f in convex_objects(src)]
        proxy=mesh(union([solid(h) for h in hulls]));points=np.vstack([sample(h,density) for h in hulls]);out=outside_distances(cad,points)
        original=sample(cad,density);inside=np.zeros(len(original),bool)
        for h in hulls:
            mask=np.all(original>=h.bounds[0]-1e-5,axis=1)&np.all(original<=h.bounds[1]+1e-5,axis=1)&~inside
            q=original[mask];normals=h.face_normals;off=np.einsum('ij,ij->i',normals,h.triangles_center)
            inside[np.where(mask)[0]]=np.all(q@normals.T<=off+1e-5,axis=1)
        inset=nearest(proxy,original[~inside]);all_out.extend(out.tolist());all_in.extend(inset.tolist())
        s=solid(proxy);intersections=[]
        for label,k in keep.items():
            v=abs(float((s^k).volume()))
            if v>1e-6:intersections.append(dict(space=label,volume_mm3=v))
        for c in mf['contact_planes']:
            if c['pad']!=name:continue
            # Signed distance along the exact fitted normal at all actual CAD
            # contact-face vertices and area samples (not a whole-pad AABB).
            n=-np.array(c['normal']);P=np.array(c['plane_point_mm']);v=sample(cad,density)
            face=v[abs((v-P)@n)<1e-4]
            error=nearest(proxy,face)
            contacts[c['name']]=dict(source_samples=len(face),**summary(error))
        row=dict(hulls=len(hulls),vertices_max=max(len(h.vertices) for h in hulls),outward=summary(out),inward=summary(inset),protected_intersections=intersections,source_samples=len(original),hull_samples=len(points))
        rows[name]=row;(directory/'accuracy_progress.json').write_text(json.dumps(rows,indent=2)+'\n');protected_rows.extend([dict(part=name,**r) for r in intersections]);print(name,json.dumps(row),flush=True)
    result=dict(executed=True,density_points_per_mm2=density,method='Area samples plus all vertices/face centres; outward only outside CAD solid, exact point-to-triangle distance. Inward only CAD samples not contained in any hull (10 nm tolerance in mm coordinates). Sample maxima are not continuous certificates.',parts=rows,hulls=sum(r['hulls'] for r in rows.values()),outward=summary(np.array(all_out)),inward=summary(np.array(all_in)),contacts=contacts,protected_intersections=protected_rows,control_sweep_samples=17,control_sweep_sagitta_bounds_mm=asset.sweep_sampling_bounds(17),elapsed_seconds=time.monotonic()-start)
    result['gates']=dict(outward=result['outward']['max_mm']<=1.5,contacts=len(contacts)==3 and all(c['source_samples']>0 and c['max_mm']<=.1 for c in contacts.values()),vertices=all(r['vertices_max']<=255 for r in rows.values()),protected_zero=not protected_rows)
    (directory/'accuracy.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='parts'}),flush=True)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--density',type=float,default=4);a=p.parse_args();check(a.directory,a.density)
