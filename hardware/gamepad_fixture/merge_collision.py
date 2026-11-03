#!/usr/bin/env python3
"""Bounded greedy merges of CAD convex cells, independently checked afterwards.

Only touching hulls are candidates. Exact convex unions are preferred; approximate
merges require point-to-CAD error <=1.25 mm and exact protected-space exclusion.
This finds a small cover in this candidate family, not a global optimum.
"""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
import trimesh
from convex_collision import HERE,ROOT,solid,mesh,union,write_obj
from check_collision import outside_distances,protected,sample
sys.path.insert(0,str(ROOT/'src'))
from gamepad_vla.workstation.fixture_collision import convex_objects

def contained(h,q):
    if h.volume>q.volume+1e-5:return False
    if np.any(h.bounds[0]<q.bounds[0]-1e-5) or np.any(h.bounds[1]>q.bounds[1]+1e-5):return False
    n=q.face_normals;d=np.einsum('ij,ij->i',n,q.triangles_center)
    return np.all(h.vertices@n.T<=d+1e-5)

def optimize(src,dest,seconds):
    start=time.monotonic();keep,_=protected();mf=json.loads((HERE/'out/manifest.json').read_text())
    for op in mf['openings']:
        b=np.array(op['bounds_pad_mm']);m=trimesh.creation.box(b[1]-b[0]);m.apply_translation(b.mean(0));keep['opening_'+op['name']]=solid(m)
    dest.mkdir(parents=True,exist_ok=True);rows={}
    for path in sorted(src.glob('*.obj')):
        t=time.monotonic();hulls=[trimesh.Trimesh(v*1000,f,process=False) for v,f in convex_objects(path)];before=len(hulls);cad=trimesh.load(HERE/'out/assembly_mesh'/(path.stem+'.stl'),force='mesh')
        hulls.sort(key=lambda h:-h.volume);retained=[]
        for h in hulls:
            if not any(contained(h,q) for q in retained):retained.append(h)
        hulls=retained;denested=len(hulls);rejected=set();merges=0
        deadline=t+seconds
        while time.monotonic()<deadline:
            candidates=[]
            for i,a in enumerate(hulls):
                for j in range(i):
                    if time.monotonic()>deadline:break
                    if (id(a),id(hulls[j])) in rejected:continue
                    b=hulls[j]
                    gap=np.maximum(a.bounds[0]-b.bounds[1],b.bounds[0]-a.bounds[1]).max()
                    if gap>.02:continue
                    q=trimesh.convex.convex_hull(np.vstack([a.vertices,b.vertices]))
                    if len(q.vertices)>255:rejected.add((id(a),id(b)));continue
                    # Volume beyond the pair. Negative values can mean overlap.
                    score=(q.volume-a.volume-b.volume)/max(q.area,1)
                    candidates.append((score,i,j,q))
            if not candidates:break
            accepted=False
            for score,i,j,q in sorted(candidates,key=lambda v:v[0]):
                if time.monotonic()>deadline:break
                key=(id(hulls[i]),id(hulls[j]));s=solid(q)
                # Protect existing control sweeps, hand keepouts, and ports.
                if any(abs((s^k).volume())>1e-6 for k in keep.values()):rejected.add(key);continue
                added=s-union([solid(hulls[i]),solid(hulls[j])])
                if added.volume()>1e-4:
                    # Check all vertices/face centres plus area samples of the
                    # added solid. No internal hull-to-hull face distance.
                    points=sample(mesh(added),density=4)
                    out=outside_distances(cad,points)
                    if out.max(initial=0)>1.25:rejected.add(key);continue
                hulls=[h for k,h in enumerate(hulls) if k not in (i,j)]+[q];merges+=1;accepted=True;break
            if not accepted:break
        hulls=write_obj(dest/path.name,hulls);rows[path.stem]=dict(before=before,after_containment=denested,after=len(hulls),merges=merges,seconds=time.monotonic()-t)
        print(path.stem,rows[path.stem],flush=True)
    (dest/'merge.json').write_text(json.dumps(dict(parts=rows,hulls=sum(v['after'] for v in rows.values()),seconds=time.monotonic()-start,per_part_budget_seconds=seconds),indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=ROOT/'.local/fixture-convex/cad');p.add_argument('--dest',type=Path,default=ROOT/'.local/fixture-convex/cad_merged');p.add_argument('--seconds',type=float,default=40);a=p.parse_args();optimize(a.source,a.dest,a.seconds)
