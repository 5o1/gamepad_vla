#!/usr/bin/env python3
"""CAD CSG convex cover, plus optional CoACD comparison, in controller mm.

Method A replays fixture.py's additive primitives and subtractive operations.
Convex subtraction partitions by cutter face planes; no voxelization or hollow
shell is decomposed. Small drill holes may be filled within a 1 mm allowance.
All output is measured independently against the final production STL solids.
"""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
import manifold3d as man
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def solid(mesh):
    return man.Manifold(man.Mesh(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.uint32)))

def mesh(value):
    q=value.to_mesh()
    return trimesh.Trimesh(np.asarray(q.vert_properties)[:,:3],np.asarray(q.tri_verts),process=True)

def union(cells):return man.Manifold.batch_boolean(cells,man.OpType.Add)

def transform(value,matrix):return value.transform(np.asarray(matrix)[:3,:4])

def subtract_convex(cell,tool):
    if (cell ^ tool).volume()<1e-7:return [cell]
    tm=mesh(tool);planes=np.c_[tm.face_normals,np.einsum('ij,ij->i',tm.face_normals,tm.triangles_center)]
    planes=np.unique(np.round(planes,8),axis=0)
    outside=[];remaining=cell
    for p in planes:
        a,remaining=remaining.split_by_plane(p[:3],float(p[3]))
        if a.volume()>1e-6:outside.append(a)
        if remaining.volume()<1e-6:break
    return outside

class CSG:
    def __init__(self,cells,*,cutter=None):self.cells=cells;self.cutter=cutter
    def __add__(self,other):return CSG(self.cells+other.cells)
    def __sub__(self,other):
        tools=other.cutter if other.cutter is not None else other.cells
        cells=self.cells
        for tool in tools:cells=[p for c in cells for p in subtract_convex(c,tool)]
        return CSG(cells)
    def __and__(self,other):return CSG([union(self.cells)^union(other.cells)])
    @property
    def volume(self):return union(self.cells).volume()
    def bounding_box(self):
        bounds=np.asarray(union(self.cells).bounding_box()).reshape(2,3)
        return SimpleNamespace(min=SimpleNamespace(X=bounds[0,0],Y=bounds[0,1],Z=bounds[0,2]),max=SimpleNamespace(X=bounds[1,0],Y=bounds[1,1],Z=bounds[1,2]))
    def moved(self,loc):
        q=loc.wrapped.Transformation();M=np.array([[q.Value(i+1,j+1) for j in range(4)] for i in range(3)]+[[0,0,0,1.]])
        return CSG([transform(c,M) for c in self.cells],cutter=None if self.cutter is None else [transform(c,M) for c in self.cutter])
    def rotate(self,axis,degrees):
        M=trimesh.transformations.rotation_matrix(math.radians(degrees),list(axis.direction),list(axis.position))
        return CSG([transform(c,M) for c in self.cells],cutter=None if self.cutter is None else [transform(c,M) for c in self.cutter])

def box(x0,x1,y0,y1,z0,z1):
    q=trimesh.creation.box([x1-x0,y1-y0,z1-z0]);q.apply_translation([(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]);return CSG([solid(q)])

def cyl(radius,length,center,axis='z'):
    R=Rotation.from_euler('y',90,degrees=True).as_matrix() if axis=='x' else Rotation.from_euler('x',90,degrees=True).as_matrix() if axis=='y' else np.eye(3)
    def make(r,n):
        q=trimesh.creation.cylinder(radius=r,height=length,sections=n);T=np.eye(4);T[:3,:3]=R;T[:3,3]=center;q.apply_transform(T);return solid(q)
    # Choose the cheapest inscribed drill exclusion under the geometric gate.
    # For r<=3.5 a square loses <=r*(1-cos(pi/4))<=1.026 mm.
    # Larger holes use an octagon with .5 mm radial allowance: even r=6.25
    # loses only .5+(r-.5)*(1-cos(pi/8))<.94 mm. Hole centres stay empty.
    cutters=[] if radius<=1.0 else [make(radius,4) if radius<=3.5 else make(radius-.5,8)]
    return CSG([make(radius,64)],cutter=cutters)

def prism(points,direction):
    v=np.asarray(points);q=trimesh.convex.convex_hull(np.vstack([v,v+direction]));return CSG([solid(q)])

def hirth(p,moving=False):
    i,t=p.installation,p.printing;k=math.tan(math.radians(i.tooth_flank_deg));pitch=2*math.pi/i.teeth;half=pitch/2;rings=[]
    for r in (i.tooth_inner_radius,i.tooth_outer_radius):
        maximum=k*(r*math.sin(half)-i.tooth_root_flat/2-i.tooth_tip_flat/2)
        transitions=[math.asin((height/k+i.tooth_root_flat/2)/r) for height in (0,t.tooth_axial_clearance,maximum,maximum+t.tooth_axial_clearance)]
        a=np.array([0,*transitions,half,*[pitch-a for a in transitions[::-1]],pitch]);raw=k*np.minimum(r*np.sin(a)-i.tooth_root_flat/2,r*np.sin(pitch-a)-i.tooth_root_flat/2)
        height=np.clip(raw,t.tooth_axial_clearance,maximum+t.tooth_axial_clearance) if moving else np.clip(raw,0,maximum)
        rings.append((r,a,height))
    cells=[]
    for tooth in range(i.teeth):
        # Moving teeth have the solid on the opposite side of the tent-shaped
        # flank. One hull across both flanks fills the valley by >1.5 mm;
        # retain the two half-tooth convex regions separately.
        spans=(range(6),range(5,11)) if moving else (range(11),)
        for indices in spans:
            vertices=[]
            for r,a,h in rings:
                for j in indices:
                    angle=a[j]+tooth*pitch;x=h[j]
                    vertices.extend([(i.pivot_fixed_face_x+x,r*math.cos(angle),r*math.sin(angle)),(-9 if moving else -21,r*math.cos(angle),r*math.sin(angle))])
            cells.append(solid(trimesh.convex.convex_hull(vertices)))
    return CSG(cells)+cyl(i.fixed_disc_radius,3 if moving else 4,(-10.5 if moving else -19,0,0),'x')

def cad_cover():
    import fixture as f
    originals={n:getattr(f,n) for n in ('box','cyl','prism','hirth','sweep')}
    try:
        for n in ('box','cyl','prism','hirth'):setattr(f,n,globals()[n])
        # Procurement springs are visual-only, as in the baseline. Avoid OCC
        # helix construction when replaying the structural collision CSG.
        f.sweep=lambda *a,**k:box(-1,1,-1,1,-1,1)
        parts,_,_=f.build()
        return {n:[mesh(c).convex_hull for c in shape.cells if c.volume()>1e-3] for n,shape in parts.items() if not n.startswith('hardware_')}
    finally:
        for n,fn in originals.items():setattr(f,n,fn)

def write_obj(path,hulls):
    normalized=[]
    for h in hulls:
        vertices=np.unique(np.round(h.vertices,4),axis=0)
        if len(vertices)<4 or np.linalg.matrix_rank(vertices-vertices.mean(0),tol=1e-7)<3:continue
        normalized.append(trimesh.convex.convex_hull(vertices))
    hulls=normalized
    if not hulls:raise ValueError(f'{path}: no volumetric hulls after 0.1 micrometre rounding')
    path.parent.mkdir(parents=True,exist_ok=True);rows=['# controller-local metres; one closed convex hull per object'];offset=0
    for i,h in enumerate(hulls):
        if len(h.vertices)>255:raise ValueError(f'{path}: hull {i} has {len(h.vertices)} vertices')
        rows.append(f'o convex_{i:04d}')
        rows += ['v '+' '.join(f'{x*.001:.12g}' for x in p) for p in h.vertices]
        rows += ['f '+' '.join(str(int(x)+offset+1) for x in face) for face in h.faces]
        offset+=len(h.vertices)
    path.write_text('\n'.join(rows)+'\n')
    return hulls

def main():
    p=argparse.ArgumentParser();p.add_argument('--method',choices=['cad','coacd'],default='cad');p.add_argument('--threshold',type=float,default=.03);p.add_argument('--output',type=Path);p.add_argument('--part');p.add_argument('--mcts-iterations',type=int,default=60);p.add_argument('--mcts-nodes',type=int,default=10);a=p.parse_args()
    target=a.output or ROOT/'.local/fixture-convex'/a.method
    target.mkdir(parents=True,exist_ok=True);start=time.monotonic()
    if a.method=='cad':parts=cad_cover()
    else:
        import coacd
        coacd.set_log_level('error');parts={}
        for src in sorted((HERE/'out/assembly_mesh').glob('*.stl')):
            if src.stem.startswith('hardware_') or (a.part and src.stem!=a.part):continue
            m=trimesh.load(src,force='mesh');assert m.is_watertight
            # Exact convex inputs need no approximate decomposition. Keep this
            # fast path explicit in metadata; the .005 pilot exposed severe
            # oversplitting of the already-convex cylindrical guide rods.
            if m.is_convex:
                parts[src.stem]=[m.convex_hull]
            else:
                parts[src.stem]=[trimesh.Trimesh(v,f,process=True) for v,f in coacd.run_coacd(coacd.Mesh(m.vertices,m.faces),threshold=a.threshold,preprocess_mode='off',max_convex_hull=-1,decimate=True,max_ch_vertex=255,seed=0,mcts_iterations=a.mcts_iterations,mcts_nodes=a.mcts_nodes)]
    rows={}
    for n,hulls in parts.items():
        hulls=write_obj(target/(n+'.obj'),hulls)
        rows[n]=dict(hulls=len(hulls),max_vertices=max(len(h.vertices) for h in hulls),exact_convex_input=bool(trimesh.load(HERE/'out/assembly_mesh'/(n+'.stl'),force='mesh').is_convex) if a.method=='coacd' else False)
        print(n,rows[n],flush=True)
    result=dict(coacd_parameters=dict(mcts_iterations=a.mcts_iterations,mcts_nodes=a.mcts_nodes,mcts_max_depth=3,seed=0,preprocess_mode='off',max_convex_hull=-1,decimate=True,max_ch_vertex=255) if a.method=='coacd' else None,vertex_grid_mm=.0001,maximum_rounding_distance_mm=math.sqrt(3)*.00005,method=a.method,threshold=a.threshold if a.method=='coacd' else None,parts=rows,hulls=sum(x['hulls'] for x in rows.values()),elapsed_seconds=time.monotonic()-start)
    (target/(f'generation_{a.part}.json' if a.part else 'generation.json')).write_text(json.dumps(result,indent=2)+'\n');print(result['hulls'],result['elapsed_seconds'],flush=True)
if __name__=='__main__':main()
