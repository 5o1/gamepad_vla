#!/usr/bin/env python3
"""CAD, cooked triangles and cooked convex error provenance."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from convex_collision import HERE,ROOT,solid,union,write_obj
from asset_geometry import ControllerAsset,SHELL_NAMES
sys.path.insert(0,str(ROOT/'src'))
from gamepad_vla.workstation.fixture_collision import convex_objects


def face_set(faces):return set(map(tuple,np.sort(faces,axis=1)))

def main(a):
    from check_collision import protected
    keep,asset=protected();protected_intersections=[];clearances={};arc_bounds=asset.sweep_sampling_bounds(17)
    cooked=json.loads((a.native/'cooked_meshes.json').read_text());convex=json.loads((a.native/'cooked_convex.json').read_text());rows={};overlap={}
    asset=ControllerAsset();body=union([solid(m) for _,m in asset.components(SHELL_NAMES)])
    for name,records in cooked.items():
        source=trimesh.load(HERE/'out/assembly_mesh'/(name+'.stl'),force='mesh');tree=cKDTree(source.vertices);faces=set();error=0.;offsets=[];groups=[]
        for record in records:
            v=np.array(record['vertices_m'])*1000;dist,ids=tree.query(v);error=max(error,float(dist.max(initial=0)));faces.update(face_set(ids[np.array(record['triangles'])]));offsets.append(record['contact_offset_m']);groups.append(record['groups'])
        exact_faces=faces==face_set(source.faces)
        rows[name]=dict(cooked_shapes=len(records),source_faces=len(source.faces),cooked_unique_faces=len(faces),identical_triangle_vertex_mapping=exact_faces,max_corresponding_vertex_error_mm=error,continuous_two_sided_surface_bound_mm=error if exact_faces else None,contact_offsets_m=offsets,groups=groups)
        original_solid=solid(source)
        for label,k in keep.items():
            bounds=np.array(k.bounding_box()).reshape(2,3)
            if np.any(source.bounds[1]<bounds[0]) or np.any(bounds[1]<source.bounds[0]):continue
            volume=abs(float((original_solid^k).volume()))
            if volume>1e-6:protected_intersections.append(dict(part=name,space=label,volume_mm3=volume))
        proxy=union([solid(trimesh.Trimesh(v*1000,f,process=False)) for v,f in convex_objects(a.hulls/(name+'.obj'))])
        clearances[name]={}
        for label,k in keep.items():
            tolerance=arc_bounds[label.removeprefix('sweep_')] if label.startswith('sweep_') else 1-np.cos(np.pi/32) if label.startswith('approach_') else 0.
            cad_gap=float(original_solid.min_gap(k,10.));convex_gap=float(proxy.min_gap(k,10.))
            clearances[name][label]=dict(cad_gap_capped_10mm=cad_gap,convex_gap_capped_10mm=convex_gap,sweep_or_round_patch_bound_mm=float(tolerance),cad_clear_after_bound=bool(cad_gap>tolerance+1e-5),convex_clear_after_bound=bool(convex_gap>tolerance+1e-5))
        overlap[name]=dict(cad_body_intersection_mm3=abs(float((solid(source)^body).volume())),convex_body_intersection_mm3=abs(float((proxy^body).volume())))
    cookdir=a.native/'cooked_convex_obj';cookdir.mkdir(exist_ok=True);crows={}
    for name,records in convex.items():
        hulls=[trimesh.Trimesh(np.array(r['vertices_m'])*1000,np.array(r['triangles']),process=True) for r in records];before=convex_objects(a.hulls/(name+'.obj'));write_obj(cookdir/(name+'.obj'),hulls)
        # Quantify both directions per hull, including vertices and triangle
        # centres. Full CAD signed metrics are checked on these exported hulls.
        from check_collision import nearest,sample
        outward=[];inward=[]
        for (vertices,faces),h in zip(before,hulls):
            original=trimesh.Trimesh(vertices*1000,faces,process=False)
            outward.extend(nearest(original,sample(h,1.)).tolist());inward.extend(nearest(h,sample(original,1.)).tolist())
        crows[name]=dict(input_hulls=len(before),cooked_hulls=len(hulls),input_max_vertices=max(len(v) for v,f in before),cooked_max_vertices=max(len(h.vertices) for h in hulls),two_sided_sample_max_mm=max(max(outward,default=0),max(inward,default=0)),two_sided_sample_p95_mm=float(np.percentile(outward+inward,95)))
        print(name,crows[name],flush=True)
    result=dict(protected_clearances=clearances,cad_protected_intersections=protected_intersections,cad_protected_space_count=len(keep),triangles=rows,convex_cooking=crows,body_overlap=overlap,body_reference='Union of original upstream collision shell/front-plate hulls at the v3 compressed-pad assembly pose; reference only, excluded from fixture contact.',triangle_continuous_bound_mm=max(r['continuous_two_sided_surface_bound_mm'] if r['continuous_two_sided_surface_bound_mm'] is not None else float('inf') for r in rows.values()),triangle_faces_all_match=all(r['identical_triangle_vertex_mapping'] for r in rows.values()))
    a.report.write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--native',type=Path,required=True);p.add_argument('--hulls',type=Path,required=True);p.add_argument('--report',type=Path,required=True);main(p.parse_args())
