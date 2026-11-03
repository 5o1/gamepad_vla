"""Deterministic normal-ray FDM screening of the *exported print geometry*.

Adapted from .local/review/2026-11-03/check_printability.py. Every flag is
retained (including isolated face-centre hits), and exemptions are spatial,
kind-specific, reasoned records; an unwaived flag makes the gate fail. The
sampling gate is necessary screening, not a proof of sliceability or strength.
"""
from __future__ import annotations
import argparse
import hashlib
import itertools
import json
import os
import tempfile
from pathlib import Path
import numpy as np
import trimesh

KINDS=('thin_wall','narrow_gap','unsupported_overhang')


def clusters(points, values, kind, cell_mm=2.):
    if len(points)==0:return []
    keys=np.floor(points/cell_mm).astype(int)
    unique,inv=np.unique(keys,axis=0,return_inverse=True)
    lookup={tuple(k):j for j,k in enumerate(unique)}
    labels=np.full(len(unique),-1,int);count=0
    offsets=list(itertools.product((-1,0,1),repeat=3))
    for i in range(len(unique)):
        if labels[i]>=0:continue
        todo=[i];labels[i]=count
        while todo:
            j=todo.pop()
            for d in offsets:
                near=lookup.get(tuple(unique[j]+d))
                if near is not None and labels[near]<0:
                    labels[near]=count;todo.append(near)
        count+=1
    point_labels=labels[inv]
    rows=[]
    for j in range(count):
        selected=point_labels==j;p=points[selected];v=values[selected]
        rows.append({'kind':kind,'points':int(selected.sum()),'minimum_mm':float(v.min()),
                     'centre_print_mm':p.mean(0).tolist(),'bounds_print_mm':[p.min(0).tolist(),p.max(0).tolist()],
                     'extent_mm':np.ptp(p,axis=0).tolist()})
    return sorted(rows,key=lambda x:-x['points'])


def region_mask(points,normals,region,kind):
    if kind not in region.get('kinds',[region.get('kind')]):return np.zeros(len(points),bool)
    bounds=np.asarray(region['bounds_print_mm'],float)
    selected=np.all((points>=bounds[0]-1e-5)&(points<=bounds[1]+1e-5),axis=1)
    if 'radial_interval_mm' in region:
        axes=region.get('radial_axes',[0,1]);centre=np.asarray(region['radial_centre_print_mm'],float)
        radius=np.linalg.norm((points-centre)[:,axes],axis=1)
        lo,hi=region['radial_interval_mm'];selected&=(radius>=lo)&(radius<=hi)
    if 'normal_print' in region:
        n=np.asarray(region['normal_print'],float);n/=np.linalg.norm(n)
        selected&=normals@n>=np.cos(np.radians(region.get('normal_tolerance_deg',5)))
    return selected


def probe(path, *, line_width=.42,samples=60000,exemptions=(),support_regions=(),cache_dir=None):
    path=Path(path);raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    cache_key=hashlib.sha256(f'{digest}/{samples}/v3-ray-4'.encode()).hexdigest()
    cache=Path(cache_dir)/f'{cache_key}.npz' if cache_dir else None
    mesh=trimesh.load(path,force='mesh',process=True)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        return {'pass':False,'error':'Print STL must be watertight with consistent outward winding','sha256':digest}
    if cache is not None and cache.exists():
        with np.load(cache) as z:pts,normals,wall,gap,down=[z[k] for k in ('pts','normals','wall','gap','down')]
    else:
        pts,fid=trimesh.sample.sample_surface(mesh,samples,seed=0)
        # All tiny triangles get a deterministic witness, not just the ones
        # random area-weighted sampling happens to visit. Large-face centres
        # ensure a horizontal unsupported roof is still represented.
        centres=mesh.triangles_center
        pts=np.vstack((pts,centres));fid=np.concatenate((fid,np.arange(len(centres))))
        normals=mesh.face_normals[fid];eps=.001
        def rays(directions):
            origins=pts+eps*directions
            loc,ray,_=mesh.ray.intersects_location(origins,directions,multiple_hits=False)
            distance=np.full(len(pts),np.inf)
            if len(ray):distance[ray]=np.linalg.norm(loc-origins[ray],axis=1)+eps
            return distance
        wall=rays(-normals);gap=rays(normals)
        # Vertical ray measures the gap that the slicer would have to bridge
        # or fill with support. The bed is an available support surface.
        direction=np.tile([0.,0.,-1.],(len(pts),1))
        down=rays(direction)
        down=np.minimum(down,np.maximum(pts[:,2],0))
        if cache is not None:
            cache.parent.mkdir(parents=True,exist_ok=True)
            # Variants share geometry hashes and may finish concurrently.
            # Publish a complete cache file so another checker cannot read a
            # half-written ZIP archive.
            with tempfile.NamedTemporaryFile(dir=cache.parent,suffix='.npz',delete=False) as temporary:
                np.savez_compressed(temporary,pts=pts,normals=normals,wall=wall,gap=gap,down=down)
                temporary_path=Path(temporary.name)
            temporary_path.replace(cache)
    raw_masks={'thin_wall':wall<2*line_width,'narrow_gap':gap<1.5*line_width,
               'unsupported_overhang':(normals[:,2]<-np.cos(np.radians(45))-.00001)&(pts[:,2]>.21)&(down>.21)}
    values={'thin_wall':wall,'narrow_gap':gap,'unsupported_overhang':down}
    clean_masks={};waivers=[];rows=[]
    for kind,mask in raw_masks.items():
        waived=np.zeros(len(pts),bool)
        candidates=list(exemptions)+(list(support_regions) if kind=='unsupported_overhang' else [])
        for j,region in enumerate(candidates):
            if not str(region.get('reason','')).strip():continue
            r=dict(region)
            if region in support_regions:r.setdefault('kind','unsupported_overhang')
            selected=mask & region_mask(pts,normals,r,kind)
            if selected.any():
                waived|=selected;waivers.append({'kind':kind,'region':r,'waived_points':int(selected.sum())})
        clean_masks[kind]=mask&~waived
        rows+=clusters(pts[clean_masks[kind]],values[kind][clean_masks[kind]],kind)
    dims=np.ptp(mesh.vertices,axis=0)
    result={'pass':not rows,'sha256':digest,'samples':len(pts),'random_area_samples':samples,'face_centres':len(mesh.faces),
            'min_wall_mm':2*line_width,'min_gap_mm':1.5*line_width,'minimum_feature_mm':2*line_width,
            'overhang_limit_deg':45,'bed_layer_tolerance_mm':.21,'cluster_grid_mm':2.,
            'bounds_print_mm':mesh.bounds.tolist(),'dimensions_mm':dims.tolist(),
            'raw_flag_points':{k:int(v.sum()) for k,v in raw_masks.items()},
            'unwaived_flag_points':{k:int(v.sum()) for k,v in clean_masks.items()},
            'unwaived_clusters':rows,'applied_exemptions':waivers,
            'method_limits':'Normal-direction distance and sampled overhang screening; an edge can be flagged by a short chord. Local waivers must explain it. No slicer or material certification.'}
    return result,(mesh,pts,raw_masks,clean_masks)


def plot(path,data):
    config=Path(__file__).resolve().parents[2]/'.local/cad-env/.cache/matplotlib'
    config.mkdir(parents=True,exist_ok=True);os.environ.setdefault('MPLCONFIGDIR',str(config))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    mesh,pts,raw,clean=data
    fig,axes=plt.subplots(1,3,figsize=(16,5))
    for ax,(i,j,name) in zip(axes,((0,1,'XY'),(0,2,'XZ'),(1,2,'YZ'))):
        v=mesh.vertices;ax.scatter(v[:,i],v[:,j],s=.2,color='#bbbbbb',rasterized=True)
        for kind,col in zip(KINDS,('#d62728','#176be8','#d88900')):
            p=pts[clean[kind]];ax.scatter(p[:,i],p[:,j],s=2,color=col,label=kind,rasterized=True)
        waived=np.logical_or.reduce([raw[k]&~clean[k] for k in KINDS]);p=pts[waived]
        ax.scatter(p[:,i],p[:,j],s=1,color='#6f9854',label='documented waiver/support',rasterized=True)
        ax.set_aspect('equal');ax.set_title(name);ax.set_xlabel('mm');ax.set_ylabel('mm')
    axes[-1].legend(loc='best',fontsize=6)
    fig.suptitle(Path(path).stem+' — exported PRINT axes; red wall / blue gap / orange unsupported')
    fig.tight_layout();fig.savefig(path,dpi=100);plt.close(fig)


def run(out,manifest,samples=60000,plots=True):
    out=Path(out);dest=out/'printability';dest.mkdir(parents=True,exist_ok=True)
    regions=manifest.get('printability_exemptions',[]);supports=manifest.get('support_regions',[])
    line=float(manifest.get('parameters',{}).get('printing',{}).get('line_width_mm',.42))
    results={}
    for part in manifest['parts']:
        if not part.get('is_printed'):continue
        name=part['name'];path=out/part.get('print_file',f'print/{name}.stl')
        r=probe(path,line_width=line,samples=samples,
                exemptions=[x for x in regions if x.get('part')==name],
                support_regions=[x for x in supports if x.get('part')==name],
                cache_dir=Path(__file__).resolve().parents[2]/'.local/cad-env/.cache/fixture-printability')
        if isinstance(r,tuple):
            result,data=r
            if plots:plot(dest/f'{name}_printability.png',data)
        else:result=r
        results[name]=result
        print(f"Printability {name}: {'PASS' if result['pass'] else 'FAIL'} {result.get('unwaived_flag_points',{})}",flush=True)
    result={'pass':bool(results) and all(r['pass'] for r in results.values()),'parts':results,
            'part_count':len(results),'line_width_mm':line,
            'all_unwaived_flags_are_failures':True,'sub_three_point_clusters_retained':True,
            'slicer_cross_check':{'executed':False,'reason':'Not yet executed; normal rays do not replace slicer inspection'}}
    (dest/'printability.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--samples',type=int,default=60000)
    ap.add_argument('--no-plots',action='store_true');a=ap.parse_args()
    result=run(a.out,json.loads((a.out/'manifest.json').read_text()),a.samples,not a.no_plots)
    raise SystemExit(0 if result['pass'] else 1)
