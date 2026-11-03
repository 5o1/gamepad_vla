#!/usr/bin/env python3
"""V3 actual meshes, regular collision boxes, full-visual camera projection.

Runtime assets use workspace://assets/derived/gamepad_fixture_v3, matching
repository asset management. No collection default or live service is changed.
"""
from __future__ import annotations
import argparse,itertools,json,math,shutil
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from asset_geometry import ControllerAsset
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]

def pose(T):
    q=Rotation.from_matrix(T[:3,:3]).as_quat()
    if q[3]<0:q=-q
    return (T[:3,3]/1000).tolist()+[q[3],*q[:3]]

def collision_boxes(mesh,pitch=2.):
    """Ray-column decomposition, retaining all regular plane coordinates.

    Axis-aligned plane boundaries are exact, including contact seats and lips.
    Curved edges become <=pitch XY stair steps. Z bounds use actual ray hits.
    Adjacent equal intervals are merged; no hull bridges either grip space.
    """
    grid=[]
    for axis in (0,1):
        lo,hi=mesh.bounds[:,axis];regular=np.arange(lo,hi,pitch)
        normal=abs(mesh.face_normals[:,axis]);fi=np.where(normal>.999999)[0]
        planes=mesh.triangles_center[fi,axis]
        vals=np.r_[regular,planes,hi];vals=np.unique(np.round(vals,5));vals=vals[np.r_[True,np.diff(vals)>.03]]
        grid.append(vals)
    xs,ys=grid;orig=np.array([[(a+b)/2,(c+d)/2,mesh.bounds[1,2]+2] for a,b in zip(xs[:-1],xs[1:]) for c,d in zip(ys[:-1],ys[1:])])
    hit,ids,_=mesh.ray.intersects_location(orig,np.tile([0,0,-1.],(len(orig),1)),multiple_hits=True)
    cells={};ny=len(ys)-1
    for idx in range(len(orig)):
        zs=np.unique(np.round(hit[ids==idx,2],4))
        if len(zs)%2:raise RuntimeError('Odd ray parity in collision conversion')
        for z0,z1 in zip(zs[::2],zs[1::2]):
            if z1-z0<.05:continue
            cells.setdefault((float(z0),float(z1)),set()).add((idx//ny,idx%ny))
    result=[]
    for (z0,z1),remaining in cells.items():
        while remaining:
            a,b=min(remaining);a1=a+1;b1=b+1
            while (a1,b) in remaining:a1+=1
            while all((x,b1) in remaining for x in range(a,a1)):b1+=1
            for x in range(a,a1):
                for y in range(b,b1):remaining.remove((x,y))
            result.append([[float(xs[a]),float(ys[b]),z0],[float(xs[a1]),float(ys[b1]),z1]])
    return result

def distance_to_boxes(points,boxes):
    d=np.full(len(points),np.inf)
    for lo,hi in boxes:
        d=np.minimum(d,np.linalg.norm(np.maximum(np.maximum(np.asarray(lo)-points,points-np.asarray(hi)),0),axis=1))
    return d

def project(out,destination):
    mf=json.loads((out/'manifest.json').read_text());scene=json.loads((ROOT/mf['source_scene']).read_text());cam=next(c for c in scene['cameras'] if c['name']=='camera_3')
    q=cam['pose_optical'];C=np.eye(4);C[:3,:3]=Rotation.from_quat(q[4:]+q[3:4]).as_matrix();C[:3,3]=np.asarray(q[:3])*1000
    T=np.linalg.inv(C)@np.asarray(mf['world_from_pad']);k=cam['intrinsics'];asset=ControllerAsset();neutral=asset.visual_meshes()
    links={n:l for n,l,*_ in asset.geometry};alluv=[];cases=[];worst={name:(float('inf'),None) for name in ('left','right','top','bottom')}
    for name,m in neutral.items():
        chain=asset.ancestors(links.get(name,'controller_body'))
        # Each part depends only on its own ancestor joints. All combinations
        # of extremes and intermediate samples cover coupled stick axes too.
        grids=[np.linspace(*j['limits'],9 if j['type'] in ('revolute','continuous') else 2) for j in chain]
        states=list(itertools.product(*grids)) if grids else [()]
        for values in states:
            state={j['name']:float(v) for j,v in zip(chain,values)};M=asset.link_transform(links[name],state)@np.linalg.inv(asset.link_transform(links[name])) if name in links else np.eye(4)
            v=trimesh.transform_points(m.vertices,T@M)
            if v[:,2].min()<=0:raise RuntimeError('controller behind optical camera')
            uv=v[:,:2]/v[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']];alluv.append(uv)
            margins=[uv[:,0].min(),k['width']-1-uv[:,0].max(),uv[:,1].min(),k['height']-1-uv[:,1].max()]
            row=dict(part=name,joint_state=state,vertices=len(v),margins_px=dict(zip(worst,map(float,margins))))
            for key,val in row['margins_px'].items():
                if val<worst[key][0]:worst[key]=(val,row)
            cases.append(row)
    uv=np.vstack(alluv);margins={key:float(v[0]) for key,v in worst.items()}
    endpoints={key:{j['name']:float(j['limits'][index]) for j in asset.joints.values()} for key,index in [('lower',0),('upper',1)]}
    result=dict(all_joint_endpoints=endpoints,executed=True,source='Complete upstream visual glTF, all vertices; appearance scale/rotation + URDF link FK',
        state_cases=len(cases),projected_vertices=len(uv),image_size=[k['width'],k['height']],required_margin_px=20.,
        bounds_px=[uv.min(0).tolist(),uv.max(0).tolist()],margins_px=margins,worst_cases={key:v[1] for key,v in worst.items()},
        pass_all_edges=all(v>=20 for v in margins.values()),notes='Includes joint limits and 9-point revolute grids; shell, both sticks, shoulders, triggers, digital controls, Pair and Xbox. Projection, not occlusion certification.')
    destination.mkdir(parents=True,exist_ok=True);(destination/'projection.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

def export(out,destination):
    mf=json.loads((out/'manifest.json').read_text());scene=json.loads((ROOT/mf['source_scene']).read_text())
    scene['name']='station-g20-v3-fixture-planar-fork';scene['gamepad']['root_pose']=mf['derived_root_pose'];scene['gamepad']['release']='xbox_gamepad_physics==0.4.1'
    scene['fixtures']=[f for f in scene['fixtures'] if f['name'] not in ('gamepad_support_post','gamepad_clamp_base','station_table_frame_panel')]
    installation=mf['parameters']['installation'];thick=installation['table_thickness']/1000
    top=installation['tabletop_z']/1000;scene['table']['pose'][2]=top-thick/2;scene['table']['half_extents'][2]=thick/2
    apron_x=(installation['table_edge_x']-installation['table_edge_to_apron'])/1000
    scene['fixtures'].append(dict(name='v3_table_apron',type='box',pose=[apron_x-.005,installation['table_center_y']/1000,top-thick-.06,1,0,0,0],
        half_extents=[.005,.625,.06],collision=True,status='assumed',source='V3 edge-to-apron default 30 mm, table thickness25 mm; apron thickness10/drop120 provisional, caliper required'))
    # Appearance overrides only proxies and fixture materials, preserving all
    # source station camera/light/robot/table rules.
    appearance=json.loads((ROOT/'configs/appearance/station_v2.json').read_text());appearance['name']='station-v3-fixture'
    rules=[dict(name='v3_collision_invisible',match=dict(link_glob=['v3_proxy_*'],mesh_glob=[]),base_color=[.2,.2,.2,0.],metallic=0.,roughness=.7,status='design',source='Collision-only box proxies; actual appearance uses the v3 CAD meshes')]
    assets=ROOT/'assets/derived/gamepad_fixture_v3';assets.mkdir(parents=True,exist_ok=True)
    W=np.asarray(mf['world_from_pad']);deviations={};count=0
    for row in mf['parts']:
        name=row['name'];mesh=trimesh.load(out/'assembly_mesh'/(name+'.stl'),force='mesh')
        mesh_m=mesh.copy();mesh_m.apply_scale(.001);mesh_m.export(assets/(name+'.glb'))
        visualname='v3_visual_'+name
        scene['fixtures'].append(dict(name=visualname,type='mesh',pose=mf['derived_root_pose'],mesh=f'workspace://assets/derived/gamepad_fixture_v3/{name}.glb',collision=False,status='design',source='fixture.py v3 production CAD; controller local metres'))
        rgb=[.035,.21,.26,1.] if row['material']=='PETG' else [.28,.14,.34,1.] if name.startswith('pad_') else [.38,.43,.48,1.]
        rules.append(dict(name='v3_mat_'+name,match=dict(link_glob=[visualname],mesh_glob=[]),base_color=rgb,metallic=.6 if row['material'] in ('Steel','Spring_steel') else 0.,roughness=.7,status='design',source='V3 material palette; source CAD part '+name))
        # Springs/fasteners are small procurement references inside structural
        # envelopes; collision covers structure and installed soft contacts.
        if name.startswith('hardware_'):continue
        local=mesh.copy();T=np.eye(4)
        if row['fixed_in_base_frame']:
            F=np.eye(4);F[:3,:3]=mf['fixed_frame_world_axes'];F[:3,3]=mf['fixed_frame_world_origin_mm'];T=np.linalg.inv(F)@W;local.apply_transform(T)
        proxy_source=local
        if name.startswith('hirth_'):
            # Preserve the actual central plate and outer tooth-annulus
            # steps. Only individual teeth become a smooth annular envelope.
            import manifold3d as man
            lo,hi=local.bounds;centre=local.bounds.mean(0);ri=installation['tooth_inner_radius'] if 'tooth_inner_radius' in installation else mf['parameters']['installation']['tooth_inner_radius']
            spans=[(4.3,34.,-21.,-17.),(ri,30.,-17.,float(hi[0]))] if name=='hirth_fixed' else [(4.3,34.,-12.,-9.),(ri,30.,float(lo[0]),-12.)]
            pieces=[]
            for rin,rout,x0,x1 in spans:
                q=trimesh.creation.annulus(r_min=rin,r_max=rout,height=x1-x0,sections=96)
                q.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[0,1,0]));q.apply_translation([(x0+x1)/2,centre[1],centre[2]])
                pieces.append(man.Manifold(man.Mesh(np.asarray(q.vertices,np.float32),np.asarray(q.faces,np.uint32))))
            union=man.Manifold.batch_boolean(pieces,man.OpType.Add).to_mesh();proxy_source=trimesh.Trimesh(np.asarray(union.vert_properties)[:,:3],np.asarray(union.tri_verts),process=True)
        override=mf.get('collision_overrides',{}).get(name,{})
        if override.get('residual_mesh'):proxy_source=trimesh.load(out/override['residual_mesh'],force='mesh')
        boxes=[] if override.get('boxes') and not override.get('residual_mesh') else collision_boxes(proxy_source)
        records=[dict(bounds_mm=b,transform=np.eye(4)) for b in boxes]
        records += [dict(bounds_mm=b['bounds_mm'],transform=np.asarray(b['pad_from_box_frame'])) for b in override.get('boxes',[])]
        for record in records:
            lo,hi=np.asarray(record['bounds_mm']);Q=np.asarray(record['transform']).copy();Q[:3,3]+=Q[:3,:3]@((lo+hi)/2)
            Q=(W@np.linalg.inv(T))@Q
            scene['fixtures'].append(dict(name=f'v3_proxy_{name}_{count:04d}',type='box',pose=pose(Q),half_extents=((hi-lo)/2000).tolist(),collision=True,status='design',source='Ray-column boxes from actual v3 mesh; regular contact planes retained'))
            count+=1
        # Union boundary avoids mistaking internal box-to-box faces for
        # external geometry. Sampling every subdivided vertex with max edge
        # 2 mm gives a conservative continuous bound by 1-Lipschitz distance.
        import manifold3d as man
        solids=[]
        for rec in records:
            lo,hi=np.asarray(rec['bounds_mm']);center=(lo+hi)/2
            B=np.asarray(rec['transform']).copy();B[:3,3]+=B[:3,:3]@center
            q=trimesh.creation.box(hi-lo,transform=B)
            solids.append(man.Manifold(man.Mesh(np.asarray(q.vertices,np.float32),np.asarray(q.faces,np.uint32))))
        union=man.Manifold.batch_boolean(solids,man.OpType.Add).to_mesh()
        proxy=trimesh.Trimesh(np.asarray(union.vert_properties)[:,:3],np.asarray(union.tri_verts),process=True)
        cover=2.
        def dense_vertices(source):
            vertices,faces=trimesh.remesh.subdivide_to_size(source.vertices,source.faces,max_edge=cover,max_iter=12)
            return vertices
        def maximum_distance(points,target):
            maximum=0.
            for start in range(0,len(points),2000):
                _,dist,_=trimesh.proximity.closest_point(target,points[start:start+2000]);maximum=max(maximum,float(dist.max(initial=0)))
            return maximum
        source_vertices=dense_vertices(local);proxy_vertices=dense_vertices(proxy)
        a=maximum_distance(source_vertices,proxy);b=maximum_distance(proxy_vertices,local)
        deviations[name]=dict(boxes=len(records),mesh_to_proxy_sample_max_mm=a,proxy_to_mesh_sample_max_mm=b,
            sampled_hausdorff_mm=max(a,b),continuous_deviation_upper_bound_mm=max(a,b)+cover,
            surface_cover_radius_bound_mm=cover,source_samples=len(source_vertices),proxy_samples=len(proxy_vertices),
            method='Exact box-union boundary; both triangle surfaces subdivided to max edge2mm, all vertices tested. Distance is 1-Lipschitz, so sample maximum+2mm bounds the continuous maximum.',
            exact_planar_contact_boxes=bool(override.get('boxes')),xy_step_limit_mm=2.)
    appearance['materials']=rules+appearance['materials']
    ap=ROOT/'configs/appearance/station_v3_fixture.json';ap.write_text(json.dumps(appearance,indent=2)+'\n');scene['appearance']='../../appearance/station_v3_fixture.json'
    # placement_trials lives one level below workcell; existing X's inherited
    # ../appearance path would resolve incorrectly here, so set it explicitly.
    scene['notes']+=['V3: actual CAD visual meshes plus collision-only box decomposition. Root is derived from compressed-pad assembly, not prescribed d050-h220.',
        'V3 vertical post axis from integer base/flange/post dimensions. Camera_3 calibration unchanged. This is a trial spec only; no service/default switch.']
    spec=ROOT/'configs/workcell/placement_trials/gamepad_fixture_v3.json';spec.write_text(json.dumps(scene,indent=2)+'\n')
    (assets/'provenance.json').write_text(json.dumps(dict(generator='hardware/gamepad_fixture/scene_export.py',cad_generator='hardware/gamepad_fixture/fixture.py',revision=mf['design_revision'],units='m',source_out='hardware/gamepad_fixture/out',spec=str(spec.relative_to(ROOT))),indent=2)+'\n')
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'collision_approximation.json').write_text(json.dumps(dict(executed=True,box_count=count,parts=deviations,max_sampled_deviation_mm=max(r['sampled_hausdorff_mm'] for r in deviations.values()),continuous_maximum_upper_bound_mm=max(r['continuous_deviation_upper_bound_mm'] for r in deviations.values()),certified_continuous_maximum=False,certified_conservative_upper_bound=True),indent=2)+'\n')
    project(out,destination)
    print(spec,count,flush=True)
    return spec

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=HERE/'out');p.add_argument('--evidence',type=Path,default=ROOT/'docs/reports/2026-11-04-gamepad-fixture-v3');a=p.parse_args();export(a.out,a.evidence)
