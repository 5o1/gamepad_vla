#!/usr/bin/env python3
"""V3 exported-solid, contact-chain and FDM hard gates (mm, N, N m).

Checks use STL geometry, not only the CAD parameter claims. No physical
manipulation, FDM material or continuously moving-hand success is certified.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import manifold3d as mf
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from asset_geometry import ControllerAsset,SHELL_NAMES
from printability import run as check_printability
HERE=Path(__file__).resolve().parent


def manifold(mesh):
    solid=mf.Manifold(mf.Mesh(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.uint32)))
    if solid.status()!=mf.Error.NoError:raise RuntimeError(f'Invalid triangle manifold: {solid.status()}')
    return solid


def aabb_gap(a,b):
    return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0)))


def overlap(a,b):return abs(float((a^b).volume()))


def boxmesh(bounds):
    b=np.asarray(bounds);return trimesh.creation.box(b[1]-b[0],trimesh.transformations.translation_matrix(b.mean(0)))


def pair_intersections(meshes,solids,tolerance=.05):
    rows=[]
    for a,b in itertools.combinations(meshes,2):
        if aabb_gap(meshes[a].bounds,meshes[b].bounds)>0:continue
        v=overlap(solids[a],solids[b])
        if v>tolerance:rows.append(dict(parts=[a,b],overlap_mm3=v))
    return dict(pass_=not rows,intersections=rows,ignore_below_mm3=tolerance)


def cap_fastener_access(manifest,meshes):
    """Verify cap-insert installation access, not merely absence of overlap."""
    i,h=manifest['parameters']['installation'],manifest['parameters']['hardware'];frame=meshes['cradle_lower_jaw'];cap=meshes['guide_cap']
    rows=[];offsets=[(0.,0.)]+[(r*math.cos(a),r*math.sin(a)) for r in (1.4,1.9) for a in np.linspace(0,2*math.pi,8,endpoint=False)]
    for sign in (-1,1):
        x=sign*i['cap_bolt_x'];y=i['frame_top_y']-4;back=i['frame_back_z'];front=i['frame_front_z']
        origins=np.array([[x+dx,y+dy,back-1] for dx,dy in offsets]);hit,ids,_=frame.ray.intersects_location(origins,np.tile([0,0,1.],(len(origins),1)),multiple_hits=True)
        blocking=[dict(ray=int(j),hit_pad_mm=p.tolist()) for p,j in zip(hit,ids) if back-.01<=p[2]<=front+.01]
        # A nearby line outside the bore measures the actual six-mm frame
        # wall; the bore itself must be continuously open through that wall.
        outside=[[x+sign*2.5,y,back-1]];surface,_,_=frame.ray.intersects_location(outside,[[0,0,1]],multiple_hits=True)
        zs=sorted(float(p[2]) for p in surface if back-.01<=p[2]<=front+.01)
        thickness=zs[-1]-zs[0] if len(zs)>=2 else None
        # The guide cap is twenty mm along this screw axis. Its bore must
        # also pass the nominal three-mm metal shaft without a printed skin.
        cap_origins=np.array([[x+dx,y+dy,float(cap.bounds[0,2])-1] for dx,dy in offsets[:9]])
        cap_hits,cap_ids,_=cap.ray.intersects_location(cap_origins,np.tile([0,0,1.],(len(cap_origins),1)),multiple_hits=True)
        cap_blocking=[dict(ray=int(j),hit_pad_mm=p.tolist()) for p,j in zip(cap_hits,cap_ids) if cap.bounds[0,2]-.01<=p[2]<=cap.bounds[1,2]+.01]
        cap_thickness=float(cap.bounds[1,2]-cap.bounds[0,2]);cap_length=h.get('cap_bolt_length',25.)
        penetration=cap_length-cap_thickness;marker_penetration=30.-(4.+cap_thickness+2.)
        engagement=min(h['cap_insert_length'],penetration);marker_engagement=min(h['cap_insert_length'],marker_penetration)
        rows.append(dict(side=sign,axis_pad_xy_mm=[x,y],frame_probe_rays=len(origins),frame_blocking_hits=blocking,
            actual_frame_thickness_mm=thickness,cap_probe_rays=len(cap_origins),cap_blocking_hits=cap_blocking,
            cap_thickness_mm=cap_thickness,cap_bolt_length_mm=cap_length,cap_insert_engagement_mm=engagement,
            cap_tip_to_frame_front_mm=(thickness-penetration) if thickness is not None else None,
            marker_chain_mm=dict(bolt=30.,anchor=4.,cap=cap_thickness,spacer=2.,penetration=marker_penetration,insert_engagement=marker_engagement),
            pass_=not blocking and not cap_blocking and thickness is not None and thickness-penetration>=.5 and 3<=engagement<=4.01 and 3<=marker_engagement<=4.01))
    return dict(pass_=all(r['pass_'] for r in rows),channels=rows,
        scope='17 axial frame rays including off-axis witnesses must see no closed skin; nine cap rays verify the M3 shaft path. Length chain uses M3x25 or marker M3x30 + 4mm anchor + 2mm spacer; insert retention still requires a physical coupon.')


def contact_chain(manifest,meshes):
    rows=[]
    for p in manifest.get('contact_planes',[]):
        pad=meshes[p['pad']];rigid=meshes[p['rigid_part']];n=np.asarray(p['normal'],float);n/=np.linalg.norm(n)
        fit=float(p['fit_offset_mm']);projection=pad.vertices@n
        front=float(projection.min());back=float(projection.max())
        expected=p['free_thickness_mm']-p['compression_mm']
        front_faces=pad.face_normals@(-n)>.9999
        front_values=pad.triangles_center[front_faces]@n
        plane_error=float(np.max(np.abs(front_values-fit))) if len(front_values) else math.inf
        # Probe actual pad-back points against the actual rigid seat.
        face=pad.face_normals@n>.9999;origins=pad.triangles_center[face]-n*.01
        hit,ids,_=rigid.ray.intersects_location(origins,np.tile(n,(len(origins),1)),multiple_hits=False)
        seat_error=float(np.max(np.abs(hit@n-p['rigid_face_offset_mm']))) if len(hit) else None
        offset_errors=dict(front_surface_to_fit_mm=plane_error,compressed_thickness_error_mm=abs(back-front-expected),
             rigid_seat_offset_error_mm=seat_error,backend_ray_count=len(origins),backend_hit_count=len(set(ids)))
        # Retain actual planar face coordinates; this catches a mesh-derived
        # ridge even if the declared nominal plane remains correct.
        rows.append(dict(name=p['name'],pad=p['pad'],offsets_mm=[front,back],expected_fit_mm=fit,
            expected_seat_mm=p['rigid_face_offset_mm'],**offset_errors,
            declared_groove_depth_mm=p['groove_depth_mm'],
            pass_=plane_error<=.05 and abs(back-front-expected)<=.05 and seat_error is not None and seat_error<=.05 and len(set(ids))==len(origins),
            rim_declared_clearance_mm=p['rim_offset_mm']-fit,
            scope='Compressed pad and rigid seat STL plane coincidence. Actual shell residuals and rigid clearance are checked separately.'))
    pm=manifest['parameters']['measured'];beta=math.atan(pm['rear_plane_slope']);yc=pm['rear_center_y'];back=pm['rear_plane_z']
    back_y_min=yc-pm['rear_support_height']/2
    joint_y=yc+(back_y_min-yc)/math.cos(beta)-(-24-back)*math.tan(beta)
    joint_bounds=[[-20,joint_y-.5,-24.5],[20,joint_y+.5,-23.5]]
    joint=manifold(meshes['cradle_lower_jaw'])^manifold(boxmesh(joint_bounds));jm=joint.to_mesh()
    local=trimesh.Trimesh(np.asarray(jm.vert_properties)[:,:3],np.asarray(jm.tri_verts),process=True)
    components=len(local.split());volume=abs(float(joint.volume()))
    junction=dict(bounds_pad_mm=joint_bounds,local_material_volume_mm3=volume,connected_components=components,
        pass_=volume>10 and components==1,
        scope='Actual exported solid clipped around the waived open concave edge; positive connected material excludes a detached thin fin or two disconnected parts at this junction.')
    return dict(pass_=len(rows)==3 and all(r['pass_'] for r in rows) and junction['pass_'],contacts=rows,rear_frame_junction=junction)


def visual_contact_clearance(manifest,meshes,asset):
    from contact_fit import scan
    visual=asset.visual_shell();rows=[];m,i=manifest['parameters']['measured'],manifest['parameters']['installation']
    uw,lw,rw=[m[k]/2 for k in ('upper_jaw_width','lower_jaw_width','rear_support_width')]
    y0,y1=m['rear_center_y']-m['rear_support_height']/2,m['rear_center_y']+m['rear_support_height']/2
    definitions={'upper':(1,1,[-uw,uw,i['top_pad_z_min'],i['top_pad_z_max']]),
        'lower':(1,-1,[-lw,lw,i['lower_pad_z_min'],i['lower_pad_z_max']]),'back':(2,-1,[-rw,rw,y0,y1])}
    for p in manifest['contact_planes']:
        axis,side,window=definitions[p['name']];pts,total=scan(visual,axis,side,window,.5)
        n=np.asarray(p['normal'],float);rigid=meshes[p['rigid_part']]
        hit,ids,_=rigid.ray.intersects_location(pts+n*.001,np.tile(n,(len(pts),1)),multiple_hits=False)
        distance=(hit-pts[ids])@n
        gap=float(distance.min()) if len(distance) else None
        rows.append(dict(contact=p['name'],visual_samples=len(pts),rigid_ray_hits=len(hit),minimum_normal_clearance_mm=gap,
            pass_=gap is not None and gap>=.25-.001,
            note='Measured against upstream visual triangles, not merely against the fitted plane. Ports/unsupported samples with no fixture hit do not create false rigid contact.'))
    return dict(pass_=all(x['pass_'] for x in rows),contacts=rows,minimum_requested_clearance_mm=.25)


def inspect_cuts(out,manifest,solids):
    rows=[]
    for c in manifest.get('cut_audit',[]):
        path=out/c['removed_mesh'];m=trimesh.load(path,force='mesh');s=manifold(m)
        b=np.asarray(c['allowed_bounds_pad_mm']);outside=np.maximum(b[0]-m.bounds[0],m.bounds[1]-b[1]);bounds_error=float(np.maximum(outside,0).max())
        delta=c['before_volume_mm3']-c['after_volume_mm3'];volume_error=abs(abs(s.volume())-delta)
        unexpected=[]
        for name in c.get('forbidden_parts',[]):
            if name in solids and overlap(s,solids[name])>.05:unexpected.append(name)
        rows.append(dict(part=c['part'],name=c['name'],removed_mesh=c['removed_mesh'],
            measured_removed_volume_mm3=abs(float(s.volume())),before_minus_after_mm3=delta,volume_error_mm3=volume_error,
            allowed_bounds_error_mm=bounds_error,unexpected_parts=unexpected,
            pass_=delta>0 and volume_error<=max(.05,.005*delta) and bounds_error<=.01 and not unexpected))
    # Independent root keepout: intended flange mounting bolts are outside the
    # upright x +/-18. Any cut into its first 25 mm remains a hard failure.
    i=manifest['parameters']['installation'];W=np.asarray(manifest['world_from_pad']);F=np.eye(4);F[:3,:3]=manifest['fixed_frame_world_axes'];F[:3,3]=manifest['fixed_frame_world_origin_mm'];padF=np.linalg.inv(W)@F
    floor=manifest['base_bottom_rigid_plane_above_table_mm']+(i['aluminum_thickness'] if i['aluminum_base'] else i['printed_base_thickness'])
    root=boxmesh([[-i['column_width']/2+.1,i['column_center_y']-i['column_depth']/2+.1,floor+i['flange_thickness']],
                  [i['column_width']/2-.1,i['column_center_y']+i['column_depth']/2-.1,floor+i['flange_thickness']+25]])
    root.apply_transform(padF);rs=manifold(root);root_bad=[]
    for c in manifest.get('cut_audit',[]):
        if c['part']!='column':continue
        m=trimesh.load(out/c['removed_mesh'],force='mesh')
        if aabb_gap(m.bounds,root.bounds)>0:continue
        v=overlap(manifold(m),rs)
        if v>.05:root_bad.append(dict(cut=c['name'],volume_mm3=v))
    return dict(pass_=bool(rows) and all(r['pass_'] for r in rows) and not root_bad,cuts=rows,
                protected_upright_root_cut_intersections=root_bad,
                scope='Removed STL bounds and volume versus per-cut intention, plus independently defined root keepout. Tool-envelope agreement alone cannot certify design intention.')


def structural_symmetry(manifest,meshes,solids):
    reflection=np.diag([-1.,1,1,1]);rows=[]
    regions=manifest.get('symmetry',{}).get('exemption_regions_pad_mm',[])
    for name in manifest.get('symmetry',{}).get('structural_parts',[]):
        if name not in meshes:continue
        m=meshes[name].copy();m.apply_transform(reflection);s=manifold(m)
        diff=(solids[name]-s)+(s-solids[name]);raw=abs(float(diff.volume()))
        waivers=[]
        for r in regions:
            if r.get('part')!=name or not r.get('reason'):continue
            rr=manifold(boxmesh(r['bounds_pad_mm']));v=overlap(diff,rr);diff=diff-rr
            waivers.append(dict(reason=r['reason'],volume_mm3=v,bounds_pad_mm=r['bounds_pad_mm']))
        remaining=abs(float(diff.volume()))
        # Circular holes are independently triangulated by OCCT; their two
        # chord patterns need not be vertex-for-vertex mirrors. Check actual
        # reflected-surface distance as well as retaining the difference solid.
        points=np.vstack((m.vertices,m.triangles_center))
        keep=np.ones(len(points),bool)
        for region in regions:
            if region.get('part')!=name or not region.get('reason'):continue
            b=np.asarray(region['bounds_pad_mm']);keep&=~np.all((points>=b[0])&(points<=b[1]),axis=1)
        points=points[keep];maximum=0.
        for start in range(0,len(points),1000):
            _,distance,_=trimesh.proximity.closest_point(meshes[name],points[start:start+1000])
            maximum=max(maximum,float(distance.max(initial=0)))
        tolerance=float(manifest['parameters']['printing']['mesh_tolerance'])
        rows.append(dict(part=name,raw_asymmetry_volume_mm3=raw,
            unwaived_asymmetry_volume_mm3=remaining,maximum_mirrored_surface_distance_mm=maximum,
            mesh_tessellation_tolerance_mm=tolerance,checked_points=len(points),
            waivers=waivers,pass_=maximum<=tolerance))
    ribs=[]
    W=np.asarray(manifest['world_from_pad']);F=np.eye(4);F[:3,:3]=manifest['fixed_frame_world_axes'];F[:3,3]=manifest['fixed_frame_world_origin_mm'];Fpad=np.linalg.inv(F)@W
    for r in manifest.get('ribs',[]):
        part=meshes[r['part']].copy();part.apply_transform(Fpad);clip=manifold(boxmesh(r['bounds_base_mm']));actual=manifold(part)^clip;v=abs(float(actual.volume()))
        n=np.asarray(r['plane_normal'],float);load=np.asarray(r['load_plane_normal'],float);angle=math.degrees(math.acos(np.clip(abs(n@load)/(np.linalg.norm(n)*np.linalg.norm(load)),-1,1)))
        b=np.asarray(r['bounds_base_mm']);dims=b[1]-b[0];thin_axis=int(np.argmin(dims))
        am=actual.to_mesh();geometry=trimesh.Trimesh(np.asarray(am.vert_properties)[:,:3],np.asarray(am.tri_verts),process=True)
        inertia=geometry.moment_inertia/max(abs(geometry.volume),1e-12)
        covariance=np.eye(3)*np.trace(inertia)/2-inertia
        eigenvalues,eigenvectors=np.linalg.eigh(covariance);actual_normal=eigenvectors[:,0]
        actual_angle=math.degrees(math.acos(np.clip(abs(actual_normal@load)/np.linalg.norm(load),-1,1)))
        flatness=float(eigenvalues[0]/max(eigenvalues[1],1e-12))
        ribs.append(dict(side=r['side'],bounds_base_mm=b.tolist(),actual_material_mm3=v,
            load_plane_mismatch_deg=angle,thin_axis=thin_axis,
            actual_volume_principal_plane_normal_base=actual_normal.tolist(),actual_load_plane_mismatch_deg=actual_angle,
            actual_thickness_to_in_plane_variance_ratio=flatness,
            pass_=v>100 and angle<1 and thin_axis==0 and actual_angle<1 and flatness<.2))
    return dict(pass_=bool(rows) and all(r['pass_'] for r in rows) and len(ribs)==2 and {r['side'] for r in ribs}=={-1,1} and all(r['pass_'] for r in ribs),parts=rows,ribs=ribs)


def mounting(manifest,meshes,solids,out):
    i=manifest['parameters']['installation'];W=np.asarray(manifest['world_from_pad']);F=np.eye(4);F[:3,:3]=manifest['fixed_frame_world_axes'];F[:3,3]=manifest['fixed_frame_world_origin_mm'];Fpad=np.linalg.inv(F)@W
    name='base_aluminum' if i['aluminum_base'] else 'base_printed';base=meshes[name].copy();base.apply_transform(Fpad)
    top=float(base.bounds[1,2]);depth=min(i['clamp_band_depth'],i['table_edge_to_apron']);segments=manifest['clamp_usable_segments_base_x_mm'];rows=[]
    # Test all allowed clamp segments, not only a pair of arbitrary points.
    fixed=[p['name'] for p in manifest['parts'] if p.get('fixed_in_base_frame') and p['name']!=name]
    for lo,hi in segments:
        xs=np.arange(lo+.5,hi,.75);ys=np.arange(-depth+.5,0,1.)
        origins=np.array([[x,y,top+.1] for x in xs for y in ys]);hit,ids,_=base.ray.intersects_location(origins,np.tile([0,0,-1.],(len(origins),1)),multiple_hits=False)
        deviation=float(abs(hit[:,2]-top).max()) if len(hit) else math.inf
        keepout=boxmesh([[lo,-depth,top+.05],[hi,0,top+60]]);keepout.apply_transform(np.linalg.inv(Fpad));s=manifold(keepout);intrusions=[]
        for pn in fixed:
            if aabb_gap(meshes[pn].bounds,keepout.bounds)>0:continue
            v=overlap(s,solids[pn])
            if v>.05:intrusions.append(dict(part=pn,volume_mm3=v))
        rows.append(dict(segment_base_x_mm=[lo,hi],depth_mm=depth,probe_count=len(origins),hit_count=len(set(ids)),
            top_deviation_mm=deviation,intrusions=intrusions,pass_=len(set(ids))==len(origins) and deviation<.02 and not intrusions))
    mods=[]
    for path in (out/'modifiers').glob('*.stl'):
        m=trimesh.load(path,force='mesh');pb=trimesh.load(out/'print'/f'{name}.stl',force='mesh');v=overlap(manifold(m),manifold(pb));expected=manifest['modifier_expected_volumes_mm3'][path.stem]
        mods.append(dict(name=path.stem,covered_mm3=v,expected_mm3=expected,pass_=abs(v-expected)<.5))
    fasteners=[];h=manifest['parameters']['hardware']
    for pn in sorted(n for n in meshes if n.startswith('hardware_base_M5_')):
        bolt=meshes[pn].copy();bolt.apply_transform(Fpad);xy=bolt.bounds[:,:2].mean(0)
        origin=np.array([[*xy,top+.1]]);hit,_,_=base.ray.intersects_location(origin,[[0,0,-1]],multiple_hits=False)
        if not len(hit):
            fasteners.append(dict(part=pn,pass_=False,error='Base bolt has no blind-hole floor'));continue
        floor=float(hit[0,2]);tip=float(bolt.bounds[0,2]);clearance=tip-floor
        probe=np.array([[*xy,top-min(3.,(top-floor)/2)]])
        right,_,_=base.ray.intersects_location(probe,[[1,0,0]],multiple_hits=False)
        left,_,_=base.ray.intersects_location(probe,[[-1,0,0]],multiple_hits=False)
        diameter=float(right[0,0]-left[0,0]) if len(right) and len(left) else None
        expected=h['aluminum_tap_drill_d'] if i['aluminum_base'] else h['base_insert_pilot']
        fasteners.append(dict(part=pn,pilot_diameter_mm=diameter,pilot_depth_mm=top-floor,
            nominal_thread_engagement_mm=top-tip,bolt_tip_to_hole_bottom_mm=clearance,
            remaining_base_floor_mm=floor-float(base.bounds[0,2]),
            pass_=diameter is not None and abs(diameter-expected)<.05 and clearance>=.5 and floor>base.bounds[0,2],
            scope='Actual STL pilot and bolt-tip rays. Nominal M5 thread crests/heat-set insert pull-out strength are not modelled; machining/insert fit requires physical verification.'))
    return dict(pass_=all(r['pass_'] for r in rows) and depth<=i['table_edge_to_apron'] and all(r['pass_'] for r in mods) and len(fasteners)==4 and all(r['pass_'] for r in fasteners),
        segments=rows,free_edge_length_mm=sum(hi-lo for lo,hi in segments),apron_limit_mm=i['table_edge_to_apron'],
        modifiers=mods,base_fastener_geometry=fasteners,
        scope='Unobstructed upper C-arm envelope 60 mm above the allowed edge segments. Clamp contact centre must also be >= its foot radius from either edge; base M5 bolts need positive blind-hole bottom clearance.')


def measured_mechanics(manifest,meshes):
    p=manifest['parameters'];h,i,t=p['hardware'],p['installation'],p['printing']
    # Capacities are recomputed, not accepted from manifest's mechanics table.
    ri,ro,N=i['tooth_inner_radius'],i['tooth_outer_radius'],i['teeth'];r=(ri+ro)/2
    preload=h['pivot_hand_torque_Nm']/(h['pivot_nut_factor']*h['pivot_bolt_d']/1000)
    separating=4*1000/(r*math.tan(math.radians(i['tooth_flank_deg'])))
    area=(ro-ri)*(2*math.pi*ri/N-i['tooth_root_flat'])
    capacity=t['effective_loaded_teeth']*area*t['tooth_shear_allow_MPa']*r/1000
    pads={n:meshes[n] for n in ('pad_upper_jaw','pad_lower_jaw','pad_back')}
    # Actual notched pad areas reduce effective x moments, retained explicitly.
    actual=[]
    for n,m in pads.items():
        normal=np.array(next(x['normal'] for x in manifest['contact_planes'] if x['pad']==n))
        face=m.face_normals@(-normal)>.9999;tri=m.triangles[face];facearea=float(m.area_faces[face].sum())
        first=second=0.
        for triangle in tri:
            area=float(np.linalg.norm(np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]))/2)
            x=triangle[:,0];second+=area*(float(np.sum(x*x))+float(x[0]*x[1]+x[1]*x[2]+x[2]*x[0]))/6
            # Clip each surface triangle against x=0 before integrating |x|.
            for side in (-1,1):
                polygon=[]
                for a,b in zip(triangle,np.roll(triangle,-1,axis=0)):
                    inside_a=side*a[0]>=0;inside_b=side*b[0]>=0
                    if inside_a:polygon.append(a)
                    if inside_a!=inside_b:polygon.append(a+(b-a)*(-a[0]/(b[0]-a[0])))
                for j in range(1,len(polygon)-1):
                    q=np.array([polygon[0],polygon[j],polygon[j+1]])
                    a=float(np.linalg.norm(np.cross(q[1]-q[0],q[2]-q[0]))/2)
                    first+=a*abs(float(q[:,0].mean()))
        mean_radius=first/facearea
        actual.append(dict(pad=n,width_mm=float(np.ptp(m.vertices[:,0])),contact_area_mm2=facearea,
            mean_abs_x_mm=mean_radius,area_second_moment_mm4=second))
    closing=2*h['spring_rate']*(h['spring_free_length']-(i['cap_spring_seat_y']-i['slider_spring_seat_y']))
    contact_radius=sum(r['mean_abs_x_mm'] for r in actual if r['pad']!='pad_back')
    friction=h['pad_friction']*closing*contact_radius/1000
    back=next(r for r in actual if r['pad']=='pad_back');thick=p['measured']['back_pad_thickness']-p['measured']['pad_compression']
    kt=h['pad_effective_E_MPa']/thick*back['area_second_moment_mm4']/1000
    return dict(pass_=preload/separating>=3 and capacity>=4 and friction>=.4,
        pivot_separation_safety_factor=preload/separating,pivot_shear_capacity_Nm=capacity,preload_N=preload,
        anti_twist_friction_capacity_Nm=friction,anti_twist_safety_factor_at_0_4Nm=friction/.4,
        anti_twist_backpad_stiffness_Nm_per_rad=kt,anti_twist_rotation_deg_at_0_4Nm=math.degrees(.4/kt),actual_pad_moments=actual,
        assumptions='Uniform pad pressure, nominal spring preload, dry mu=.6, pad effective E=.6 MPa; bench test required. Back stiffness applies only with continuous preloaded contact.')


def tooth_sections(manifest,meshes,samples=7200):
    """Measure actual mating STLs at three radii; no parameter-only tooth pass."""
    i=manifest['parameters']['installation'];pivot=np.asarray(manifest['pivot_pad_mm']);rows=[]
    line=manifest['parameters']['printing']['line_width_mm']
    angle=np.linspace(0,2*math.pi,samples,endpoint=False)
    for name in ('hirth_fixed','hirth_moving'):
        mesh=meshes[name];fixed=name=='hirth_fixed'
        for radius in (i['tooth_inner_radius']+.5,(i['tooth_inner_radius']+i['tooth_outer_radius'])/2,i['tooth_outer_radius']-.5):
            origins=np.c_[np.full(samples,100. if fixed else -100.),pivot[1]+radius*np.cos(angle),pivot[2]+radius*np.sin(angle)]
            directions=np.tile([-1. if fixed else 1.,0,0],(samples,1))
            hit,ids,faces=mesh.ray.intersects_location(origins,directions,multiple_hits=False)
            profile=np.full(samples,np.nan);profile[ids]=hit[:,0]
            lo,hi=float(np.nanmin(profile)),float(np.nanmax(profile));step=2*math.pi*radius/samples
            # Extrema bands are the truncated tooth tops/roots; count widths
            # per tooth and subtract two sample intervals conservatively.
            def bands(mask):
                starts=np.flatnonzero(mask&~np.roll(mask,1));width=[]
                for s in starts:
                    length=0
                    while length<len(mask) and mask[(s+length)%len(mask)]:length+=1
                    width.append(max(0,length*step-2*step))
                return width
            low=bands(profile<=lo+.003);high=bands(profile>=hi-.003)
            top=high if fixed else low;root=low if fixed else high
            normals=mesh.face_normals[faces];nx=abs(normals[:,0]);loaded=(nx>.3)&(nx<.8)
            angles=np.degrees(np.arccos(np.clip(nx[loaded],0,1)))
            deviation=float(abs(angles-i['tooth_flank_deg']).max(initial=0))
            min_tip=min(top,default=0);min_root=min(root,default=0)
            rows.append(dict(part=name,radius_mm=radius,ray_count=samples,hit_count=len(set(ids)),
                measured_teeth=len(top),tip_flat_min_lower_bound_mm=min_tip,root_flat_min_lower_bound_mm=min_root,
                section_sampling_step_mm=step,flank_angle_max_error_deg=deviation,
                pass_=len(set(ids))==samples and len(top)==i['teeth'] and len(root)==i['teeth'] and min_tip>=2*line and min_root>=1.5*line and deviation<=1.))
    return dict(pass_=all(r['pass_'] for r in rows),sections=rows,
        scope='First-hit ray sections of actual STL; lower bounds subtract two angular sample intervals. Top/root radial-edge short-chord waivers do not waive these interior sections.')


def slider_motion(manifest,meshes,solids):
    nominal=manifest['nominal_span_mm'];lo,hi=manifest['jaw_span_range_mm'];moving={'slider_upper_jaw','pad_upper_jaw','lock_screw_spacer'}
    spans=np.linspace(lo,hi,29);bad=[];backing=[]
    for span in spans:
        delta=span-nominal
        for name in moving&meshes.keys():
            mesh=meshes[name].copy();mesh.apply_translation([0,delta,0]);solid=manifold(mesh)
            for other in meshes.keys()-moving:
                if 'compression_spring' in other:continue
                if aabb_gap(mesh.bounds,meshes[other].bounds)>0:continue
                v=overlap(solid,solids[other])
                if v>.05:bad.append(dict(span_mm=float(span),moving=name,fixed=other,overlap_mm3=v))
        # Screw-end must remain over continuous frame material throughout the
        # stroke, rather than dropping into the guide-frame's upper window.
        y=manifest['parameters']['installation']['lock_center_y']+delta
        origins=np.array([[x,y+dy,-33.] for x,dy in [(0,0)]+[(1.5*math.cos(a),1.5*math.sin(a)) for a in np.linspace(0,2*math.pi,8,endpoint=False)]])
        hit,ids,_=meshes['cradle_lower_jaw'].ray.intersects_location(origins,np.tile([0,0,1.],(len(origins),1)),multiple_hits=False)
        backing.append(dict(span_mm=float(span),supported_rays=len(set(ids)),pass_=len(set(ids))==len(origins) and np.all(hit[:,2]<-23)))
    p=manifest['parameters'];i,h=p['installation'],p['hardware'];lengths=[i['cap_spring_seat_y']-i['slider_spring_seat_y']-(s-nominal) for s in (lo,nominal,hi)]
    margin=min(lengths)-h['spring_solid_height_max']
    return dict(pass_=not bad and all(r['pass_'] for r in backing) and margin>0,sampled_spans_mm=spans.tolist(),
        intersections=bad,locking_backing=backing,spring_lengths_mm=lengths,spring_coil_bind_margin_mm=margin,
        scope='29 actual rigid-STL translations, excluding deforming metal springs; continuous stroke between samples is not a motion proof.')


def _keys(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {('pass' if k=='pass_' else k):_keys(v) for k,v in value.items()}
    if isinstance(value,list):return [_keys(v) for v in value]
    return value


def waiver_junctions(out,manifest):
    meta={p['name']:p for p in manifest['parts']};rows=[]
    for region in manifest.get('printability_junctions',[]):
        part=meta[region['part']];mesh=trimesh.load(out/part.get('print_file',f"print/{part['name']}.stl"),force='mesh')
        local=manifold(mesh)^manifold(boxmesh(region['bounds_print_mm']));tm=local.to_mesh();volume=abs(float(local.volume()))
        m=trimesh.Trimesh(np.asarray(tm.vert_properties)[:,:3],np.asarray(tm.tri_verts),process=True)
        count=len(m.split());rows.append(dict(**region,local_material_volume_mm3=volume,connected_components=count,pass_=volume>1 and count==1))
    return dict(pass_=all(x['pass_'] for x in rows),applicable=bool(rows),junctions=rows,
        scope='Actual print STL clipped at each locally waived ordinary open corner must contain positive, connected material; no independent narrow channel is waived.')


def main(out=HERE/'out',samples=60000,plots=True):
    out=Path(out);m=json.loads((out/'manifest.json').read_text());r=dict(executed=True,completed=True,design_revision=m['design_revision'],units='mm,N,N m,deg')
    parts=[p for p in m['parts'] if p.get('is_printed')];build=[]
    for p in parts:
        mesh=trimesh.load(out/p.get('print_file',f"print/{p['name']}.stl"),force='mesh');dims=np.ptp(mesh.vertices,axis=0);limits=np.array([300,320,325.]);build.append(dict(part=p['name'],dimensions_mm=dims.tolist(),limits_mm=limits.tolist(),watertight=bool(mesh.is_watertight),pass_=bool(np.all(dims<=limits)) and mesh.is_watertight))
    r['build_volume']=dict(pass_=all(x['pass_'] for x in build),printer='Bambu H2D dual-nozzle envelope',parts=build)
    if m.get('printability_junctions'):r['local_waiver_junctions']=waiver_junctions(out,m)
    if not m.get('artifact_type'):
        meshes={p.stem:trimesh.load(p,force='mesh') for p in (out/'assembly_mesh').glob('*.stl')};solids={n:manifold(mesh) for n,mesh in meshes.items()}
        r['exported_mesh_validity']=dict(pass_=all(mesh.is_watertight and mesh.is_winding_consistent for mesh in meshes.values()),parts={n:dict(watertight=bool(mesh.is_watertight),volume_mm3=float(mesh.volume),connected_components=len(mesh.split())) for n,mesh in meshes.items()})
        r['assembly_interference']=pair_intersections(meshes,solids)
        access=cap_fastener_access(m,meshes);r['assembly_interference']['cap_M3_installation_access']=access
        r['assembly_interference']['pass_']=r['assembly_interference']['pass_'] and access['pass_']
        asset=ControllerAsset();controls={}
        for n,sweep in asset.sweep_envelopes(9).items():
            s=manifold(sweep);closest=1000.;near=None;bad=[]
            for pn,mesh in meshes.items():
                if aabb_gap(mesh.bounds,sweep.bounds)<closest:
                    d=float(s.min_gap(solids[pn],closest))
                    if d<closest:closest=d;near=pn
                if aabb_gap(mesh.bounds,sweep.bounds)==0:
                    v=overlap(s,solids[pn])
                    if v>.05:bad.append(dict(part=pn,volume_mm3=v))
            allowance=max(.35,asset.sweep_sampling_bounds(9)[n]);required=max(allowance,2. if n=='pair' else 0.)
            controls[n]=dict(minimum_gap_mm=closest,nearest_part=near,sampling_allowance_mm=allowance,
                required_geometric_gap_mm=required,overlaps=bad,pass_=closest>=required-.001 and not bad)
        r['control_full_travel_clearance']=dict(pass_=all(x['pass_'] for x in controls.values()),controls=controls)
        shell=asset.components(SHELL_NAMES);ss=[manifold(x) for _,x in shell];srows=[]
        for pn,mesh in meshes.items():
            soft=pn.startswith('pad_');volume=sum(overlap(solids[pn],s) for (_,sm),s in zip(shell,ss) if aabb_gap(sm.bounds,mesh.bounds)==0)
            gap=1000.
            for (_,sm),s in zip(shell,ss):
                if aabb_gap(sm.bounds,mesh.bounds)<gap:gap=min(gap,float(solids[pn].min_gap(s,gap)))
            # Soft planar pads span real curved shell residuals. Their exact
            # nominal plane is separately checked; soft overlap isn't reported
            # as a rigid collision or hidden as a perfect surface fit.
            srows.append(dict(part=pn,soft=soft,summed_hull_overlap_mm3=volume,minimum_collision_shell_gap_mm=gap,pass_=soft or volume<=.05))
        r['shell_clearance']=dict(pass_=all(x['pass_'] for x in srows),parts=srows,scope='Collision shell comparison; visual shell residuals retained in contact_fit, soft pad overlap represents compliant contact.')
        r['openings']=[]
        for op in m['openings']:
            om=boxmesh(op['bounds_pad_mm']);os=manifold(om);bad=[]
            for pn,mesh in meshes.items():
                if aabb_gap(mesh.bounds,om.bounds)>0:continue
                v=overlap(solids[pn],os)
                if v>.05:bad.append(dict(part=pn,volume_mm3=v))
            r['openings'].append(dict(name=op['name'],bounds_pad_mm=op['bounds_pad_mm'],intersections=bad,pass_=not bad))
        r['openings']=dict(pass_=all(x['pass_'] for x in r['openings']),ports=r['openings'])
        grip=[]
        for side,lo,hi in [('left',[-80,-60,-48],[-30,3,-12]),('right',[30,-60,-48],[80,3,-12])]:
            gm=boxmesh([lo,hi]);gs=manifold(gm);bad=[]
            for pn,mesh in meshes.items():
                if aabb_gap(mesh.bounds,gm.bounds)>0:continue
                v=overlap(solids[pn],gs)
                if v>.05:bad.append(dict(part=pn,volume_mm3=v))
            grip.append(dict(side=side,intersections=bad,pass_=not bad))
        r['grip_back_keepout']=dict(pass_=all(x['pass_'] for x in grip),sides=grip)
        r['contact_offset_chain']=contact_chain(m,meshes)
        r['visual_contact_clearance']=visual_contact_clearance(m,meshes,asset)
        r['cut_audit']=inspect_cuts(out,m,solids)
        r['structural_symmetry_and_ribs']=structural_symmetry(m,meshes,solids)
        r['mounting_band']=mounting(m,meshes,solids,out)
        r['mechanics']=measured_mechanics(m,meshes)
        r['hirth_actual_sections']=tooth_sections(m,meshes)
        r['slider_motion']=slider_motion(m,meshes,solids)
        W=np.asarray(m['world_from_pad']);i=m['parameters']['installation'];depth=i['table_edge_x']-W[0,3];height=W[2,3]-i['tabletop_z'];region=m['feasible_region']
        base_name='base_aluminum' if i['aluminum_base'] else 'base_printed';worldbase=meshes[base_name].copy();worldbase.apply_transform(W)
        errors=dict(base_far_edge_mm=abs(float(worldbase.bounds[1,0])-i['table_edge_x']),
            base_lateral_midline_mm=abs(float(worldbase.bounds[:,1].mean())-i['table_center_y']),
            base_underside_mm=abs(float(worldbase.bounds[0,2])-i['tabletop_z']-m['base_bottom_rigid_plane_above_table_mm']))
        root=m['derived_root_pose'];root_R=Rotation.from_quat([*root[4:],root[3]]).as_matrix()
        position_error=float(np.linalg.norm(np.asarray(root[:3])*1000-W[:3,3]));angle_error=math.degrees(Rotation.from_matrix(root_R@W[:3,:3].T).magnitude())
        r['derived_pose']=dict(pass_=region['depth_mm'][0]<=depth<=region['depth_mm'][1] and region['height_mm'][0]<=height<=region['height_mm'][1] and max(errors.values())<=.01 and position_error<=.001 and angle_error<=.001,
            depth_from_edge_mm=depth,height_above_table_mm=height,root_world_mm=W[:3,3].tolist(),tilt_deg=m['key_angle_deg'],
            exported_base_registration_errors=errors,scene_root_position_error_mm=position_error,scene_root_rotation_error_deg=angle_error,
            pose_source='Actual base/post/support geometry; exported base edge/underside/midline registration and scene root cross-check; no v1/v2 root equality requirement')
        sim=trimesh.load(out/'simulation/fixture.glb',force='scene');expected=np.array([np.min([mesh.bounds[0] for mesh in meshes.values()],axis=0),np.max([mesh.bounds[1] for mesh in meshes.values()],axis=0)])
        error=float(np.abs(sim.bounds*1000-expected).max());r['simulation_export']=dict(pass_=error<.001 and set(sim.geometry)==set(meshes),units='m',bounds_scale_error_mm=error)
        r['hand_space']=dict(executed=False,scope='Legacy v2 witnesses are not a v3 reach-coverage certification; new scene/reach check is a separate task result.')
    r['printability']=check_printability(out,m,samples=samples,plots=plots)
    r=_keys(r);r['summary']={k+'_pass':v['pass'] for k,v in r.items() if isinstance(v,dict) and 'pass' in v}
    r['summary']['all_hard_gates_pass']=all(r['summary'].values());r['physical_pose_certified']=False;r['full_control_motion_certified']=False
    (out/'checks.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r['summary'],indent=2),flush=True)
    return r['summary']['all_hard_gates_pass']


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=HERE/'out');ap.add_argument('--samples',type=int,default=60000);ap.add_argument('--no-plots',action='store_true');a=ap.parse_args()
    try:
        passed=main(a.out,a.samples,not a.no_plots)
    except Exception as error:
        # A failed current check must not leave a stale previous PASS behind.
        a.out.mkdir(parents=True,exist_ok=True)
        (a.out/'checks.json').write_text(json.dumps(dict(executed=True,completed=False,error=str(error),summary=dict(all_hard_gates_pass=False)),indent=2)+'\n')
        raise
    raise SystemExit(0 if passed else 1)
