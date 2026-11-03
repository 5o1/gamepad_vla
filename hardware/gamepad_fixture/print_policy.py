"""Explicit V3 printing/support instructions and bounded ray-screen waivers.

This is a manufacturing plan, not a slicer result. Supports must be painted in
these named regions before printing. No unsupported-overhang flag outside the
plan is accepted, and no thin-wall/gap is auto-waived from a scan result.
"""
from __future__ import annotations
import itertools
import numpy as np
import trimesh


def _bounds(bounds,transform):
    b=np.asarray(bounds,float);corners=np.array(list(itertools.product(*zip(b[0],b[1]))))
    v=trimesh.transform_points(corners,transform);return [v.min(0).tolist(),v.max(0).tolist()]


def _hirth_regions(manifest,name,part):
    tf=np.asarray(part['print_transform']);pivot=np.asarray(manifest['pivot_pad_mm'])
    centre=trimesh.transform_points([pivot],tf)[0];b=np.asarray(part['print_bounds_mm']);i=manifest['parameters']['installation'];rows=[]
    edge_width=2*manifest['parameters']['printing']['line_width_mm']
    for radius in (i['tooth_inner_radius'],i['tooth_outer_radius']):
        interval=[radius-.05,radius+edge_width] if radius==i['tooth_inner_radius'] else [radius-edge_width,radius+.05]
        rows.append(dict(part=name,kind='thin_wall',bounds_print_mm=[[-.01,-.01,-.01],(b+.01).tolist()],
            radial_centre_print_mm=centre.tolist(),radial_axes=[0,1],radial_interval_mm=interval,
            reason='Short normal chord at radial tooth termination edge only, not a separate thin wall. Tip/root flats and loaded middle annulus are checked independently; verify edge fidelity on Hirth coupon before production.'))
    for cut in manifest.get('cut_audit',[]):
        if cut['part']==name and cut['name'] in ('index_groove','alignment_groove'):
            bounds=np.asarray(cut['removed_bounds_pad_mm']);bounds[0]-=.9;bounds[1]+=.9
            rows.append(dict(part=name,kind='thin_wall',bounds_print_mm=_bounds(bounds,tf),
                reason='Open end of engraved alignment groove at circular rim has a short corner chord; groove is >=1.2 mm wide and >=.8 mm deep, no projecting pointer or tiny text.'))
    return rows


def apply_policy(manifest):
    """Called after print_transform is assigned, before manifest serialization."""
    meta={p['name']:p for p in manifest['parts']};exempt=[];supports=[]
    manifest['printability_exemptions']=exempt;manifest['support_regions']=supports
    manifest['printability_junctions']=[]
    manifest['print_support_status']='Required local support plan; not sliced or physically verified'
    if manifest.get('artifact_type'):
        # Sections expose the original production geometry at artificial cut
        # faces. Their small complete bounding region is the named coupon.
        for name,p in meta.items():
            if name in ('guide_rod_segment','clamp_band_block','insert_M5'):continue
            if name in ('hirth_fixed','hirth_moving'):
                exempt.extend(_hirth_regions(manifest,name,p))
                continue
            if name=='marker_bracket':
                reason='Marker neck/anchor underside and keyed slot roof require painted removable support; keep board registration/card face clean.'
                tf=np.asarray(p['print_transform'])
                for label,bounds in (
                    ('neck_anchor',[[-6.1,71.1,-54.25],[6.1,71.65,-53.8]]),
                    ('neck_board',[[-6.1,109.9,-62.35],[6.1,110.85,-61.5]])):
                    exempt.append(dict(part=name,kind='narrow_gap',bounds_print_mm=_bounds(bounds,tf),
                        reason=f'Marker {label}: normal short chord at the open concave intersection of overlapping regular solids, not a separate gap or thin wall; local connected-material evidence is recorded.'))
                    centre=np.asarray(bounds).mean(0);probe=[(centre-np.array([6.,.6,.6])).tolist(),(centre+np.array([6.,.6,.6])).tolist()]
                    manifest['printability_junctions'].append(dict(part=name,name=label,bounds_print_mm=_bounds(probe,tf)))
            else:reason='Production-profile coupon: paint removable support only under downward jaw ledges and horizontal bore roofs; match production orientation, clean before fit testing.'
            b=p.get('print_bounds_mm',p.get('bounds_mm'))
            if b:supports.append(dict(part=name,kind='unsupported_overhang',bounds_print_mm=[[-.01,-.01,-.01],(np.asarray(b)+.01).tolist()],reason=reason))
        return manifest
    p=manifest['parameters'];i=p['installation'];m=p['measured'];pivot=np.asarray(manifest['pivot_pad_mm']);W=np.asarray(manifest['world_from_pad']);F=np.eye(4)
    F[:3,:3]=manifest['fixed_frame_world_axes'];F[:3,3]=manifest['fixed_frame_world_origin_mm'];pad_from_base=np.linalg.inv(W)@F
    def support(name,bounds,reason,base=False):
        if name not in meta:return
        tf=np.asarray(meta[name]['print_transform'])
        if base:tf=tf@pad_from_base
        supports.append(dict(part=name,kind='unsupported_overhang',bounds_print_mm=_bounds(bounds,tf),reason=reason))
    cy=i['column_center_y'];axis=manifest['pivot_base_mm'][2]
    for sign in (-1,1):
        xlo,xhi=(-29.1,-20.9) if sign<0 else (20.9,29.1)
        support('column',[[xlo,cy-34.1,axis-34.1],[xhi,cy+34.1,axis+34.1]],
            'Paint removable support under circular fork head, horizontal axis hole roof and M3/nut pocket roofs on this ear; detach before metal-axis fit.',True)
        xlo,xhi=(-29.1,-17.9) if sign<0 else (17.9,29.1)
        support('column',[[xlo,cy-16.1,axis-65.1],[xhi,cy+16.1,axis-35.9]],
            'Paint removable support under the widened fork cross-member lateral ledge; upright and gussets remain self supporting.',True)
    uw,lw,rw=[m[k]/2 for k in ('upper_jaw_width','lower_jaw_width','rear_support_width')]
    ylo=m['top_center_y']-m['center_span'];by0=m['rear_center_y']-m['rear_support_height']/2
    support('cradle_lower_jaw',[[-max(lw,m.get('lateral_stop_half_gap',32.75))-.25,min(-32,ylo-8),-31],[max(lw,m.get('lateral_stop_half_gap',32.75))+.25,max(-16,ylo+4),i['jaw_front_end_z']+1]],
            'Side-down print: support lower-jaw lateral steps, front lip and guide-socket underside; keep planar contact seat free from support.')
    support('cradle_lower_jaw',[[-max(31,rw+1),-32,-31],[max(31,rw+1),76,-8]],
            'Side-down print: support frame, rear block and cap-insert roofs; remove support before installing pads/inserts.')
    support('cradle_lower_jaw',[[pivot[0]-10,pivot[1]-33,pivot[2]-33],[pivot[0]+10,max(-19,pivot[1]+33),-23]],
            'Side-down print: support centered pivot hub/web and horizontal M8/M3 bore ceilings; metal bolt, not a printed hinge pin.')
    support('cradle_lower_jaw',[[-19,-32,-50],[19,0,-23]],
            'Side-down print: support lower rail seats and mechanical-stop collar underside; ream/coupon-check guide fit after cleanup.')
    upper=p['measured']['top_center_y']
    support('slider_upper_jaw',[[-uw-1,24,-36],[uw+1,upper+7.1,i['jaw_front_end_z']+1]],
            'Jaw-down print: support neck step and Pair/USB channel roofs; protect planar pad seats from support scars.')
    support('slider_upper_jaw',[[-19,-8.1,-51],[19,32.1,-33]],
            'Jaw-down print: support slider guide-bore/set-screw roofs and rear-body ledge; clear both guide bores after printing.')
    support('guide_cap',[[-31,30,-51],[31,75.1,-29]],
            'Cap-down print: support annular stop-tube/bore roofs and cap notch underside; clear sockets and screw holes after printing.')
    # Rotating the rear block makes its lower face meet the frame front at a
    # slightly non-orthogonal open concave corner. A normal ray from a point
    # within a few micrometres of that junction crosses the other face. That
    # is not a slot width; both faces belong to one overlapping solid.
    beta=np.arctan(m['rear_plane_slope']);back=m['rear_plane_z'];yc=m['rear_center_y']
    junction_y=yc+(by0-yc)/np.cos(beta)-(-24-back)*np.tan(beta)
    tf=np.asarray(meta['cradle_lower_jaw']['print_transform'])
    for normal in ([0,0,1],[0,-np.cos(beta),-np.sin(beta)]):
        exempt.append(dict(part='cradle_lower_jaw',kind='narrow_gap',
            bounds_print_mm=_bounds([[-rw-.05,junction_y-.12,-24.06],[rw+.05,junction_y+.12,-23.94]],tf),
            normal_print=(tf[:3,:3]@normal).tolist(),normal_tolerance_deg=.2,
            reason='Strictly local open concave rear-block/frame junction, not a separate narrow slot. Tilted block and frame overlap into one solid; checker records the connected local material volume. Contact seats and actual slots are not waived.'))
    # Narrow normal chords at the circular termination of teeth are not a
    # freestanding wall. Confine this exception to <0.84 mm from the end edge;
    # interior tips, roots, relief transitions and gaps remain hard checked.
    for name in ('hirth_fixed','hirth_moving'):
        exempt.extend(_hirth_regions(manifest,name,meta[name]))
    symmetry=manifest.setdefault('symmetry',{});regions=[]
    for cut in manifest.get('cut_audit',[]):
        if (cut['part']=='column' and cut['name'] in ('left_axis','right_sliding_sleeve','fixed_ring_M3','fixed_ring_nut')) or (cut['part']=='cradle_lower_jaw' and cut['name'] in ('moving_ring_M3','moving_ring_nut')):
            b=np.asarray(cut['removed_bounds_pad_mm']);b[0]-=.03;b[1]+=.03
            reason='Functional single-side fastener/axis fit: left toothed-ring locator versus right sliding double-shear sleeve; outer structural load path remains mirror symmetric.'
            regions.append(dict(part=cut['part'],bounds_pad_mm=b.tolist(),reason=reason))
            reflected=b.copy();reflected[:,0]=[-b[1,0],-b[0,0]]
            regions.append(dict(part=cut['part'],bounds_pad_mm=reflected.tolist(),reason=reason))
    symmetry['exemption_regions_pad_mm']=regions
    return manifest
