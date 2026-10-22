#!/usr/bin/env python3
"""Export the selected K1-a v4.5 config, mesh previews and fixture measurements.

CPU only; no viewer, hardware or collection-station connection. The calibrated
image is a deterministic triangle painter rendering; visibility is not an
acceptance test. Projected support-footprint recommendations use a 1mm raster,
with an explicit additional 2mm raster guard; no optional geometry dependency.
"""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from PIL import Image,ImageDraw
from scipy.ndimage import distance_transform_edt,label

from gamepad_vla.feasibility import CollisionScene,transform
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from k1_height_common import (MeshGeometry,CameraGate,support_point,support_fixture,
    with_support,write_json,SUPPORT_NAME)


def put_pair(scene,pair,pose,opened=False):
    for side in ('left','right'):
        e=pair[side];scene.set_arm(side,e['arm_q'])
        scene.set_hand(side,pose_to_matrix(e['palm_world']),{} if opened else e['resting_hand_q'])
    scene.set_pad(pose,{})
    scene.mj.mj_forward(scene.model,scene.data)


def fixture_box(scene,g):
    m,d=scene.model,scene.data
    xyz=np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)],float)*m.geom_size[g]
    points=xyz@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                    [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return points,faces


def ego_preview(scene,geo,camera,path):
    alltri=[];allcolour=[];alldepth=[]
    colours={'HL':np.array([.20,.52,.83]),'HR':np.array([.88,.51,.22]),
             'AL':np.array([.45,.49,.54]),'AR':np.array([.49,.52,.58]),'pad':np.array([.15,.16,.18])}
    items=[(g,geo.vertices(g),v[1],colours.get(scene.groups[g],np.array([.45,.49,.54]))) for g,v in geo.meshes.items()]
    for g in range(scene.model.ngeom):
        if int(scene.model.geom_type[g])==int(scene.mj.mjtGeom.mjGEOM_BOX):
            name=scene.model.geom(g).name
            if name=='station_floor':continue
            v,f=fixture_box(scene,g)
            colour=np.array([.73,.24,.24]) if name==SUPPORT_NAME else np.array([.31,.45,.40]) if name=='table' else np.array([.66,.68,.70])
            items.append((g,v,f,colour))
    light=np.array([-.3,.4,.85]);light/=np.linalg.norm(light)
    for g,v,f,c in items:
        uv,z=camera.project(v);tri=uv[f];zz=z[f]
        keep=(zz>.001).all(axis=1)
        keep&=(tri[:,:,0].max(axis=1)>=0)&(tri[:,:,0].min(axis=1)<camera.k.width)
        keep&=(tri[:,:,1].max(axis=1)>=0)&(tri[:,:,1].min(axis=1)<camera.k.height)
        if not keep.any():continue
        ft=f[keep];tv=v[ft]
        normal=np.cross(tv[:,1]-tv[:,0],tv[:,2]-tv[:,0]);length=np.linalg.norm(normal,axis=1)
        normal/=np.maximum(length[:,None],1e-20)
        brightness=.58+.42*np.abs(normal@light)
        alltri.extend(tri[keep]);alldepth.extend(zz[keep].mean(axis=1));allcolour.extend(np.clip(c[None]*brightness[:,None],0,1))
    order=np.argsort(-np.asarray(alldepth),kind='stable')
    fig=plt.figure(figsize=(12.8,7.2),dpi=100);ax=fig.add_axes([0,0,1,1])
    ax.set_facecolor('#eef0f1');ax.add_collection(PolyCollection(np.asarray(alltri)[order],facecolors=np.asarray(allcolour)[order],edgecolors='none',antialiased=False))
    ax.set_xlim(0,camera.k.width);ax.set_ylim(camera.k.height,0);ax.set_aspect('equal');ax.axis('off')
    ax.add_patch(plt.Rectangle((40,40),camera.k.width-81,camera.k.height-81,fill=False,edgecolor='#31b878',linewidth=1,linestyle='--'))
    ax.plot(camera.k.cx,camera.k.cy,'+',color='#e1bd39',markersize=10)
    centre,_=camera.project(geo.centroid('pad')[None]);ax.plot(*centre[0],'o',color='#e1bd39',markersize=3)
    support_geom=scene.model.geom(SUPPORT_NAME).id
    support_vertices,support_faces=fixture_box(scene,support_geom)
    support_uv,_=camera.project(support_vertices)
    from scipy.spatial import ConvexHull
    hull=ConvexHull(support_uv)
    contour=support_uv[np.r_[hull.vertices,hull.vertices[0]]]
    ax.plot(contour[:,0],contour[:,1],color='#ed5353',linewidth=1.2,linestyle='--')
    anchor=support_uv.mean(axis=0)
    ax.annotate('Support column: projected outline\n(under the shell; occlusion not assessed)',xy=anchor,xytext=(40,670),
                color='#ad2828',fontsize=8,arrowprops=dict(arrowstyle='-',color='#ad2828',lw=.7))
    fig.savefig(path,dpi=100);plt.close(fig)


def side_preview(scene,geo,features,pair,pose,path):
    fig,ax=plt.subplots(figsize=(10,6),dpi=140)
    for group,c in [('AL','#999fa8'),('AR','#bbc0c6'),('HL','#3685c2'),('HR','#db8e3d'),('pad','#25272a')]:
        for g in geo.groups[group]:
            v=geo.vertices(g);faces=geo.meshes[g][1]
            # True triangle projection in x/z; filled for a readable geometric side view.
            ax.add_collection(PolyCollection(v[faces][:,:,[0,2]],facecolor=c,edgecolor='none',alpha=.12 if group in ('AL','AR') else .32))
    top=scene.spec.table.pose[2]+scene.spec.table.half_extents[2]
    ax.axhline(top,color='#4f7664',lw=3,label='Table')
    f=support_fixture(scene.spec,pose,support_point())
    column_vertices,_=fixture_box(scene,scene.model.geom(SUPPORT_NAME).id)
    ax.add_patch(plt.Rectangle((column_vertices[:,0].min(),top),np.ptp(column_vertices[:,0]),2*f.half_extents[2],color='#bc4b44',alpha=.8))
    for side,c,shift in [('left','#1664a5',.015),('right','#bc6718',-.035)]:
        m=features[side].measure(np.array([pair[side]['arm_q']]),np.array([pose[:3]]))
        points=[m[k][0] for k in ('shoulder','elbow','wrist')]
        points=np.array(points);ax.plot(points[:,0],points[:,2],'-o',color=c,lw=2,label=side)
        n=pair[side]['natural']
        up=points[1]-points[0];fore=points[2]-points[1]
        projupper=np.degrees(np.arctan2(up[0],-up[2]));projfore=np.degrees(np.arctan2(fore[2],fore[0]))
        ax.text(.02,.975 if side=='left' else .875,
            f"{side}: 3D upper {n['upper_tilt_deg']:.1f} deg; fore {n['forearm_pitch_deg']:.1f} deg; elbow {n['elbow_angle_deg']:.1f} deg\nXZ projected upper/fore: {projupper:.1f}/{projfore:.1f} deg",color=c,fontsize=8.5,
            transform=ax.transAxes,va='top',bbox=dict(facecolor='white',edgecolor=c,alpha=.9,pad=4))
    ax.set_xlim(-.04,.75);ax.set_ylim(top-.03,1.5);ax.set_aspect('equal')
    ax.set_xlabel('world x (m), into table');ax.set_ylabel('world z (m)')
    ax.set_title('Selected grasp: true mesh side projection (x-z)');ax.legend(loc='upper right');ax.grid(alpha=.15)
    fig.tight_layout();fig.savefig(path);plt.close(fig)


def footprint(scene,geo,pair,pose,out):
    """Rasterize original projected triangles, including accepted operation endpoints."""
    table=scene.spec.table;lo=np.array(table.pose[:2])-table.half_extents[:2]
    hi=np.array(table.pose[:2])+table.half_extents[:2];step=.001
    shape=np.ceil((hi-lo)/step).astype(int)+1
    mask=Image.new('1',tuple(shape),0);draw=ImageDraw.Draw(mask)
    lowest=np.inf
    states=[None]+pair['accepted_states']
    for state in states:
        put_pair(scene,pair,pose)
        if state:
            side=state['side'];scene.set_hand(side,pose_to_matrix(pair[side]['palm_world']),state['hand_q'])
            scene.set_pad(pose,state['gamepad_q']);scene.mj.mj_forward(scene.model,scene.data)
        for group in ('HL','HR','AL','AR'):
            for g in geo.groups[group]:
                vertices=geo.vertices(g);lowest=min(lowest,float(vertices[:,2].min()))
                xy=(vertices[:,:2]-lo)/step
                for tri in xy[geo.meshes[g][1]]:
                    draw.polygon([tuple(v) for v in tri],fill=1)
    occupied=np.asarray(mask,dtype=bool)
    distance=distance_transform_edt(~occupied)*step
    # Two millimetres explicitly cover triangle-to-pixel boundary rounding.
    allowed=distance>=.022
    labelled,_=label(allowed)
    supportxy=np.array(pose[:2]);idx=np.rint((supportxy-lo)/step).astype(int)
    component=int(labelled[idx[1],idx[0]])
    connected=(labelled==component) if component else np.zeros_like(allowed)
    # Largest symmetric world-axis rectangle centred at the column, within this raster component.
    forbidden=(~connected).astype(int);prefix=np.pad(forbidden,((1,0),(1,0))).cumsum(0).cumsum(1)
    cx,cy=idx;maxrx=min(cx,shape[0]-1-cx);maxry=min(cy,shape[1]-1-cy);best=(0,0,0)
    for rx in range(maxrx+1):
        for ry in range(maxry+1):
            x0,x1=cx-rx,cx+rx+1;y0,y1=cy-ry,cy+ry+1
            count=prefix[y1,x1]-prefix[y0,x1]-prefix[y1,x0]+prefix[y0,x0]
            if count:break
            area=(2*rx+1)*(2*ry+1)
            if area>best[0]:best=(area,rx,ry)
    _,rx,ry=best
    if best[0]:
        rect=[(lo+np.array([cx-rx,cy-ry])*step).tolist(),(lo+np.array([cx+rx,cy+ry])*step).tolist()]
    else:rect=None
    put_pair(scene,pair,pose)
    fixture=support_fixture(scene.spec,pose,support_point())
    contact=transform(support_point(),pose_to_matrix(pose))
    thin_bottom=max(table.pose[2]+table.half_extents[2],min(float(contact[2]),lowest-.010))
    fig,ax=plt.subplots(figsize=(7,6),dpi=130)
    ax.imshow(np.where(occupied,2,np.where(connected,0,1)),origin='lower',extent=(*[lo[0],hi[0]],lo[1],hi[1]),
              cmap=matplotlib.colors.ListedColormap(['#b8dfc2','#ead5cb','#7d8a97']),interpolation='nearest',aspect='equal')
    ax.plot(*supportxy,'r+',markersize=10)
    if rect:
        a,b=np.array(rect);ax.add_patch(plt.Rectangle(a,*(b-a),fill=False,color='#234ac4',linewidth=2))
    ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)')
    ax.set_title('Robot projection + 20 mm keepout (+2 mm raster guard)\nGreen: connected allowable base footprint; blue: centred rectangle')
    fig.tight_layout();fig.savefig(out/'base-footprint.png');plt.close(fig)
    # PNG is a design boundary map, not an additional object in the scene.
    Image.fromarray((connected.astype('uint8')*255)).save(out/'base-allowed-mask.png')
    return dict(support_contact_local_m=support_point().tolist(),support_contact_world_m=contact.tolist(),
        column_pose_world=list(fixture.pose),column_height_m=2*fixture.half_extents[2],
        column_cross_section_local_m=[.020,.024],selected_robot_lowest_world_z_m=lowest,
        thin_section_bottom_world_z_m=thin_bottom,thin_section_height_m=float(contact[2]-thin_bottom),
        potential_widened_section_height_m=float(thin_bottom-(table.pose[2]+table.half_extents[2])),
        footprint_method='original triangle projections, neutral + accepted control endpoints; 1mm raster; 20mm margin + 2mm conservative raster guard; only column-connected table component',
        raster_world_xy_min_m=lo.tolist(),raster_step_m=step,connected_allowed_area_m2=float(connected.sum()*step**2),
        centred_axis_aligned_base_rectangle_world_m=rect,
        centred_base_rectangle_size_m=(np.subtract(rect[1],rect[0]).tolist() if rect else None),
        structural_status='design only; no load, tipping, print strength, clamp, cables or fastener certificate')


def height_chart(scan,out):
    heights=sorted({r['height_mm'] for r in scan['placements']})
    tiers=range(scan['total_required_controls']+1)
    grid=np.zeros((len(tiers),len(heights)),int)
    for r in scan['placements']:
        if r['grip_pairs']:grid[r['max_controls'],heights.index(r['height_mm'])]+=1
    fig,ax=plt.subplots(figsize=(9,4),dpi=140)
    im=ax.imshow(grid,origin='lower',aspect='auto',cmap='Blues',extent=(heights[0]-5,heights[-1]+5,-.5,len(tiers)-.5))
    ax.set_xticks(heights);ax.set_yticks(list(tiers));ax.set_xlabel('Shell minimum clearance above table (mm)')
    ax.set_ylabel('Maximum covered controls (left + right)')
    for y,x in zip(*np.where(grid)):
        ax.text(heights[x],y,str(grid[y,x]),ha='center',va='center',color='#9d281d',fontsize=9)
    fig.colorbar(im,ax=ax,label='Number of positions with a feasible pair (out of 49)')
    fig.tight_layout();fig.savefig(out/'height-coverage.png');plt.close(fig)



def solve_open_home(scene,geo,features,pair,pose):
    """Nearest vertical parking target above the grasp, with full open/default meshes clear."""
    from itertools import product
    from search_gamepad_height import multistart_ik
    offsets=np.arange(0,.120001,.010)
    answers={};measures={}
    centre=np.array(pair['placement']['gamepad_centroid_world_m'])
    for side in ('left','right'):
        targets=np.tile(pose_to_matrix(pair[side]['palm_world']),(len(offsets),1,1))
        targets[:,2,3]+=offsets
        arm,hand=getattr(scene.spec,side+'_arm'),getattr(scene.spec,side+'_hand')
        answers[side]=multistart_ik(arm,hand,targets,2021+(side=='right'))
        measures[side]=[features[side].measure(a[0],np.tile(centre,(len(offsets),1))) for a in answers[side]]
    for oi,offset in enumerate(offsets):
        starts={s:[i for i,a in enumerate(answers[s]) if a[1][oi]] for s in ('left','right')}
        combos=list(product(starts['left'],starts['right']))
        combos.sort(key=lambda ij:(max(measures[s][i]['deviation'][oi] for s,i in zip(('left','right'),ij)),
                                   max(measures[s][i]['wrist_deviation_deg'][oi] for s,i in zip(('left','right'),ij))))
        for indices in combos:
            home={};checks={};valid=True
            for side,i in zip(('left','right'),indices):
                q=answers[side][i][0][oi];palm=features[side].frames(q)['palm'][0]
                home[side]=dict(arm_q=q.tolist(),palm_world=palm.tolist(),arm_start=i,
                    arm_position_error_m=float(answers[side][i][2][oi]),arm_rotation_error_rad=float(answers[side][i][3][oi]),
                    natural={k:bool(v[oi]) if np.asarray(v).dtype.kind=='b' else float(v[oi])
                             for k,v in measures[side][i].items() if np.asarray(v).ndim==1})
                scene.set_arm(side,q)
            for scenario in ('open','configured'):
                for side in ('left','right'):
                    q={} if scenario=='open' else dict(zip(G20_JOINT_NAMES,getattr(scene.spec,side+'_hand').home_q))
                    scene.set_hand(side,np.array(home[side]['palm_world']),q)
                scene.set_pad(pose,{})
                checks[scenario]={}
                for side in ('left','right'):
                    clear,bad,_,_=scene.check(side);gap=geo.table_clearance(side)
                    checks[scenario][side]=dict(clear=clear,violations=bad,table_clearance_m=gap)
                    if not clear or min(gap.values())<.030-1e-10:valid=False;break
                if not valid:break
                distances=scene.pair_clearance()
                checks[scenario]['pair_clearances_m']=distances
                if min(distances.values()) < -1e-6:valid=False;break
            if valid:
                return dict(vertical_offset_m=float(offset),arms=home,checks=checks,
                    method='minimum common upward offset in 0..120mm/10mm; six IK starts, minimum natural deviation among mesh-clear starts; actual FK palms; open and inherited hand home values both checked at >=30mm')
    raise RuntimeError('no collision-free open/configured parking home within 120mm; do not export a success config')


def main():
    from k1_natural_reference import ArmFeatures
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pairs',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config-output',type=Path,required=True)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    scan=json.loads((args.pairs/'scan.json').read_text());height_chart(scan,args.output)
    if not scan['choice']:
        write_json(args.output/'validation.json',dict(stop=scan['stop']));return
    pair=json.loads((args.pairs/'selected_pair.json').read_text());place=pair['placement'];pose=place['pose']
    cfg=load_task(args.task);spec=load_spec(cfg['workcell'])
    scene=CollisionScene(with_support(spec,pose,support_point()),cfg['resource_root'],args.output,force_static_hand_pairs=True)
    geo=MeshGeometry(scene);camera=CameraGate(next(c for c in spec.cameras if c.name=='camera_3'),geo)
    features={s:ArmFeatures(getattr(spec,s+'_arm'),getattr(spec,s+'_hand')) for s in ('left','right')}
    put_pair(scene,pair,pose)
    validation={'grasp':{},'open_home':{},'configured_home':{},'operations':[]}
    for side in ('left','right'):
        ok,bad,grips,_=scene.check(side)
        validation['grasp'][side]=dict(clear=ok,grips=sorted(grips),violations=bad,table_clearance_m=geo.table_clearance(side))
        assert ok and grips=={'ring','pinky'}
    validation['camera']=camera.measure();assert validation['camera']['passed']
    validation['pair_clearances_m']=scene.pair_clearance()
    assert min(validation['pair_clearances_m'].values())>=-1e-6
    for state in pair['accepted_states']:
        if state['side']+':'+state['control'] not in pair['controls']:continue
        put_pair(scene,pair,pose);side=state['side']
        scene.set_hand(side,pose_to_matrix(pair[side]['palm_world']),state['hand_q']);scene.set_pad(pose,state['gamepad_q'])
        goal_side=json.loads((args.pairs.parent/(side+'-world')/'world.json').read_text())
        goal=next(g for g in goal_side['goals'] if g['name']==state['control'])
        ok,bad,grips,_=scene.check(side,goal['part'],goal['finger']);active=scene.last_active_contact
        other='right' if side=='left' else 'left';ook,obad,og,_=scene.check(other)
        gate=camera.measure();gaps={s:geo.table_clearance(s) for s in ('left','right')}
        assert ok and ook and active and grips==og=={'ring','pinky'} and gate['passed']
        assert min(v for d in gaps.values() for v in d.values())>=.030-1e-10
        validation['operations'].append(dict(side=side,control=state['control'],state=state['state'],camera=gate,table_clearance_m=gaps))
    put_pair(scene,pair,pose,opened=True)
    actual_home_palms={s:features[s].frames(np.array([pair[s]['arm_q']]))['palm'][0] for s in ('left','right')}
    for side in ('left','right'):scene.set_hand(side,actual_home_palms[side],{})
    for side in ('left','right'):
        _,bad,_,_=scene.check(side)
        fixture_bad=[b for b in bad if scene.groups[scene.model.geom(b['geom_a']).id]=='fixture' or scene.groups[scene.model.geom(b['geom_b']).id]=='fixture']
        gap=geo.table_clearance(side)
        validation['open_home'][side]=dict(fixture_collisions=fixture_bad,table_clearance_m=gap,arm_q=pair[side]['arm_q'])
    validation['grasp_arm_pose_is_open_home']=all(not d['fixture_collisions'] and min(d['table_clearance_m'].values())>=.030 for d in validation['open_home'].values())
    write_json(args.output/'pre-home-validation.json',validation)
    home=solve_open_home(scene,geo,features,pair,pose)
    validation['parking_home']=home
    # Fixed arm bases and mocap palm roots are another no-DOF pair. Prove
    # separation across the entire searched translation grid, using the same
    # original mesh bounds; dynamic arm/finger pairs were checked natively.
    root_translations=np.array([r['pose'][:3] for r in scan['placements']])-np.array(pose[:3])
    fixed_clearance={}
    for side,letter,other_letter in [('left','L','R'),('right','R','L')]:
        rel=json.loads((args.pairs.parent/(side+'-relative')/'relative.json').read_text())
        arm_geoms=[g for g in geo.groups['A'+other_letter] if scene.model.body_dofnum[scene.model.body_weldid[scene.model.geom_bodyid[g]]]==0]
        hand_geoms=[g for g in geo.groups['H'+letter] if scene.model.body_dofnum[scene.model.body_weldid[scene.model.geom_bodyid[g]]]==0]
        for entry in rel['bank']:
            palm=pose_to_matrix(pose)@pose_to_matrix(entry['palm_in_pad'])
            scene.set_hand(side,palm,entry['baselines'][0]['resting_hand_q']);scene.mj.mj_kinematics(scene.model,scene.data)
            hp=np.concatenate([geo.vertices(g) for g in hand_geoms]);hlo=hp.min(axis=0);hhi=hp.max(axis=0)
            for g in arm_geoms:
                ap=geo.vertices(g);alo=ap.min(axis=0);ahi=ap.max(axis=0)
                separation=np.maximum(0,np.maximum(hlo+root_translations-ahi,alo-hhi-root_translations))
                gap=float(np.linalg.norm(separation,axis=1).min())
                fixed_clearance[f'{side}_palm_{entry["palm_index"]}_opposite_arm_base']=gap
                assert gap>0,'static arm/palm AABB overlaps: a native distance audit is required'
    validation['static_arm_hand_global_distance_lower_bounds_m']=fixed_clearance
    raw=json.loads(Path(cfg['workcell']).read_text());raw['name']='station-g20-v4.5';raw['gamepad']['root_pose']=pose
    f=support_fixture(spec,pose,support_point())
    raw['fixtures'].append(dict(name=f.name,type=f.type,pose=list(f.pose),half_extents=list(f.half_extents),collision=True,status='design',source=f.source))
    for side in ('left','right'):raw[side+'_arm']['home_q']=home['arms'][side]['arm_q']
    raw['notes']=['v4.4 provenance: '+note for note in raw['notes']]
    raw['notes'].append(f"Open/default hand parking home is {home['vertical_offset_m']*1000:.0f}mm above the selected grasp palm targets; arm home_q is separately solved. The ego preview uses the selected grasp arm_q stored in the report, not this parking home. Both open/default hand meshes and arms have >=30mm table clearance at parking home.")
    raw['notes'].append(f"K1-a selected static coverage {len(pair['controls'])}/{scan['total_required_controls']}; natural hard gate passed={pair['natural_pass']}, max normalized deviation={pair['natural_deviation']:.6f}. This records the requested fallback when no natural solution exists at the best coverage tier.")
    raw['notes'].extend([
        'K1-a 2026-10-22, reviewer decisions 1/3: position/height jointly selected; fixed root face up yaw -90 deg. Candidate only; collection default not switched.',
        'Source: docs/reports/2026-10-22-gamepad-height/. AR5 home_q recomputed by six-start FK/DLS IK, retaining collision-free solutions and preferring minimum natural-pose deviation. Open q=0 mesh clearance checked.',
        '20x24mm full-height central support is design geometry, not a structural or clamping certificate; K0 shell contact point is retained. Widened base suggestion is not included in this scene.',
        'Geometry evaluation uses the pinned 84f43e6 full_controls resources; the release metadata field is inherited from v4.4 and is not a runtime package version check.',
        'Right/left physical hand mounts remain unverified. Finite static grasps do not establish full-control manipulation or dynamic pressing.'
    ])
    write_json(args.config_output,raw)
    put_pair(scene,pair,pose)
    ego_preview(scene,geo,camera,args.output/'ego-preview.png')
    side_preview(scene,geo,features,pair,pose,args.output/'natural-side-view.png')
    design=footprint(scene,geo,pair,pose,args.output)
    write_json(args.output/'fixture-design.json',design);write_json(args.output/'validation.json',dict(status='passed',**validation))
    # Compact public evidence. Raw arrays, converted meshes and full per-baseline outputs remain on compute.
    write_json(args.output/'scan.json',scan);write_json(args.output/'selected-pair.json',pair)
    print(json.dumps(dict(choice=place,fixture=design,validation=validation)),flush=True)


if __name__=='__main__':main()
