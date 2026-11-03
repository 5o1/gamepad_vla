"""Export Z's compact evidence and report from native witnesses; CPU only."""
import os
import argparse
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time
import numpy as np


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def rejection_reasons(value):
    out=[]
    if isinstance(value,dict):
        if isinstance(value.get('reasons'),list):out+=value['reasons']
        if isinstance(value.get('native_rejections'),dict):out+=list(value['native_rejections'])
        if isinstance(value.get('reason'),str):out.append(value['reason'])
        for v in value.values():out+=rejection_reasons(v)
    elif isinstance(value,list):
        for v in value:out+=rejection_reasons(v)
    return out


def main(args):
    from gamepad_vla.feasibility import Pad
    from gamepad_vla.workstation.spec import load_spec,pose_to_matrix
    from k1_natural_reference import ArmFeatures
    root=args.run;source=args.source;report=root/'report';report.mkdir(exist_ok=True)
    pad=Pad(os.environ['GAMEPAD_VLA_WORKSPACE']+'/assets/xbox_gamepad_physics_mujoco_84f43e6/resources')
    scene=load_spec(source/'final/rank-1/spec.json')
    features={side:ArmFeatures(getattr(scene,side+'_arm'),getattr(scene,side+'_hand')) for side in ('left','right')}
    goals={side:{g['name']:g for g in pad.targets(side) if g['name'] not in ('View','Xbox','Menu')} for side in ('left','right')}
    points=['d050-h220-t+10','d075-h190-t+10','d025-h250-t+10','d050-h190-t+10']
    summary=[];candidates={};allrows=[];failed=[];assertions=0
    for rank,point in enumerate(points,1):
        for side in ('left','right'):
            path=root/'solve'/point/side/'result.json'
            r=json.loads(path.read_text())
            item=dict(point=point,rank=rank,side=side,completed=r['completed'],
                candidates=r['candidates'],elapsed_seconds=r['elapsed_seconds'])
            item['search_status']={k:r.get(k) for k in ('completed','stage2_completed','stage3_completed',
                'stage4_completed','elapsed_seconds','stage2_elapsed_seconds','stage3_elapsed_seconds','stage4_elapsed_seconds')}
            old=json.loads((source/'controls'/f'{point}-{side}.json').read_text())
            item['X_selected_controls']=old['controls'];item['X_maximum_count']=old['maximum_controls_found']
            for mode in ('strict','realistic'):
                if mode not in r:raise RuntimeError(f'{point}/{side}/{mode} never completed')
                result=r[mode];base=result['check'];assert base['passed']
                baseq=base['arm_q'];accepted=[];operations=[]
                feature=features[side];bp=feature.frames(baseq)['palm'][0]
                bc=bp[:3,:3]@feature.palm_point_local+bp[:3,3]
                for name in result['controls']:
                    rows=[s for s in result['states'] if s['control']==name]
                    goal=goals[side][name]
                    assert len(rows)>0
                    if not goal.get('alternatives'):
                        assert {s['state'] for s in rows}==set(range(len(goal['states'])))
                    for s in rows:
                        c=s['check'];m=c['natural'];a=c['active_contact']
                        assert c['passed'] and not c['violations'] and not c['reasons']
                        assert 10<=m['upper_tilt_deg']<=35 and -10<=m['forearm_pitch_deg']<=25
                        assert m['forearm_heading_deg']<=30 and m['upper_forward']
                        assert a['point_error_m']<=.00025 and a['native_active_contact']
                        assert -.0001<=a['native_active_distance_m']<=0
                        if name not in ('LS','RS','LT','RT'):assert a['normal_cos']>=3**.5/2
                        if mode=='strict':assert c['arm_q']==baseq
                        else:
                            wp=feature.frames(c['arm_q'])['palm'][0]
                            wc=wp[:3,:3]@feature.palm_point_local+wp[:3,3]
                            distance=float(np.linalg.norm(wc-bc)*1000)
                            root_distance=float(np.linalg.norm(wp[:3,3]-bp[:3,3])*1000)
                            angle=float(np.degrees(np.arccos(np.clip((np.trace(wp[:3,:3]@bp[:3,:3].T)-1)/2,-1,1))))
                            assert distance<=20.0000001 and angle<=15.0000001
                            assert abs(distance-s['motion']['centroid_displacement_mm'])<.0001
                            assert abs(root_distance-s['motion']['root_displacement_mm'])<.0001
                            assert abs(angle-s['motion']['rotation_deg'])<.0001
                        s['gamepad_q']=goal['states'][s['state']]['gamepad_q']
                        assertions+=1
                    accepted+=rows
                    op=dict(control=name,states=len(rows),finger=goal['finger'],
                        minimum_rear_contact_fingers=min(len(s['check']['grips']) for s in rows),
                        all_states_both_rear_contacts=all(set(s['check']['grips'])=={'ring','pinky'} for s in rows),
                        centroid_displacement_mm=max(s['motion']['centroid_displacement_mm'] for s in rows),
                        root_displacement_mm=max(s['motion']['root_displacement_mm'] for s in rows),
                        rotation_deg=max(s['motion']['rotation_deg'] for s in rows),
                        natural_ranges={k:[min(s['check']['natural'][k] for s in rows),max(s['check']['natural'][k] for s in rows)]
                            for k in ('upper_tilt_deg','elbow_angle_deg','forearm_pitch_deg','forearm_heading_deg','wrist_deviation_deg')},
                        maximum_point_error_mm=max(s['check']['active_contact']['point_error_m']*1000 for s in rows))
                    operations.append(op)
                    ranges={k+'_'+end:v[i] for k,v in op['natural_ranges'].items() for i,end in enumerate(('min','max'))}
                    allrows.append(dict(point=point,side=side,mode=mode,**op,**ranges))
                item[mode]=dict(count=len(result['controls']),controls=result['controls'],label=result.get('label',result.get('base_label')),
                    natural=base['natural'],elbow_forward_mm=(base['natural_points_m']['elbow'][0]-base['natural_points_m']['shoulder'][0])*1000,
                    grips=base['grips'],grip_point_error_mm=[x*1000 for x in base['grip_point_error_m']],operations=operations)
                candidates[f'{point}/{side}/{mode}']=dict(x=result['x'],check=base)
            # Add deterministic goal q metadata for rendering; checks stay intact.
            write(path,r)
            missing=set(goals[side])-set(r['strict']['controls'])-set(r['realistic']['controls'])
            checked=json.loads((root/'solve'/point/side/'candidates-progress.json').read_text())
            for name in sorted(missing):
                g=goals[side][name];s=r['strict']['radial'][name];v=r['realistic']['radial'][name]
                # Bound about the constrained palm mesh centroid, not the root.
                import math
                palm=pose_to_matrix(r['realistic']['check']['palm_world'])
                centroid=np.array(r['realistic']['check']['palm_mesh_centroid_world_m'])
                fingerbase=palm[:3,:3]@v['finger_base_in_palm_m']+palm[:3,3]
                lever=float(np.linalg.norm(fingerbase-centroid))
                allowance=.020+2*lever*math.sin(math.radians(7.5))
                d=r['realistic']['diagnostics'][name]
                reasons=rejection_reasons(d)
                failed.append(dict(point=point,side=side,control=name,finger=g['finger'],
                    radius_mm=s['chain_radius_upper_bound_m']*1000,
                    strict_chain_deficit_mm=s['radial_point_error_lower_bound_m']*1000,
                    realistic_base_chain_deficit_mm=v['radial_point_error_lower_bound_m']*1000,
                    realistic_region_certified_deficit_mm=max(0.,v['radial_point_error_lower_bound_m']-allowance)*1000,
                    alternate_valid_holds=[c['label'] for c in checked if c['check']['passed'] and name in c['controls']],
                    final_attempt_reasons=sorted(set(reasons)),scope='selected hold only; enlarged sphere covers all allowed centroid translations and rotations'))
            summary.append(item)
    write(report/'summary.json',summary);write(report/'candidates.json',candidates)
    write(report/'unreached.json',failed)
    with (report/'operations.csv').open('w') as f:
        fields=['point','side','mode','control','finger','states','centroid_displacement_mm','root_displacement_mm','rotation_deg','maximum_point_error_mm',
            'minimum_rear_contact_fingers','all_states_both_rear_contacts']
        fields += [k+'_'+end for k in ('upper_tilt_deg','elbow_angle_deg','forearm_pitch_deg','forearm_heading_deg','wrist_deviation_deg') for end in ('min','max')]
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(allrows)
    files=[source/'code/scripts'/n for n in ['natural_grip_fixed.py','check_natural_grip_fixed.py','k1_natural_reference.py','gamepad_placement.py','k1b_native.py','k1b_contact_fit.py']]
    files += [source/'code/src/gamepad_vla/feasibility.py',source/'code/src/gamepad_vla/workstation/operator_view.py']
    files += [source/'final'/f'rank-{k}'/'spec.json' for k in range(1,5)]
    files += [Path(getattr(scene,s+'_'+kind).urdf) for s in ('left','right') for kind in ('arm','hand')]
    files += [root/n for n in ['solve-stage1.py','solve-stage2.py','solve.py','launch.py','refine.py','launch_refine.py',
        'adjust.py','launch_adjust.py','rebase.py','render.py','finalize.py'] if (root/n).exists()]
    x_provenance=json.loads((source/'provenance.json').read_text())
    resource_comparison=[]
    for entry in x_provenance['collision_comparison']:
        rel=entry['path'];expected=entry['sha256']
        a=Path(os.environ['GAMEPAD_VLA_WORKSPACE'],'assets/xbox_gamepad_physics_mujoco_84f43e6/resources')/rel
        b=Path(os.environ['GAMEPAD_VLA_WORKSPACE'],'assets/xbox_gamepad_physics_v0.4.1/resources')/rel
        hashes=[hashlib.sha256(p.read_bytes()).hexdigest() for p in (a,b)]
        resource_comparison.append(dict(path=rel,sha256=hashes,identical_to_X=all(h==expected for h in hashes)))
    assert all(c['identical_to_X'] for c in resource_comparison),'collision resources changed since X'
    import xbox_gamepad_physics
    write(report/'provenance.json',dict(baseline_head='37f44eff2e05732aa6151c44a2a5d032aa0146af',
        source_run=str(source),run=str(root),sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        budget=json.loads((root/'budget.json').read_text()),asserted_native_witnesses=assertions,exported_epoch=time.time(),
        collision_resources=resource_comparison,interface_source=str(Path(xbox_gamepad_physics.__file__).resolve()),
        package_metadata={n:importlib.metadata.version(n) for n in ['numpy','scipy','trimesh','mujoco','xbox-gamepad-physics']}))
    print(json.dumps([dict(point=r['point'],side=r['side'],strict=r['strict']['count'],realistic=r['realistic']['count'],completed=r['completed']) for r in summary]),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    main(p.parse_args())
