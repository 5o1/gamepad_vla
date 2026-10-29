#!/usr/bin/env python3
"""Render the CAD ChArUco card in the existing SAPIEN scene, then estimate RGB.

Absolute and two-image root displacement are evaluated separately, with the
same sixteen perturbations as validate_gamepad_pose.py. No hardware or ROS.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from gamepad_vla.workstation.gamepad_pose_check import (DEFAULT_SPEC, SilhouetteModel,
    load_hulls, matrix_pose, perturbed_pose, pose_matrix, write_result)
from gamepad_vla.workstation.gamepad_marker import (DEFAULT_MARKER, board_rectangles,
    check_marker_pose, relative_motion)
from validate_gamepad_pose import cases


def write_rectangles(path,rectangles,z):
    rows=[];offset=1
    for x,y,w,h in rectangles:
        rows.extend(f'v {a:.9f} {b:.9f} {z:.9f}' for a,b in [(x,y),(x+w,y),(x+w,y+h),(x,y+h)])
        rows.extend([f'f {offset} {offset+2} {offset+1}',f'f {offset} {offset+3} {offset+2}'])
        offset+=4
    path.write_text('\n'.join(rows)+'\n')


def add_card(backend,config,output):
    import sapien
    b=config['board'];w=b['squares_x']*b['square_m'];h=b['squares_y']*b['square_m'];margin=b['border_m']
    write_rectangles(output/'card-white.obj',[(-margin,-margin,w+2*margin,h+2*margin)],.00025)
    write_rectangles(output/'card-black.obj',board_rectangles(config),0.)
    actors=[]
    for color,file in [([1,1,1,1],'card-white.obj'),([0,0,0,1],'card-black.obj')]:
        material=sapien.render.RenderMaterial();material.base_color=color;material.roughness=1.
        builder=backend.scene.create_actor_builder()
        builder.add_visual_from_file(str(output/file),material=material)
        actors.append(builder.build_static(name=file))
    return actors


def main():
    from gamepad_vla.site import configure_runtime
    configure_runtime()
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--spec',type=Path,default=DEFAULT_SPEC)
    p.add_argument('--marker-manifest',type=Path,default=DEFAULT_MARKER)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=int)
    p.add_argument('--supersample',type=int,default=1)
    p.add_argument('--bracket-mesh',type=Path)
    p.add_argument('--sticker-slip',action='store_true',help='small sticker/card relative perturbations')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    config=json.loads(a.marker_manifest.read_text());scene=json.loads(a.spec.read_text())
    import sapien
    from gamepad_vla.workstation.sapien_backend import SapienBackend
    from gamepad_vla.workstation.spec import load_spec
    from gamepad_vla.workstation.render_review import create_spec_camera
    from gamepad_vla.workstation.collection_camera import color_to_rgb8
    sapien.render.set_camera_shader_dir('default')
    spec=load_spec(a.spec);backend=SapienBackend(spec,render=True,work_dir=a.output/'render_work')
    definition=next(c for c in spec.cameras if c.name=='camera_3')
    if a.supersample != 1:
        from dataclasses import replace
        intr0=definition.intrinsics
        definition=replace(definition,intrinsics=replace(intr0,width=intr0.width*a.supersample,height=intr0.height*a.supersample,fx=intr0.fx*a.supersample,fy=intr0.fy*a.supersample,cx=intr0.cx*a.supersample,cy=intr0.cy*a.supersample))
    camera=create_spec_camera(backend,definition)
    kk=definition.intrinsics
    camera.set_perspective_parameters(.01,20.,kk.fx,kk.fy,kk.cx,kk.cy,0.)
    actors=add_card(backend,config,a.output)
    bracket=None
    if a.bracket_mesh:
        material=sapien.render.RenderMaterial();material.base_color=[.015,.2,.23,1.]
        builder=backend.scene.create_actor_builder()
        builder.add_visual_from_file(str(a.bracket_mesh),scale=[.001]*3,material=material)
        bracket=builder.build_static(name='removable_marker_bracket')
    sticker_actors=[]
    if 'sticker' in config:
        c=config['sticker'];size=c['size_m'];border=c['paper_border_m']
        code=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,c['dictionary'])),c['id'],6)
        black=[(-size/2+x*size/6,-size/2+y*size/6,size/6,size/6) for y,x in np.argwhere(code==0)]
        write_rectangles(a.output/'sticker-white.obj',[(-size/2-border,-size/2-border,size+2*border,size+2*border)],.00025)
        write_rectangles(a.output/'sticker-black.obj',black,0.)
        for color,file in [([1,1,1,1],'sticker-white.obj'),([0,0,0,1],'sticker-black.obj')]:
            material=sapien.render.RenderMaterial();material.base_color=color;material.roughness=1.
            builder=backend.scene.create_actor_builder();builder.add_visual_from_file(str(a.output/file),material=material)
            sticker_actors.append(builder.build_static(name=file))
    target=pose_matrix(scene['gamepad']['root_pose']);mount=np.asarray(config['gamepad_from_board'])
    sticker_mount=np.asarray(config.get('sticker',{}).get('gamepad_from_sticker_synthetic_only',np.eye(4)))
    # Flat simulation label stands 1 mm off the convex collision approximation;
    # this is NOT physical clearance certification or a calibrated hand placement.
    sticker_mount[2,3]+=.001
    experiment_cases=cases()
    if a.sticker_slip:
        experiment_cases=[('nominal',np.zeros(6))]
        for axis in range(2):
            for amplitude in [-.5,-.2,.2,.5]:
                delta=np.zeros(6);delta[axis]=amplitude
                experiment_cases.append((f'slip_{axis}_{amplitude:+.1f}',delta))
        for angle in [-1.,-.3,.3,1.]:
            delta=np.zeros(6);delta[5]=angle
            experiment_cases.append((f'slip_turn_{angle:+.1f}',delta))
        experiment_cases.extend([('common_left',np.zeros(6)),('common_pitch',np.zeros(6)),('planar_noise',np.array([.3,-.3,0,0,0,.3]))])
    intr=scene['cameras'][0]['intrinsics']
    k=np.array([[intr['fx'],0,intr['cx']],[0,intr['fy'],intr['cy']],[0,0,1.]])
    meta=dict(width=1280,height=720,K=k.ravel().tolist(),D=intr['distortion'],distortion_model='plumb_bob',source='sapien-marker')
    hulls=load_hulls();rows=[];baseline=None;truth_baseline=None
    rng=np.random.default_rng(20261030)
    for name,delta in experiment_cases[:a.limit]:
        truth=target.copy() if a.sticker_slip else perturbed_pose(target,delta)
        if name=='common_left':truth=perturbed_pose(target,[-3,0,0,0,0,0])
        if name=='common_pitch':truth=perturbed_pose(target,[0,0,0,2,0,0])
        q=matrix_pose(truth);backend.gamepad.articulation.set_root_pose(sapien.Pose(q[:3],q[3:]))
        if bracket is not None:bracket.set_pose(sapien.Pose(q[:3],q[3:]))
        card_pose=matrix_pose(truth@mount)
        for actor in actors:actor.set_pose(sapien.Pose(card_pose[:3],card_pose[3:]))
        sticker_truth=truth@sticker_mount
        if a.sticker_slip:
            delta_transform=np.eye(4);delta_transform[:3,3]=delta[:3]/1000
            delta_transform[:3,:3]=Rotation.from_euler('xyz',delta[3:],degrees=True).as_matrix()
            sticker_truth=truth@sticker_mount@delta_transform
        sp=matrix_pose(sticker_truth)
        for actor in sticker_actors:actor.set_pose(sapien.Pose(sp[:3],sp[3:]))
        backend.scene.update_render();camera.take_picture()
        image=cv2.cvtColor(color_to_rgb8(camera.get_picture('Color')),cv2.COLOR_RGB2BGR)
        if a.supersample != 1:
            image=cv2.resize(image,(1280,720),interpolation=cv2.INTER_AREA)
        if name in ('combined_noise','planar_noise'):
            image=cv2.GaussianBlur(image,(3,3),.65)
            image=np.clip(image.astype(float)+rng.normal(0,2,image.shape),0,255).astype(np.uint8)
        cv2.imwrite(str(a.output/(name+'.input.png')),image)
        result,overlay=check_marker_pose(image,meta,scene,hulls,config,reference=baseline)
        write_result(a.output/name,result,overlay)
        row=dict(case=name,truth_delta_mm_deg=delta.tolist(),detected=result['detected'])
        if result['detected']:
            actual=pose_matrix(result['estimated_pose'])
            row.update(position_error_mm=float(np.linalg.norm(actual[:3,3]-truth[:3,3])*1000),
                rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3]@truth[:3,:3].T).magnitude())),
                corners=result['quality']['charuco_corners'],rms_px=result['quality']['reprojection_rms_px'])
            if baseline is None:
                baseline=result;truth_baseline=truth
            actual_relative=(np.asarray(result['relative_to_reference']['relative_transform'])
                             if result.get('relative_to_reference',{}).get('relative_transform') is not None
                             else np.eye(4))
            truth_relative=np.linalg.inv(truth_baseline)@truth
            row['relative_position_error_mm']=float(np.linalg.norm(actual_relative[:3,3]-truth_relative[:3,3])*1000)
            row['relative_rotation_error_deg']=float(np.degrees(Rotation.from_matrix(actual_relative[:3,:3]@truth_relative[:3,:3].T).magnitude()))
            # Alter world extrinsic while leaving the two camera measurements
            # unchanged: relative root norms must be exactly invariant.
            rel=result.get('relative_to_reference') or relative_motion(np.asarray(baseline['camera_from_board']),np.asarray(result['camera_from_board']),mount)
            if 'position_change_mm' in rel:
                row['relative_position_change_mm']=rel['position_change_mm']
                row['relative_rotation_change_deg']=rel['rotation_change_deg']
            else:
                row['tracking_failure']=rel.get('message','unknown')
                row['relative_position_error_mm']=None
                row['relative_rotation_error_deg']=None
        if a.sticker_slip:
            slip=result.get('sticker',{}).get('change_from_reference',{})
            row['sticker_detected']=result.get('sticker',{}).get('detected',False)
            if name=='nominal' and row['sticker_detected']:
                row['sticker_position_error_mm']=0.;row['sticker_rotation_error_deg']=0.
            elif 'relative_transform' in slip:
                measured=np.asarray(slip['in_plane_translation_mm'])
                expected=np.array([delta[0],-delta[1]])
                row['sticker_position_error_mm']=float(np.linalg.norm(measured-expected))
                row['sticker_rotation_error_deg']=float(abs((slip['rotation_about_normal_deg']+delta[5]+180)%360-180))
                row['sticker_position_change_mm']=slip['position_change_mm']
            else:
                row['sticker_position_error_mm']=None;row['sticker_rotation_error_deg']=None
                row['sticker_failure']=slip.get('message','not detected')
        rows.append(row);print(json.dumps(row),flush=True)
    summary=dict(source='SAPIEN 3.0.3, shared collection backend, CAD marker geometry',supersample=a.supersample,
                 cases=rows,total=len(rows),detected=sum(r['detected'] for r in rows))
    good=[r for r in rows if r['detected']]
    if good:
        summary['accuracy']={key:dict(median=float(np.median([r[key] for r in good if r[key] is not None])),
            p95=float(np.percentile([r[key] for r in good if r[key] is not None],95)),max=float(max(r[key] for r in good if r[key] is not None)))
            for key in ['position_error_mm','rotation_error_deg','relative_position_error_mm','relative_rotation_error_deg']}
    if a.sticker_slip:
        summary['sticker_successes']=sum(r.get('sticker_position_error_mm') is not None for r in rows)
        summary['sticker_accuracy']={key:dict(median=float(np.median([r[key] for r in rows if r.get(key) is not None])),max=float(max(r[key] for r in rows if r.get(key) is not None))) for key in ['sticker_position_error_mm','sticker_rotation_error_deg']}
    summary['relative_successes']=sum(r.get('relative_position_error_mm') is not None for r in rows)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if len(good)!=len(rows):raise SystemExit('Marker detection failed; inspect input images')


if __name__=='__main__':main()
