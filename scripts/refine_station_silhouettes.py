#!/usr/bin/env python3
"""Offline green-screen silhouette refinement; reviewer ruling 3, no hardware IO."""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.optimize import minimize

from gamepad_vla.kinematics import SerialChain
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.workstation.spec import load_spec, matrix_to_pose, pose_to_matrix


# Frozen from source-image visibility, independent of model projections/errors.
C1_ROIS = [
 [[0,0],[520,0],[520,70],[300,190],[220,270],[0,260]],
 [[0,0],[520,0],[520,90],[360,185],[0,170]],
 [[100,0],[555,0],[555,90],[370,180],[200,100]],
 [[80,0],[570,0],[570,80],[360,190],[180,100]],
 [[80,0],[570,0],[570,80],[380,190],[180,100]],
 [[120,0],[570,0],[570,100],[390,195],[220,110]],
 [[0,0],[639,0],[639,120],[480,190],[390,390],[0,360]],
 [[0,0],[639,0],[639,90],[415,220],[320,305],[0,320]],
 [[80,0],[639,0],[639,100],[420,230],[160,110]],
 [[80,0],[639,0],[639,100],[430,230],[160,110]],
 [[80,0],[639,0],[639,100],[420,230],[160,110]],
 [[80,0],[639,0],[639,100],[420,230],[160,110]],
]
C2_ROI = [[30,0],[465,0],[465,320],[530,380],[639,400],[639,479],[40,479]]


def observation(frame, camera, rank, directory):
    image = cv2.imread(frame['images'][camera])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, (35,60,25), (95,255,255))
    blue = cv2.inRange(hsv, (85,60,35), (140,255,255))
    # Fill bottle/label/cap colour components, including bright holes in labels.
    excluded = np.zeros(green.shape,np.uint8)
    for contour in cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(contour) >= 50:
            cv2.fillConvexPoly(excluded,cv2.convexHull(contour),255)
    excluded = cv2.dilate(excluded,np.ones((5,5),np.uint8))
    roi = np.zeros_like(green)
    if camera == 'camera_1':
        polygon = C1_ROIS[rank]
    elif camera == 'camera_2':
        polygon = C2_ROI
        if frame['name'] in ('e1_f001146','e2_f001269'):
            occluder=np.array([[345,0],[639,0],[639,479],[493,479],[394,278]],np.int32)
            cv2.fillPoly(excluded,[occluder],255)
    else:
        # The old tabletop is non-green too. Score only above its fixed image
        # horizon, separately identified in each episode, never fit it as robot.
        horizon = 320 if frame['name'].startswith('e1_') else 355
        polygon = [[0,0],[639,0],[639,horizon],[0,horizon]]
    cv2.fillPoly(roi,[np.asarray(polygon,np.int32)],255)
    valid = (roi>0) & (excluded==0)
    mask = ((green==0) & valid).astype(np.uint8)
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    mask[~valid] = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask,8)
    for label in range(1,count):
        if stats[label,cv2.CC_STAT_AREA] < 20:mask[labels==label]=0
    interior = cv2.erode(valid.astype(np.uint8),np.ones((7,7),np.uint8),
                         borderType=cv2.BORDER_CONSTANT,borderValue=0)>0
    edge = (mask>cv2.erode(mask,np.ones((3,3),np.uint8))) & interior
    distance = cv2.distanceTransform((~edge).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    directory.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(directory/'observed_mask.png'),mask*255)
    cv2.imwrite(str(directory/'valid_mask.png'),valid.astype(np.uint8)*255)
    annotated=image.copy();cv2.polylines(annotated,[np.asarray(polygon,np.int32)],True,(0,0,255),2)
    annotated[mask>0]=(annotated[mask>0]*.5+np.array([255,255,255])*.5).astype(np.uint8)
    cv2.imwrite(str(directory/'observed_evidence.png'),annotated)
    return dict(observed=mask>0,valid=valid,interior=interior,distance=distance,
                observed_contour_points=int(edge.sum()),
                source=str(directory/'observed_mask.png'),valid_source=str(directory/'valid_mask.png'),
                roi=polygon,excluded_blue_pixels=int((excluded>0).sum()))


def mesh_samples(urdf):
    rng=np.random.default_rng(20261015)
    result=[]
    for link in ET.parse(urdf).getroot().findall('link'):
        visual=link.find('visual')
        if visual is None or visual.find('geometry/mesh') is None:continue
        mesh_element=visual.find('geometry/mesh')
        path=Path(mesh_element.get('filename'))
        mesh=trimesh.load(path,process=False)
        # Station AR5 STL is in millimetres, G20 STL in metres. URDF scale
        # applies before visual origin; omitting it fills the ego image with
        # thousand-times-larger arm hulls (first two runs are retained as invalid).
        mesh.apply_scale([float(v) for v in mesh_element.get('scale','1 1 1').split()])
        number=2048 if link.get('name').endswith('hand_base_link') else 512
        weights=np.array(mesh.area_faces,copy=True);weights/=weights.sum()
        triangle=rng.choice(len(weights),number,p=weights)
        a=np.sqrt(rng.random(number));b=rng.random(number)
        bary=np.c_[1-a,a*(1-b),a*b]
        points=(np.asarray(mesh.triangles)[triangle]*bary[:,:,None]).sum(axis=1)
        # Deterministic mesh extrema protect the sampled hull at thin link ends.
        vertices=np.asarray(mesh.vertices)
        extrema=np.r_[vertices.argmin(axis=0),vertices.argmax(axis=0)]
        points=np.vstack([points,vertices[extrema]])
        origin=visual.find('origin')
        if origin is not None:
            from gamepad_vla.kinematics import rpy_to_matrix
            rotation=rpy_to_matrix([float(v) for v in origin.get('rpy','0 0 0').split()])
            points=points@rotation.T+np.array([float(v) for v in origin.get('xyz','0 0 0').split()])
        result.append((link.get('name'),points))
    return result


def model_frames(spec, frames, camera, urdf_dir):
    sides=('left','right') if camera=='camera_3' else (('left',) if camera=='camera_1' else ('right',))
    prepared=[[] for _ in frames]
    for side in sides:
        arm=getattr(spec,side+'_arm')
        prefix='right_' if side=='right' else ''
        urdf=urdf_dir/('workcell_right_robot.urdf' if side=='right' else 'workcell_robot.urdf')
        samples=mesh_samples(urdf)
        if camera!='camera_3':samples=[(n,p) for n,p in samples if not n.startswith('AR5_')]
        chains={name:SerialChain(urdf,name) for name,_ in samples if name!=arm.base_link}
        root_chain=SerialChain(urdf,prefix+'hand_base_link')
        for i,frame in enumerate(frames):
            snapshot=json.loads(Path(frame['snapshot']).read_text())
            q=dict(zip(arm.joint_names,snapshot[side+'_arm']))
            q.update(dict(zip((prefix+n for n in G20_JOINT_NAMES),snapshot[side+'_hand'])))
            base=pose_to_matrix(arm.base_pose)
            hand=base@root_chain.forward([[q[n] for n in root_chain.active]])[0]
            for name,local in samples:
                transform=(base if name==arm.base_link else
                           base@chains[name].forward([[q[n] for n in chains[name].active]])[0])
                if camera!='camera_3':transform=np.linalg.inv(hand)@transform
                points=local@transform[:3,:3].T+transform[:3,3]
                prepared[i].append(points.astype(np.float64))
    return prepared


def unpack(x):
    return cv2.Rodrigues(x[:3])[0],x[3:6],np.exp(x[6]),x[7]*640,x[8]*480


def model_mask(parts,x,valid):
    rotation,translation,focal,cx,cy=unpack(x)
    mask=np.zeros((480,640),np.uint8)
    for points in parts:
        optical=points@rotation.T+translation
        optical=optical[optical[:,2]>.001]
        if len(optical)<3:continue
        uv=optical[:,:2]/optical[:,2,None]*focal+[cx,cy]
        # Rasterization of each link's projected sampled hull preserves gaps
        # between links. It approximates concavities/holes within a single STL.
        polygon=cv2.convexHull(np.clip(uv,-100000,100000).astype(np.float32))
        cv2.fillConvexPoly(mask,np.rint(polygon).astype(np.int32),1)
    mask[~valid]=0
    return mask>0


def silhouette_metrics(mask, observation):
    real=observation['observed']; union=(mask|real).sum()
    iou=float((mask&real).sum()/union) if union else 0.
    boundary=(mask.astype(np.uint8)>cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8))) & observation['interior']
    distances=observation['distance'][boundary]
    # When both boundaries are completely cropped/occluded there is no contour
    # evidence. A boundary present on only one side is a missing/false boundary,
    # and gets the truncation penalty rather than a vacuous zero-distance score.
    if not len(distances) and not observation['observed_contour_points']:
        chamfer=None
    elif not len(distances) or not observation['observed_contour_points']:
        chamfer=20.
    else:
        chamfer=float(np.minimum(distances,20).mean())
    return dict(iou=iou,truncated_chamfer_px=chamfer,model_contour_points=int(boundary.sum()),
                observed_contour_points=observation['observed_contour_points'],
                observed_pixels=int(real.sum()),model_pixels=int(mask.sum()))


def keypoint_metrics(calibration,x,split):
    points=[p for p in calibration['points'] if p['split']==split]
    rotation,translation,focal,cx,cy=unpack(x)
    xyz=np.asarray([p['xyz'] for p in points])@rotation.T+translation
    uv=xyz[:,:2]/xyz[:,2,None]*focal+[cx,cy]
    errors=np.linalg.norm(uv-np.asarray([p['uv'] for p in points]),axis=1)
    return dict(count=len(points),median_px=float(np.median(errors)),p95_px=float(np.percentile(errors,95)))


def parameters(calibration):
    optical_parent=np.linalg.inv(pose_to_matrix(calibration['pose_optical']))
    intr=calibration['intrinsics']
    return np.r_[cv2.Rodrigues(optical_parent[:3,:3])[0].ravel(),optical_parent[:3,3],
                 np.log(intr['fx']),intr['cx']/640,intr['cy']/480]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec',type=Path,required=True)
    parser.add_argument('--selection',type=Path,required=True)
    parser.add_argument('--urdf-dir',type=Path,required=True)
    parser.add_argument('--pnp-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-seconds',type=float,default=7200)
    args=parser.parse_args()
    start=time.monotonic();deadline=start+min(7200,args.max_seconds)
    spec=load_spec(args.spec);frames=json.loads(args.selection.read_text())['frames']
    summary=dict(start_utc=datetime.now(timezone.utc).isoformat(),budget_seconds=min(7200,args.max_seconds),
                 method='surface samples; per-link projected convex hull union; multi-start Nelder-Mead',
                 objective='mean over fit frames [min(chamfer_px,20)/20 + 1-IoU] + nominal intrinsic priors',
                 observed_rule='HSV green H35:95 S>=60 V>=25; blue H85:140 S>=60 V>=35 component hulls area>=50, dilate5; open3/close3/remove<20; ROI fixed from source visibility; erode valid domain7 before contour extraction',
                 selection_rule='min validation median/3 + P95/8 + (1-mean_IoU); unchanged four validation frames; both methods scored after fitting; validation is also method-selection data, not an additional unseen test',
                 cameras={})
    args.output.mkdir(parents=True,exist_ok=True)
    for number in (1,2,3):
        camera=f'camera_{number}';directory=args.output/camera;directory.mkdir(parents=True,exist_ok=True)
        cal=json.loads((args.pnp_root/camera/'calibration.json').read_text())
        observations=[observation(f,camera,i,directory/f['name']) for i,f in enumerate(frames)]
        models=model_frames(spec,frames,camera,args.urdf_dir)
        train=[i for i,f in enumerate(frames) if f['split']=='fit']
        x0=parameters(cal);best=[float('inf'),x0.copy()];calls=[0]
        camera_deadline=min(deadline,time.monotonic()+2100)
        class BudgetExpired(Exception):pass
        def objective(x):
            if time.monotonic()>camera_deadline:raise BudgetExpired()
            _,_,focal,cx,cy=unpack(x)
            if not (160<focal<2560 and -.25*640<cx<1.25*640 and -.25*480<cy<1.25*480):return 100.
            losses=[]
            for i in train:
                metrics=silhouette_metrics(model_mask(models[i],x,observations[i]['valid']),observations[i])
                chamfer=metrics['truncated_chamfer_px']
                losses.append((chamfer/20 if chamfer is not None else 0.)+1-metrics['iou'])
            fov=np.rad2deg(2*np.arctan(320/focal))
            prior=.01*((fov-69)/10)**2+.005*((cx-320)/64)**2+.005*((cy-240)/48)**2
            value=float(np.mean(losses)+prior);calls[0]+=1
            if value<best[0]:best[:]=[value,x.copy()]
            return value
        records=[]
        seeds=[]
        for factor in (1.,.85,1.15):
            seed=x0.copy();seed[6]+=np.log(factor)
            if factor!=1:seed[7:]=[.5,.5]
            seeds.append(seed)
        if number==3:
            # The initial ego fit has near-camera arm geometry and a flat,
            # almost full-ROI hull in the first run. Perturb camera position as
            # well as K, preserving its fitted roll; these are starting points,
            # never fixed scene measurements or final manually chosen poses.
            initial_pose=pose_to_matrix(cal['pose_optical'])
            target=np.mean(np.vstack([p for model in models for p in model]),axis=0)
            for dz in (.3,.6):
                position=initial_pose[:3,3]+[-.12,0,dz]
                forward=target-position;forward/=np.linalg.norm(forward)
                right=initial_pose[:3,0]-forward*np.dot(forward,initial_pose[:3,0]);right/=np.linalg.norm(right)
                down=np.cross(forward,right)
                pose=np.eye(4);pose[:3,:3]=np.stack([right,down,forward],axis=1);pose[:3,3]=position
                inverse=np.linalg.inv(pose)
                seed=x0.copy();seed[:3]=cv2.Rodrigues(inverse[:3,:3])[0].ravel();seed[3:6]=inverse[:3,3]
                seeds.append(seed)
        for j,seed in enumerate(seeds):
            scales=np.array([.025]*3+[.008 if number<3 else .025]*3+[.06,.025,.025])
            simplex=np.tile(seed,(10,1));simplex[1:]+=np.diag(scales)
            try:
                result=minimize(objective,seed,method='Nelder-Mead',
                    options=dict(maxfev=700,maxiter=600,initial_simplex=simplex,xatol=1e-4,fatol=1e-5))
                records.append(dict(start=j,cost=float(result.fun),nfev=int(result.nfev),success=bool(result.success)))
            except BudgetExpired:
                records.append(dict(start=j,stopped='time budget'));break
            print(json.dumps(dict(camera=camera,start=j,best_train_loss=best[0],evaluations=calls[0],elapsed_seconds=time.monotonic()-start)),flush=True)
        method_results={}
        for method,x in (('pnp',x0),('silhouette',best[1])):
            per_frame={}
            for i,frame in enumerate(frames):
                mask=model_mask(models[i],x,observations[i]['valid'])
                cv2.imwrite(str(directory/frame['name']/f'{method}_model_mask.png'),mask.astype(np.uint8)*255)
                per_frame[frame['name']]=silhouette_metrics(mask,observations[i])
            aggregate={}
            for split in ('fit','validation'):
                keys=[f['name'] for f in frames if f['split']==split]
                measured=[per_frame[k]['truncated_chamfer_px'] for k in keys if per_frame[k]['truncated_chamfer_px'] is not None]
                aggregate[split]=dict(mean_iou=float(np.mean([per_frame[k]['iou'] for k in keys])),
                    mean_truncated_chamfer_px=float(np.mean(measured)) if measured else None,
                    frames_with_defined_chamfer=len(measured),
                    frames_with_both_contours=sum(per_frame[k]['model_contour_points']>0 and per_frame[k]['observed_contour_points']>0 for k in keys),
                    keypoints=keypoint_metrics(cal,x,split))
            kp=aggregate['validation']['keypoints']
            score=kp['median_px']/3+kp['p95_px']/8+(1-aggregate['validation']['mean_iou'])
            method_results[method]=dict(aggregate=aggregate,per_frame=per_frame,selection_score=score,parameters=x.tolist())
        selected=min(method_results,key=lambda method:method_results[method]['selection_score'])
        x=np.asarray(method_results[selected]['parameters']);rotation,translation,focal,cx,cy=unpack(x)
        parent_camera=np.eye(4);parent_camera[:3,:3]=rotation.T;parent_camera[:3,3]=-rotation.T@translation
        final=json.loads(json.dumps(cal));final['pose_optical']=matrix_to_pose(parent_camera)
        final['intrinsics'].update(fx=float(focal),fy=float(focal),cx=float(cx),cy=float(cy))
        final['metrics']={split:method_results[selected]['aggregate'][split]['keypoints'] for split in ('fit','validation')}
        final['selected_method']=selected;final['method_comparison']=method_results
        kp=final['metrics']['validation'];final['validation_passed']=kp['median_px']<=3 and kp['p95_px']<=8
        final['quality']='estimated, holdout threshold met' if final['validation_passed'] else '粗配准'
        final['status']='estimated' if final['validation_passed'] else '粗配准';final['source']=str(directory/'comparison.json')
        # Keep stored 2D predictions consistent with the final candidate, so the
        # SAPIEN-vs-NumPy projection check compares the same camera definition.
        for p in final['points']:
            optical=rotation@np.asarray(p['xyz'])+translation
            predicted=optical[:2]/optical[2]*focal+[cx,cy]
            p['projected']=predicted.tolist();p['error_px']=float(np.linalg.norm(predicted-p['uv']))
        result=dict(selected_method=selected,methods=method_results,starts=records,evaluations=calls[0],
                    elapsed_seconds=time.monotonic()-start,observation_rois={f['name']:o['roi'] for f,o in zip(frames,observations)})
        (directory/'comparison.json').write_text(json.dumps(result,indent=2))
        (directory/'calibration.json').write_text(json.dumps(final,indent=2,ensure_ascii=False))
        summary['cameras'][camera]=result
        summary['elapsed_seconds']=time.monotonic()-start
        (args.output/'comparison.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(dict(camera=camera,selected=selected,methods={m:r['aggregate']['validation'] for m,r in method_results.items()})),flush=True)
    summary['elapsed_seconds']=time.monotonic()-start
    (args.output/'comparison.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':main()
