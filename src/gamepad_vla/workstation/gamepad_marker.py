"""ChArUco metrology: absolute world pose and camera-frame two-image motion.

The card is rigidly attached to the moving cradle. It cannot observe controller
slip relative to that cradle. All transforms use metres and column vectors.
"""
from pathlib import Path
import json

import numpy as np

from .gamepad_pose_check import (F, DEFAULT_SPEC, SilhouetteModel, components,
                                  matrix_pose, pose_matrix)

DEFAULT_MARKER = DEFAULT_SPEC.parent/'gamepad_marker_card_v1.json'


def make_board(config):
    import cv2
    c = config['board']
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,c['dictionary']))
    ids = np.arange(c['first_id'],c['first_id']+c['squares_x']*c['squares_y']//2,dtype=np.int32)
    board = cv2.aruco.CharucoBoard((c['squares_x'],c['squares_y']),c['square_m'],c['marker_m'],dictionary,ids)
    board.setLegacyPattern(False)
    return board


def board_rectangles(config):
    """Exact vector black cells in board metres; shared by SVG and SAPIEN mesh."""
    import cv2
    board = make_board(config)
    c = config['board']; s=c['square_m']
    rects=[]
    for y in range(c['squares_y']):
        for x in range(c['squares_x']):
            if (x+y)%2 == 0:
                rects.append((x*s,y*s,s,s))
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,c['dictionary']))
    for marker_id, corners in zip(board.getIds(),board.getObjPoints()):
        code=cv2.aruco.generateImageMarker(dictionary,int(marker_id),6)
        left,top=corners[0,:2]
        size=c['marker_m']/6
        for y,x in np.argwhere(code==0):
            rects.append((float(left+x*size),float(top+y*size),size,size))
    return rects


def estimate_board(image,k,d,config):
    import cv2
    board=make_board(config)
    parameters=cv2.aruco.CharucoParameters()
    parameters.cameraMatrix=k
    parameters.distCoeffs=d
    parameters.tryRefineMarkers=True
    detector_parameters=cv2.aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.CharucoDetector(board,parameters,detector_parameters)
    corners,ids,_,_=detector.detectBoard(image)
    if ids is None or len(ids)<10:
        return None,dict(charuco_corners=0 if ids is None else len(ids),reason='fewer than 10 ChArUco corners')
    obj=board.getChessboardCorners()[ids.ravel()].astype(np.float64)
    pixels=corners.reshape(-1,2).astype(np.float64)
    # IPPE supplies both planar solutions; choose by reprojection, not target
    # closeness. LM refines the selected pose without a target-pose prior.
    ok,rs,ts,errors=cv2.solvePnPGeneric(obj,pixels,k,d,flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None,dict(charuco_corners=len(ids),reason='PnP failed')
    order=np.argsort(np.asarray(errors).ravel())
    r,t=rs[order[0]],ts[order[0]]
    r,t=cv2.solvePnPRefineLM(obj,pixels,k,d,r,t)
    projected,jac=cv2.projectPoints(obj,r,t,k,d)
    residual=projected.reshape(-1,2)-pixels
    rms=float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))
    transform=np.eye(4);transform[:3,:3]=cv2.Rodrigues(r)[0];transform[:3,3]=t.ravel()
    candidates=[float(np.asarray(errors).ravel()[j]) for j in order]
    # Covariance is conditional on the printed board geometry and intrinsics.
    j=jac[:,:6]
    covariance=np.linalg.pinv(j.T@j)*max(.08,rms/np.sqrt(2))**2
    quality=dict(charuco_corners=int(len(ids)),reprojection_rms_px=rms,
                 ippe_candidate_rms_px=candidates,
                 covariance_rvec_rad_tvec_m=covariance.tolist(),
                 corners_px=pixels.tolist(),corner_ids=ids.ravel().tolist())
    if rms>1.0 or t[2,0]<=0:
        quality['reason']='reprojection or cheirality failed'
        return None,quality
    return transform,quality


def tracking_patch(image,camera_from_board,k,d,config):
    """Small self-contained baseline patch; no growing image cache or sidecar."""
    import base64
    import cv2
    c=config['board'];w=c['squares_x']*c['square_m'];h=c['squares_y']*c['square_m']
    points=np.array([[0,0,0],[w,0,0],[w,h,0],[0,h,0]],float)
    uv=cv2.projectPoints(points,cv2.Rodrigues(camera_from_board[:3,:3])[0],camera_from_board[:3,3],k,d)[0].reshape(-1,2)
    lo=np.maximum(np.floor(uv.min(0)-6),0).astype(int)
    hi=np.minimum(np.ceil(uv.max(0)+6),image.shape[1::-1]).astype(int)
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    patch=gray[lo[1]:hi[1],lo[0]:hi[0]]
    return dict(roi=[*lo.tolist(),*hi.tolist()],png_base64=base64.b64encode(cv2.imencode('.png',patch)[1]).decode())


def refine_relative(reference,image,current_camera_from_board,k,d,config):
    """Six-DOF direct planar registration between two identified card images.

    Thousands of pixel correspondences constrain one rigid plane, rather than
    fitting eight unconstrained homography entries to a noisy raster outline.
    """
    c=config['board']
    return refine_plane(reference['tracking_patch'],np.asarray(reference['camera_from_board']),
                        image,current_camera_from_board,k,d,
                        (0.,c['squares_x']*c['square_m'],0.,c['squares_y']*c['square_m']))


def refine_plane(patch,previous,image,initial,k,d,bounds):
    import base64
    import cv2
    from scipy.optimize import least_squares
    from scipy.ndimage import map_coordinates
    from scipy.spatial.transform import Rotation
    x0,y0,x1,y1=patch['roi']
    template=cv2.imdecode(np.frombuffer(base64.b64decode(patch['png_base64']),np.uint8),cv2.IMREAD_GRAYSCALE).astype(float)/255
    current=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float)/255
    yy,xx=np.mgrid[0:template.shape[0],0:template.shape[1]]
    pixels=np.c_[xx.ravel()+x0,yy.ravel()+y0].astype(float)
    rays=cv2.undistortPoints(pixels.reshape(-1,1,2),k,d).reshape(-1,2)
    rays=np.c_[rays,np.ones(len(rays))]
    normal=previous[:3,2];offset=normal@previous[:3,3]
    points=rays*(offset/(rays@normal))[:,None]
    points=(points-previous[:3,3])@previous[:3,:3]
    xmin,xmax,ymin,ymax=bounds
    mask=(points[:,0]>xmin+.0004)&(points[:,0]<xmax-.0004)&(points[:,1]>ymin+.0004)&(points[:,1]<ymax-.0004)
    selection=np.flatnonzero(mask)
    if len(selection)>6000:selection=selection[np.linspace(0,len(selection)-1,6000).astype(int)]
    obj=points[selection];target=template.ravel()[selection]
    target=(target-target.mean())/max(target.std(),.05)
    r0=previous[:3,:3];t0=previous[:3,3]
    def pose(x):
        t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(np.radians(x[3:])).as_matrix()@r0
        t[:3,3]=t0+x[:3]/1000
        return t
    def residual(x):
        t=pose(x);uv=cv2.projectPoints(obj,cv2.Rodrigues(t[:3,:3])[0],t[:3,3],k,d)[0].reshape(-1,2)
        values=map_coordinates(current,uv[:,::-1].T,order=1,mode='nearest')
        values=(values-values.mean())/max(values.std(),.05)
        return values-target
    initial_x=np.r_[(initial[:3,3]-t0)*1000,np.degrees(Rotation.from_matrix(initial[:3,:3]@r0.T).as_rotvec())]
    # No target fixture pose is used. Only a local motion bound around the
    # previous image is imposed; loss of tracking returns an explicit failure.
    initial_x=np.clip(initial_x,[-19]*3+[-9]*3,[19]*3+[9]*3)
    fit=least_squares(residual,initial_x,bounds=([-20]*3+[-10]*3,[20]*3+[10]*3),
                      loss='soft_l1',f_scale=.3,max_nfev=80,ftol=1e-7,xtol=1e-6,gtol=1e-7,diff_step=.001)
    error=residual(fit.x);correlation=1-float(np.mean(error**2))/2
    if correlation<.85:raise ValueError(f'Plane tracking correlation {correlation:.4f}')
    return pose(fit.x),dict(correlation=correlation,method='six-DOF normalized photometric plane alignment',samples=len(obj),evaluations=fit.nfev)


def relative_motion(camera_from_board_a,camera_from_board_b,gamepad_from_board):
    """Two poses in one optical frame; NO world camera extrinsic is used.

    Position difference is evaluated at the controller root, not card centre;
    card rotation therefore includes the CAD lever-arm correction.
    """
    camera_from_pad_a=camera_from_board_a@np.linalg.inv(gamepad_from_board)
    camera_from_pad_b=camera_from_board_b@np.linalg.inv(gamepad_from_board)
    relative=np.linalg.inv(camera_from_pad_a)@camera_from_pad_b
    from scipy.spatial.transform import Rotation
    return dict(translation_in_reference_gamepad_mm=(relative[:3,3]*1000).tolist(),
                position_change_mm=float(np.linalg.norm(relative[:3,3])*1000),
                rotation_change_deg=float(np.degrees(Rotation.from_matrix(relative[:3,:3]).magnitude())),
                relative_transform=relative.tolist())


def check_marker_pose(image,metadata,scene,hulls,config,*,reference=None,
                      calibration_position_mm=None,calibration_angle_deg=None,
                      card_geometry_mm=None,card_angle_deg=None):
    import cv2
    k=np.asarray(metadata['K'],float).reshape(3,3)
    d=np.asarray(metadata.get('D',[0.]*5),float)
    model=SilhouetteModel(hulls,scene,k,d,image.shape)
    overlay=image.copy()
    x0,y0,x1,y1=model.roi
    target_mask=model.silhouette(model.target)
    contours,_=cv2.findContours(target_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay[y0:y1,x0:x1],contours,-1,(0,210,255),2)
    camera_from_board,quality=estimate_board(image,k,d,config)
    result=dict(method='charuco',status='not_detected',detected=False,
                message='未检测到本夹具的 ChArUco 标记卡；不输出手柄位姿',
                estimated_pose=None,components=None,uncertainty=None,acceptance=None,
                camera_3_pose=matrix_pose(model.camera),target_pose=matrix_pose(model.target),
                intrinsics=metadata,quality=quality,marker_config=config,
                measured_object='moving cradle; controller pose inferred assuming no pad/cradle slip',
                overlay_legend='yellow: target gamepad; magenta: marker-inferred gamepad; green: ChArUco corners')
    if camera_from_board is None:
        return result,overlay
    gamepad_from_board=np.asarray(config['gamepad_from_board'],float)
    pose=model.camera@camera_from_board@np.linalg.inv(gamepad_from_board)
    values=components(pose,model.target,scene)
    values['optical_depth_error_mm']=float((model.camera_inverse[:3,:3]@(pose[:3,3]-model.target[:3,3]))[2]*1000)
    # Measured synthetic envelopes are filled by the validation artifact, and
    # are explicitly conditional rather than claiming physical 95% coverage.
    precision=config.get('validated_precision',{})
    quality_profile='conservative native-raster envelope'
    resolved=next((profile for profile in (precision.get('well_resolved',{}),precision.get('well_resolved_opencv4',{}))
                   if profile.get('opencv_series') and cv2.__version__.startswith(profile['opencv_series']+'.')), {})
    quality['opencv_version']=cv2.__version__
    if resolved and quality['charuco_corners']>=resolved['minimum_corners'] and quality['reprojection_rms_px']<=resolved['maximum_reprojection_rms_px']:
        precision={**precision,**resolved}
        quality_profile='well-resolved synthetic conditional envelope; physical accuracy unverified'
    quality['precision_profile']=quality_profile
    abs_pos=float(precision.get('absolute_position_max_mm',2.))
    abs_ang=float(precision.get('absolute_rotation_max_deg',1.))
    cal_pos=5. if calibration_position_mm is None else calibration_position_mm
    cal_ang=1. if calibration_angle_deg is None else calibration_angle_deg
    geometry_pos=.5 if card_geometry_mm is None else card_geometry_mm
    geometry_ang=.3 if card_angle_deg is None else card_angle_deg
    distance=np.linalg.norm(pose[:3,3]-model.camera[:3,3])*1000
    abs_position_bound=abs_pos+cal_pos+distance*np.radians(cal_ang)+geometry_pos
    abs_angle_bound=abs_ang+cal_ang+geometry_ang
    known=(calibration_position_mm is not None and calibration_angle_deg is not None and
           card_geometry_mm is not None and card_angle_deg is not None and bool(precision))
    def verdict(error,limit,bound,ready):
        if not ready:return 'indeterminate'
        if abs(error)+bound<limit:return 'pass'
        if abs(error)-bound>limit:return 'fail'
        return 'indeterminate'
    acceptance={}
    for name,error in values['errors'].items():
        bound=abs_position_bound if name.endswith('_mm') else abs_angle_bound
        limit=1. if name.endswith('_mm') else .5
        acceptance[name]=dict(error=error,uncertainty=bound,limit=limit,
                              within_nominal=abs(error)<=limit,verdict=verdict(error,limit,bound,known))
    for name,error,limit,bound in [('position_norm_mm',values['position_error_mm'],1.,abs_position_bound),
                                  ('tilt_40_deg',values['tilt_deg']-40,.5,abs_angle_bound)]:
        acceptance[name]=dict(error=error,uncertainty=bound,limit=limit,
                              within_nominal=abs(error)<=limit,verdict=verdict(error,limit,bound,known))
    cv2.aruco.drawDetectedCornersCharuco(overlay,np.asarray(quality['corners_px'],np.float32).reshape(-1,1,2),
                                        np.asarray(quality['corner_ids'],np.int32).reshape(-1,1),(0,255,0))
    fitted=model.silhouette(pose)
    contours,_=cv2.findContours(fitted,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay[y0:y1,x0:x1],contours,-1,(255,0,255),2)
    cv2.drawFrameAxes(overlay,k,d,cv2.Rodrigues(camera_from_board[:3,:3])[0],camera_from_board[:3,3],.04)
    result.update(status='estimated',detected=True,message='标记卡位姿已测得；绝对与相对精度分别报告',
                  estimated_pose=matrix_pose(pose),camera_from_board=camera_from_board.tolist(),
                  tracking_patch=tracking_patch(image,camera_from_board,k,d,config),
                  components=values,acceptance=acceptance,
                  uncertainty=dict(absolute_position_bound_mm=abs_position_bound,
                      absolute_angle_bound_deg=abs_angle_bound,calibration_known=known,
                      calibration_position_mm=cal_pos,calibration_angle_deg=cal_ang,
                      camera_rotation_lever_mm=distance*np.radians(cal_ang),
                      card_geometry_mm=geometry_pos,card_angle_deg=geometry_ang,
                      definition='synthetic maximum plus engineering calibration/card bounds; not a physical confidence interval'))
    if reference is not None:
        if reference.get('method')!='charuco' or not reference.get('detected'):
            raise ValueError('Reference must be a detected ChArUco result')
        if reference['marker_config']['gamepad_from_board']!=config['gamepad_from_board'] or reference['marker_config']['board']!=config['board']:
            raise ValueError('Reference marker geometry differs; reacquire baseline')
        if reference['intrinsics']['K']!=metadata['K'] or reference['intrinsics'].get('D')!=metadata.get('D'):
            raise ValueError('Reference intrinsics differ; reacquire baseline')
        try:
            tracked,tracking=refine_relative(reference,image,camera_from_board,k,d,config)
        except (cv2.error, ValueError) as error:
            result['relative_to_reference']=dict(status='tracking_failed',message=str(error),creep_verdict='indeterminate',stiffness_verdict='indeterminate',reclamp_vs_reference_verdict='indeterminate')
            return result,overlay
        rel=relative_motion(np.asarray(reference['camera_from_board']),tracked,gamepad_from_board)
        rel['tracking']=tracking
        rel['camera_from_board_tracked']=tracked.tolist()
        rp=float(precision.get('relative_position_max_mm',2.))
        ra=float(precision.get('relative_rotation_max_deg',1.))
        # Extrinsics cancel exactly for norms. Card scale/extrinsic errors have
        # residual effects on a rotating lever arm, rather than an absolute shift.
        effective_mount_error=geometry_pos+np.linalg.norm(gamepad_from_board[:3,3])*1000*np.radians(geometry_ang)
        rp+=2*effective_mount_error*np.sin(np.radians(rel['rotation_change_deg'])/2)
        ready=bool(precision) and card_geometry_mm is not None and card_angle_deg is not None
        if (reference.get('quality',{}).get('reprojection_rms_px',99)>.2 or
            reference.get('quality',{}).get('charuco_corners',0)<32 or
            (reference.get('quality',{}).get('opencv_version') is not None and
             reference['quality']['opencv_version']!=cv2.__version__)):
            rp=max(rp,float(config.get('validated_precision',{}).get('relative_position_max_mm',4.1)))
            ra=max(ra,float(config.get('validated_precision',{}).get('relative_rotation_max_deg',1.4)))
        creep_checks=[verdict(rel['position_change_mm'],.5,rp,ready),verdict(rel['rotation_change_deg'],.3,ra,ready)]
        creep_verdict=('fail' if 'fail' in creep_checks else 'pass' if all(v=='pass' for v in creep_checks) else 'indeterminate')
        rel.update(position_uncertainty_mm=rp,angle_uncertainty_deg=ra,
                   reference_card_mount_unchanged_required=True,
                   measured_object='cradle/root under no-slip assumption',
                   external_camera_extrinsic_used=False,
                   creep_verdict=creep_verdict,
                   stiffness_verdict=verdict(rel['position_change_mm'],.5,rp,ready),
                   reclamp_vs_reference_verdict=verdict(rel['position_change_mm'],1.,rp,ready))
        result['relative_to_reference']=rel
    attach_sticker_measurement(result,image,k,d,config,reference)
    if result.get('sticker',{}).get('detected'):
        pixels=np.round(result['sticker']['corners_px']).astype(np.int32)
        cv2.polylines(overlay,[pixels],True,(0,255,255),2)
        cv2.putText(overlay,'temporary ID90',tuple(pixels[0]),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,255),1)
    return result,overlay


def detect_sticker(image,k,d,config):
    """Independent temporary ID; no assumed sticker-to-controller transform."""
    import cv2
    c=config['sticker'];size=c['size_m']
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,c['dictionary']))
    parameters=cv2.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.ArucoDetector(dictionary,parameters)
    corners,ids,_=detector.detectMarkers(image)
    if ids is None or c['id'] not in ids.ravel():return None,dict(detected=False,reason='temporary sticker ID not visible')
    pixels=corners[int(np.flatnonzero(ids.ravel()==c['id'])[0])].reshape(4,2).astype(float)
    return None,dict(detected=True,id=c['id'],corners_px=pixels.tolist(),
                     note='subpixel centre and planar angle; hand-applied in-plane offset is not calibrated')


def sticker_in_plane(pixels,camera_from_board,config,k,d):
    """Backproject subpixel centre/corners onto the CAD front-contact plane.

    No sticker-to-pad in-plane placement calibration is used. Normal motion and
    tilt are assumed restrained by the backrest and are not measured here.
    """
    import cv2
    from scipy.spatial.transform import Rotation
    mount=np.asarray(config['gamepad_from_board'])
    pad_plane=np.asarray(config['front_contact_plane_in_gamepad'],float)
    camera_plane=camera_from_board@np.linalg.inv(mount)@pad_plane
    unit=np.array([[-1.,-1.],[1,-1],[1,1],[-1,1]])
    homography=cv2.getPerspectiveTransform(unit.astype(np.float32),np.asarray(pixels,np.float32))
    centre=cv2.perspectiveTransform(np.zeros((1,1,2),np.float64),homography).reshape(1,2)
    image_points=np.vstack([centre,pixels])
    rays=cv2.undistortPoints(image_points.reshape(-1,1,2),k,d).reshape(-1,2)
    rays=np.c_[rays,np.ones(len(rays))]
    normal=camera_plane[:3,2];plane_offset=normal@camera_plane[:3,3]
    hits=rays*(plane_offset/(rays@normal))[:,None]
    local=(hits-camera_plane[:3,3])@camera_plane[:3,:3]
    edge=(local[2,:2]-local[1,:2])+(local[3,:2]-local[4,:2])
    angle=float(np.degrees(np.arctan2(edge[1],edge[0])))
    plane_marker=np.eye(4);plane_marker[:3,:3]=Rotation.from_euler('z',angle,degrees=True).as_matrix()
    plane_marker[:2,3]=local[0,:2]
    board_marker=np.linalg.inv(mount)@pad_plane@plane_marker
    return dict(centre_px=centre.ravel().tolist(),lateral_mm=float(local[0,0]*1000),
                slide_mm=float(local[0,1]*1000),angle_deg=angle,
                card_from_sticker=board_marker.tolist(),
                measured_dof=['left-right in plane','along jaw slide in plane','rotation about normal'],
                unmeasured_dof=['normal translation','out-of-plane tilt'],
                assumption='normal depth and plane orientation from cradle card + CAD front plane; backrest contact maintained')


def attach_sticker_measurement(result,image,k,d,config,reference=None):
    """Sticker centre/angle constrained to the card-referenced contact plane."""
    if not result['detected'] or 'sticker' not in config:return
    _,measurement=detect_sticker(image,k,d,config)
    measurement.pop('camera_from_sticker',None)
    result['sticker']=measurement
    if not measurement['detected']:return
    card=np.asarray(result.get('relative_to_reference',{}).get('camera_from_board_tracked',result['camera_from_board']))
    planar=sticker_in_plane(np.asarray(measurement['corners_px']),card,config,k,d)
    measurement['in_plane']=planar
    measurement['card_from_sticker']=planar['card_from_sticker']
    measurement['external_camera_extrinsic_used']=False
    measurement['hand_contact_is_placement_criterion']=False
    measurement['placement_controls_and_visibility_pass']=config['sticker'].get('controls_and_visibility_pass',False)
    measurement['remove_before_production_collection']=True
    if reference is None or not reference.get('sticker',{}).get('in_plane'):return
    previous=reference['sticker']['in_plane']
    lateral=planar['lateral_mm']-previous['lateral_mm'];slide=planar['slide_mm']-previous['slide_mm']
    angle=(planar['angle_deg']-previous['angle_deg']+180)%360-180
    position=float(np.hypot(lateral,slide))
    precision=config.get('validated_precision',{})
    bound=precision.get('sticker_in_plane_position_max_mm')
    angle_bound=precision.get('sticker_in_plane_angle_max_deg')
    verdict='indeterminate'
    if bound is not None and measurement['placement_controls_and_visibility_pass']:
        verdict='pass' if position+bound<1. else 'fail' if position-bound>=1. else 'indeterminate'
    change=np.linalg.inv(np.asarray(previous['card_from_sticker']))@np.asarray(planar['card_from_sticker'])
    measurement['change_from_reference']=dict(lateral_mm=lateral,slide_mm=slide,
        in_plane_translation_mm=[lateral,slide],position_change_mm=position,
        rotation_about_normal_deg=angle,rotation_change_deg=abs(angle),
        relative_transform=change.tolist(),card_from_sticker_tracked=planar['card_from_sticker'],
        position_uncertainty_mm=bound,angle_uncertainty_deg=angle_bound,verdict=verdict,
        note='2D plane-constrained measurement only; normal displacement/tilt require intact backrest contact')


def summarize_reclamping(results):
    """Maximum pairwise sticker-centre spread in the cradle-card frame."""
    from scipy.spatial.transform import Rotation
    poses=[];missing=[]
    for index,result in enumerate(results):
        marker=result.get('sticker',{})
        change=marker.get('change_from_reference',{})
        transform=change.get('card_from_sticker_tracked') or marker.get('card_from_sticker')
        if not marker.get('detected') or transform is None or change.get('status')=='tracking_failed':
            missing.append(index+1);continue
        poses.append(np.asarray(transform,float))
    if not poses:return dict(status='incomplete',count=0,missing=missing,verdict='indeterminate')
    xyz=np.array([p[:3,3]*1000 for p in poses])
    distance=float(np.linalg.norm(xyz[:,None,:]-xyz[None,:,:],axis=2).max())
    angles=[np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude()) for a in poses for b in poses]
    estimates=[r.get('marker_config',{}).get('validated_precision',{}).get('sticker_in_plane_position_max_mm') for r in results]
    bound=2*max(estimates) if estimates and all(v is not None for v in estimates) else None
    verdict='indeterminate'
    if bound is not None and len(poses)==10 and not missing and all(r.get('sticker',{}).get('in_plane') and r['sticker'].get('placement_controls_and_visibility_pass') for r in results):
        verdict='pass' if distance+bound<1. else 'fail' if distance-bound>=1. else 'indeterminate'
    return dict(status='complete' if len(poses)==10 and not missing else 'incomplete',count=len(poses),missing=missing,
                measured_object='temporary sticker centre in contact plane relative to moving cradle card',measurement_dimensions='in_plane_2D',
                positions_card_mm=xyz.tolist(),axis_ranges_mm=np.ptp(xyz,axis=0).tolist(),
                maximum_pairwise_distance_mm=distance,maximum_pairwise_rotation_deg=float(max(angles)),
                limit_mm=1.,nominal_within=distance<1.,position_uncertainty_mm=bound,verdict=verdict,
                note='in-plane spread only; normal displacement and tilt are not observed, and require maintained backrest contact')
