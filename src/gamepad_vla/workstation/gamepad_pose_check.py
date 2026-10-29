"""Camera_3 silhouette metrology. CPU only; no camera/robot side effects on import.

Lengths in metres internally; fit coordinates are fixture-frame mm and degrees.
The silhouette model uses the 0.4.1 neutral collision hulls, not a box around the
controller. Absolute certification needs independently measured calibration error.
"""
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

# X: along edge, positive world -Y; Y: outward across edge, positive world X.
F = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
DEFAULT_SPEC = Path(__file__).resolve().parents[3] / 'configs/workcell/station_g20_v4_4_raised_interim.json'


def pose_matrix(pose):
    from scipy.spatial.transform import Rotation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(np.asarray(pose)[[4, 5, 6, 3]]).as_matrix()
    t[:3, 3] = pose[:3]
    return t


def matrix_pose(t):
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return [*t[:3, 3].tolist(), *q[[3, 0, 1, 2]].tolist()]


def perturbed_pose(target, delta):
    """delta=[along-edge, outward, height (mm), pitch, roll, yaw (deg)]."""
    from scipy.spatial.transform import Rotation
    out = target.copy()
    out[:3, 3] += F @ np.asarray(delta[:3]) / 1000
    out[:3, :3] = F @ Rotation.from_euler('xyz', delta[3:], degrees=True).as_matrix() @ F.T @ target[:3, :3]
    return out


def load_snapshot(path, camera_json=None):
    import cv2
    path = Path(path)
    if path.suffix.lower() == '.zip':
        with zipfile.ZipFile(path) as archive:
            raw, meta = archive.read('image.png'), json.loads(archive.read('camera.json'))
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    else:
        image = cv2.imread(str(path))
        meta = json.loads(Path(camera_json or path.with_suffix('.json')).read_text())
    if image is None:
        raise ValueError('Cannot read image')
    h, w = image.shape[:2]
    if (meta['width'], meta['height']) != (w, h):
        raise ValueError('Image size differs from camera.json; do not crop or resize the photograph')
    if meta.get('distortion_model', 'plumb_bob') not in ('none', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported distortion model; supply a pinhole/plumb_bob camera snapshot')
    k = np.asarray(meta['K'], float).reshape(3, 3)
    d = np.asarray(meta.get('D', [0.] * 5), float)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    return image, meta, k, d


def load_hulls(urdf=None):
    """Read OBJ hull boundaries and apply neutral URDF FK, without modifying assets."""
    from gamepad_vla.kinematics import _origin
    if urdf is None:
        from xbox_gamepad_physics.resources import resource_path
        urdf = resource_path('robotwin', 'full_controls', 'mobility.urdf')
    urdf = Path(urdf)
    tree = ET.parse(urdf).getroot()
    joints = {j.find('child').get('link'): j for j in tree.findall('joint')}
    def transform(link):
        if link not in joints:
            return np.eye(4)
        j = joints[link]
        return transform(j.find('parent').get('link')) @ _origin(j)
    hulls = []
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            elem = collision.find('geometry/mesh')
            if elem is None:
                continue
            path = urdf.parent / elem.get('filename')
            if path.suffix.lower() != '.obj':
                raise ValueError('Expected the 0.4.1 CoACD OBJ resource')
            vertices, groups, faces = [], [], []
            for line in path.read_text().splitlines():
                fields = line.split()
                if not fields:
                    continue
                if fields[0] == 'v':
                    vertices.append([float(x) for x in fields[1:4]])
                elif fields[0] in ('o', 'g') and faces:
                    groups.append(faces); faces = []
                elif fields[0] == 'f':
                    faces.extend(int(x.split('/')[0]) - 1 for x in fields[1:])
            if faces:
                groups.append(faces)
            xyz = np.asarray(vertices) * np.fromstring(elem.get('scale', '1 1 1'), sep=' ')
            t = transform(link.get('name')) @ _origin(collision)
            xyz = xyz @ t[:3, :3].T + t[:3, 3]
            hulls.extend(xyz[np.unique(indices)] for indices in groups)
    if not hulls:
        raise ValueError('No controller hulls found')
    return hulls


class SilhouetteModel:
    def __init__(self, hulls, scene, k, d, image_shape):
        self.hulls = hulls
        self.vertices = np.vstack(hulls)
        self.offsets = np.cumsum([0] + [len(h) for h in hulls])
        self.scene, self.k, self.d = scene, k, d
        self.target = pose_matrix(scene['gamepad']['root_pose'])
        cam = next(c for c in scene['cameras'] if c['name'] == 'camera_3')
        self.camera = pose_matrix(cam['pose_optical'])
        self.camera_inverse = np.linalg.inv(self.camera)
        self.shape = image_shape[:2]
        uv = self.project(self.vertices, self.target)
        # Search is local to the fixture target, with room for +-15 mm / 10 deg.
        lo = np.maximum(np.floor(uv.min(0) - 45), 0).astype(int)
        hi = np.minimum(np.ceil(uv.max(0) + 45), self.shape[::-1]).astype(int)
        self.roi = (int(lo[0]), int(lo[1]), int(hi[0]), int(hi[1]))
        if np.any(hi - lo < 20):
            raise ValueError('Target gamepad outside camera image')

    def project(self, points, pose):
        import cv2
        t = self.camera_inverse @ pose
        camera_points = points @ t[:3, :3].T + t[:3, 3]
        if np.any(camera_points[:, 2] <= .01):
            raise ValueError('Gamepad behind camera')
        return cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), self.k, self.d)[0].reshape(-1, 2)

    def silhouette(self, pose, samples=False):
        import cv2
        x0, y0, x1, y1 = self.roi
        uv = self.project(self.vertices, pose) - [x0, y0]
        mask = np.zeros((y1-y0, x1-x0), np.uint8)
        edge_points = []
        for start, end in zip(self.offsets[:-1], self.offsets[1:]):
            indices = cv2.convexHull(uv[start:end].astype(np.float32), returnPoints=False).ravel() + start
            poly = uv[indices]
            cv2.fillConvexPoly(mask, np.round(poly*256).astype(np.int32), 1, shift=8)
            if samples:
                for a, b in zip(indices, np.roll(indices, -1)):
                    count = max(2, int(np.linalg.norm(uv[a]-uv[b])/1.5))
                    s = np.linspace(0., 1., count, endpoint=False)[:, None]
                    edge_points.append(self.vertices[a]*(1-s)+self.vertices[b]*s)
        # External silhouette; control holes and internal CoACD seams give no edges.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask[:] = 0
        cv2.drawContours(mask, contours, -1, 1, -1)
        if not samples:
            return mask
        pts = np.vstack(edge_points)
        pix = self.project(pts, pose) - [x0, y0]
        distance = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        val = sample_image(distance, pix)
        pts = pts[val < 1.6]
        if len(pts) > 1600:
            pts = pts[np.linspace(0, len(pts)-1, 1600).astype(int)]
        return mask, pts


def sample_image(im, points):
    from scipy.ndimage import map_coordinates
    return map_coordinates(im, np.asarray(points)[:, ::-1].T, order=1, mode='nearest')


def segment(image, model):
    """Connected non-green silhouette; reject boards, rails, ROI-truncated objects."""
    import cv2
    x0, y0, x1, y1 = model.roi
    b, g, r = image[y0:y1, x0:x1].astype(float).transpose(2, 0, 1)
    green = (g > r * 1.12 + 4) & (g > b * 1.08 + 4)
    non_green = (~green).astype(np.uint8)
    non_green = cv2.morphologyEx(non_green, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(non_green)
    target = model.silhouette(model.target)
    ty, tx = np.nonzero(target)
    area, width, height = target.sum(), np.ptp(tx)+1, np.ptp(ty)+1
    candidates = []
    for index in range(1, count):
        x, y, w, h, n = stats[index]
        if not (.50 < n/area < 1.65 and .70 < w/width < 1.35 and .70 < h/height < 1.35):
            continue
        if x <= 1 or y <= 1 or x+w >= x1-x0-1 or y+h >= y1-y0-1:
            continue
        mask = (labels == index).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask, contours, -1, 1, -1)
        overlap = np.count_nonzero(mask & target)/np.count_nonzero(mask | target)
        if overlap > .55:
            candidates.append((overlap, mask))
    if candidates:
        return max(candidates, key=lambda item: item[0])[1]
    # Below the far edge, the calibrated view includes the grey table frame.
    # Chroma-key alone joins it to the gamepad. Learn that local background
    # with GrabCut; the pose prior is an initialization, never detection proof.
    probable = cv2.dilate(target, np.ones((31,31), np.uint8))
    core = cv2.erode(target, np.ones((17,17), np.uint8))
    gc = np.full(target.shape, cv2.GC_BGD, np.uint8)
    gc[probable.astype(bool)] = cv2.GC_PR_BGD
    gc[target.astype(bool)] = cv2.GC_PR_FGD
    gc[core.astype(bool)] = cv2.GC_FGD
    gc[green] = cv2.GC_BGD
    if np.count_nonzero(gc == cv2.GC_FGD) < 100:
        return None
    cv2.setRNGSeed(20261030)
    cv2.grabCut(image[y0:y1,x0:x1], gc, None, np.zeros((1,65), np.float64),
                np.zeros((1,65), np.float64), 4, cv2.GC_INIT_WITH_MASK)
    mask = np.isin(gc, [cv2.GC_FGD, cv2.GC_PR_FGD]).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    mask[:] = 0
    cv2.drawContours(mask, [max(contours, key=cv2.contourArea)], -1, 1, -1)
    return mask


def boundary_evidence(image, mask, roi):
    """Fraction of the segmented rim supported by a real colour edge within 2px."""
    import cv2
    x0,y0,x1,y1 = roi
    crop = cv2.GaussianBlur(image[y0:y1,x0:x1], (3,3), .6).astype(np.float32)
    dx = cv2.Sobel(crop, cv2.CV_32F, 1, 0, ksize=3)/8
    dy = cv2.Sobel(crop, cv2.CV_32F, 0, 1, ksize=3)/8
    strength = np.sqrt((dx*dx+dy*dy).sum(axis=2))
    nearby = cv2.dilate((strength > 12).astype(np.uint8), np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return 0.
    rim = np.vstack(contours)[:,0,:]
    return float(nearby[rim[:,1],rim[:,0]].mean())


def fit_silhouette(observed, model):
    import cv2
    from scipy.optimize import least_squares
    outside = cv2.distanceTransform(1-observed, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    inside = cv2.distanceTransform(observed, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    signed = outside-inside
    x0, y0, _, _ = model.roi
    delta = np.zeros(6)
    steps = []
    # ICP refreshes the visible rim; inner LM uses a continuous bilinear SDF.
    for _ in range(5):
        current_pose = perturbed_pose(model.target, delta)
        current_mask, rim = model.silhouette(current_pose, samples=True)
        # Subtract the forward rasterizer's half-pixel/union rim offset. Without
        # this, a one-pixel silhouette dilation looks like millimetres of depth.
        current_signed = (cv2.distanceTransform(1-current_mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
                          - cv2.distanceTransform(current_mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE))
        offsets = sample_image(current_signed, model.project(rim, current_pose)-[x0, y0])
        if len(rim) < 30:
            raise ValueError('Too few visible silhouette points')
        def residual(x):
            uv = model.project(rim, perturbed_pose(model.target, x)) - [x0, y0]
            return sample_image(signed, uv)-offsets
        result = least_squares(residual, delta, bounds=([-15]*3+[-10]*3, [15]*3+[10]*3),
                               loss='soft_l1', f_scale=1.5, diff_step=.01,
                               max_nfev=80, xtol=1e-5, ftol=1e-5, gtol=1e-5)
        steps.append(result.nfev)
        change = np.linalg.norm(result.x-delta)
        delta = result.x
        if change < .015:
            break
    pose = perturbed_pose(model.target, delta)
    mask = model.silhouette(pose)
    iou = np.count_nonzero(mask & observed)/np.count_nonzero(mask | observed)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pix = np.vstack(contours)[:, 0, :]
    rms = float(np.sqrt(np.mean(sample_image(signed, pix)**2)))
    # Local normal-direction Jacobian. Pixels on one edge are correlated; only
    # about 20 independent rim sections are assumed, not thousands of samples.
    jac = result.jac
    _, singular, _ = np.linalg.svd(jac, full_matrices=False)
    covariance = np.linalg.pinv(jac.T@jac) * 1.5**2 * max(len(jac)/20, 1)
    sigma = np.sqrt(np.maximum(np.diag(covariance), 0))
    return dict(delta=delta, pose=pose, mask=mask, iou=float(iou), edge_rms_px=rms,
                conditional_sigma=sigma, singular_values=singular, evaluations=steps)


def components(pose, target, scene):
    from scipy.spatial.transform import Rotation
    d = F.T @ (pose[:3, 3]-target[:3, 3])*1000
    angles = Rotation.from_matrix(F.T@pose[:3, :3]@target[:3, :3].T@F).as_euler('xyz', degrees=True)
    local = F.T@pose[:3, :3]
    tilt = np.degrees(np.arctan2(local[2, 1], local[1, 1]))
    table = scene['table']
    top = table['pose'][2]+table['half_extents'][2]
    edge = table['pose'][0]+table['half_extents'][0]
    return dict(height_mm=float((pose[2, 3]-top)*1000),
                distance_inside_table_edge_mm=float((edge-pose[0, 3])*1000),
                lateral_offset_mm=float(-(pose[1, 3]-table['pose'][1])*1000),
                tilt_deg=float(tilt),
                errors=dict(height_mm=float(d[2]), distance_inside_table_edge_mm=float(-d[1]),
                            lateral_offset_mm=float(d[0]), pitch_deg=float(angles[0]),
                            roll_deg=float(angles[1]), yaw_deg=float(angles[2])),
                position_error_mm=float(np.linalg.norm(d)),
                rotation_error_deg=float(np.degrees(Rotation.from_matrix(pose[:3, :3]@target[:3, :3].T).magnitude())))


def check_pose(image, metadata, scene, hulls, *, calibration_position_mm=None,
               calibration_angle_deg=None, reference=None):
    import cv2
    k = np.asarray(metadata['K'], float).reshape(3, 3)
    d = np.asarray(metadata.get('D', [0.]*5), float)
    model = SilhouetteModel(hulls, scene, k, d, image.shape)
    target_mask = model.silhouette(model.target)
    overlay = image.copy()
    x0, y0, x1, y1 = model.roi
    def draw(mask, color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay[y0:y1, x0:x1], contours, -1, color, 2)
    draw(target_mask, (0, 210, 255))
    result = dict(status='not_detected', message='未检测到可验证的手柄轮廓', detected=False,
                  estimated_pose=None, components=None, uncertainty=None, acceptance=None,
                  target_pose=matrix_pose(model.target), camera_3_pose=matrix_pose(model.camera),
                  intrinsics=metadata, roi=list(model.roi),
                  overlay_legend='yellow: target; cyan: detected; magenta: fitted',
                  axes='lateral +world -Y; distance inside edge +world -X; height +Z; pitch around -Y, roll around +X, yaw around +Z')
    observed = segment(image, model)
    if observed is None:
        return result, overlay
    draw(observed, (255, 255, 0))
    evidence = boundary_evidence(image, observed, model.roi)
    if evidence < .70:
        result['quality'] = dict(boundary_evidence=evidence)
        result['message'] = '未检测到可验证的手柄：轮廓缺少真实图像边缘支持'
        return result, overlay
    fit = fit_silhouette(observed, model)
    result['quality'] = {k: fit[k] for k in ('iou', 'edge_rms_px', 'evaluations')}
    result['quality']['boundary_evidence'] = evidence
    # An optimizer always returns a pose. Its existence is not detection evidence.
    if fit['iou'] < .90 or fit['edge_rms_px'] > 2.5 or np.any(np.abs(fit['delta']) > [14.8]*3+[9.8]*3):
        result['message'] = '未检测到可验证的手柄：轮廓不符、遮挡或超出局部量程'
        return result, overlay
    draw(fit['mask'], (255, 0, 255))
    values = components(fit['pose'], model.target, scene)
    values['optical_depth_error_mm'] = float((model.camera_inverse[:3,:3] @ (fit['pose'][:3,3]-model.target[:3,3]))[2]*1000)
    # Conservative engineering floors, not a validated statistical CI. The
    # calibration floor is deliberately separate from synthetic recovery error.
    sigma = np.maximum(fit['conditional_sigma'], [.5, .5, .5, .25, .25, .25])
    position_floor = 5. if calibration_position_mm is None else calibration_position_mm
    angle_floor = 1. if calibration_angle_deg is None else calibration_angle_deg
    known_calibration = calibration_position_mm is not None and calibration_angle_deg is not None
    # Observed component maxima from the 16-case SAPIEN experiment, rounded
    # upward. This is a measured synthetic envelope, not a physical guarantee.
    model_envelope = np.array([.5, 5., 5.5, 1.2, 1.7, 1.5])
    u = 2*sigma + model_envelope + np.array([position_floor]*3+[angle_floor]*3)
    unc = dict(lateral_offset_mm=float(u[0]), distance_inside_table_edge_mm=float(u[1]),
               height_mm=float(u[2]), pitch_deg=float(u[3]), roll_deg=float(u[4]), yaw_deg=float(u[5]))
    criteria = {}
    for name, error in values['errors'].items():
        limit = 1. if name.endswith('_mm') else .5
        interval = unc[name]
        criteria[name] = dict(error=error, uncertainty=interval, limit=limit,
                              within_nominal=abs(error) <= limit,
                              verdict=('indeterminate' if not known_calibration else
                                       'fail' if abs(error)-interval > limit else
                                       'pass' if abs(error)+interval <= limit else 'indeterminate'))
    # Euclidean position <=1 mm and absolute 40+-.5 degrees are the requested
    # criteria; componentwise tests alone cannot establish a 3D norm bound.
    position_u = float(np.linalg.norm(u[:3]))
    tilt_error = values['tilt_deg']-40.
    tilt_u = float(u[3]+u[4]+u[5])
    for name, error, interval, limit in [('position_norm_mm', values['position_error_mm'], position_u, 1.),
                                          ('tilt_40_deg', tilt_error, tilt_u, .5)]:
        criteria[name] = dict(error=error, uncertainty=interval, limit=limit,
                              within_nominal=abs(error) <= limit,
                              verdict=('indeterminate' if not known_calibration else
                                       'fail' if abs(error)-interval > limit else
                                       'pass' if abs(error)+interval <= limit else 'indeterminate'))
    result.update(status='estimated', message='已估计手柄位姿；判据见 acceptance，精度不足时不判合格',
                  detected=True, estimated_pose=matrix_pose(fit['pose']), components=values,
                  uncertainty=dict(engineering_bounds=unc, conditional_sigma_mm_deg=sigma.tolist(),
                                   definition='2 x local correlated-edge sigma plus observed synthetic model envelope plus calibration prior/bound; not a certified CI',
                                   synthetic_model_envelope_mm_deg=model_envelope.tolist(),
                                   calibration_known=known_calibration,
                                   calibration_position_bound_mm=position_floor, calibration_angle_bound_deg=angle_floor,
                                   optical_depth_bound_mm=float(np.linalg.norm(u[:3])),
                                   note='single-view silhouette depth and out-of-image-plane tilt are weak; unknown board placement accuracy, shell mismatch, occlusion and camera movement can exceed these floors'),
                  acceptance=criteria)
    if reference is not None:
        if not reference.get('detected') or reference.get('estimated_pose') is None:
            raise ValueError('Reference result has no detected gamepad')
        if not np.allclose(reference['camera_3_pose'], result['camera_3_pose'], atol=1e-9) or not np.allclose(reference['intrinsics']['K'], metadata['K']):
            raise ValueError('Reference uses different calibration; reacquire baseline')
        drift = components(fit['pose'], pose_matrix(reference['estimated_pose']), scene)
        # Camera extrinsic common-mode error cancels only if the camera is fixed.
        prior_sigma = np.asarray(reference['uncertainty']['conditional_sigma_mm_deg'])
        bounds = 2*(sigma+prior_sigma)+2*model_envelope
        errors = drift['errors']
        bounds_named = dict(zip(['lateral_offset_mm', 'distance_inside_table_edge_mm', 'height_mm',
                                 'pitch_deg', 'roll_deg', 'yaw_deg'], bounds.tolist()))
        result['relative_to_reference'] = dict(components=drift, uncertainty=bounds_named,
                                               assumption='camera, exposure, segmentation and calibration unchanged; no independent physical accuracy validation',
                                               creep_limits=dict(angle_deg=.3, position_mm=.5),
                                               creep_nominal_within=drift['rotation_error_deg'] < .3 and drift['position_error_mm'] < .5,
                                               creep_verdict='indeterminate')
    return result, overlay


def write_result(output, result, overlay):
    import cv2
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output.with_suffix('.png')), overlay):
        raise OSError('Could not write overlay')
    output.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
