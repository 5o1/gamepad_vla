#!/usr/bin/env python3
"""Finite synthetic recovery experiment; SAPIEN mode reuses the collection backend.

No ROS, robot SDK or training. Output is an explicit per-run directory only.
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from gamepad_vla.workstation.gamepad_pose_check import (
    DEFAULT_SPEC, SilhouetteModel, check_pose, load_hulls, matrix_pose, perturbed_pose,
    pose_matrix, write_result,
)


def cases():
    rows = [('nominal', np.zeros(6))]
    for axis, label in enumerate(['lateral', 'outward', 'height', 'pitch', 'roll', 'yaw']):
        for sign in [-1, 1]:
            d = np.zeros(6); d[axis] = sign * (3 if axis < 3 else 2)
            rows.append((f'{label}_{sign:+d}', d))
    rows.extend([('combined_a', np.array([3,-3,3,2,-2,2.])),
                 ('combined_b', np.array([-3,3,-3,-2,2,-2.])),
                 ('combined_noise', np.array([2,-3,-2,-2,1,2.]))])
    return rows


def main():
    from gamepad_vla.site import configure_runtime
    configure_runtime()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', choices=['projection', 'sapien'], default='projection')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--urdf', type=Path)
    p.add_argument('--spec', type=Path, default=DEFAULT_SPEC)
    p.add_argument('--limit', type=int)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    scene = json.loads(a.spec.read_text())
    camera = next(c for c in scene['cameras'] if c['name']=='camera_3')
    intr = camera['intrinsics']
    k = np.array([[intr['fx'],0,intr['cx']],[0,intr['fy'],intr['cy']],[0,0,1.]])
    meta = dict(width=intr['width'], height=intr['height'], K=k.ravel().tolist(),
                D=intr['distortion'], distortion_model='plumb_bob', serial='camera_3', source=a.source)
    hulls = load_hulls(a.urdf)
    model = SilhouetteModel(hulls, scene, k, np.array(meta['D']), (intr['height'],intr['width'],3))
    backend = camera_render = None
    if a.source == 'sapien':
        import sapien
        from gamepad_vla.workstation.sapien_backend import SapienBackend
        from gamepad_vla.workstation.spec import load_spec
        from gamepad_vla.workstation.render_review import create_spec_camera
        sapien.render.set_camera_shader_dir('default')
        spec = load_spec(a.spec)
        # Same public robotwin.create_scene / load_into_scene, appearance and
        # camera helper as operator_view. No second scene/material implementation.
        backend = SapienBackend(spec, render=True, work_dir=a.output/'render_work')
        camera_render = create_spec_camera(backend, next(c for c in spec.cameras if c.name=='camera_3'))
        # Keep other scene geometry intact. Arms can occlude the gamepad; do not
        # hide them or use a privileged segmentation mask in the estimator.
    rows = []
    rng = np.random.default_rng(20261030)
    for name, delta in cases()[:a.limit]:
        truth = perturbed_pose(model.target, delta)
        if backend is not None:
            import sapien
            q = matrix_pose(truth)
            backend.gamepad.articulation.set_root_pose(sapien.Pose(q[:3], q[3:]))
            backend.scene.update_render()
            camera_render.take_picture()
            from gamepad_vla.workstation.collection_camera import color_to_rgb8
            image = cv2.cvtColor(color_to_rgb8(camera_render.get_picture('Color')), cv2.COLOR_RGB2BGR)
        else:
            image = np.full((intr['height'],intr['width'],3), [60,145,65], np.uint8)
            x0,y0,x1,y1 = model.roi
            mask = model.silhouette(truth)
            image[y0:y1,x0:x1][mask.astype(bool)] = [175,180,182]
        if name == 'combined_noise':
            image = cv2.GaussianBlur(image, (3,3), .65)
            image = np.clip(image.astype(float)+rng.normal(0,2,image.shape),0,255).astype(np.uint8)
        cv2.imwrite(str(a.output/(name+'.input.png')), image)
        started = time.monotonic()
        result, overlay = check_pose(image, meta, scene, hulls)
        write_result(a.output/name, result, overlay)
        row = dict(case=name, truth_delta_mm_deg=delta.tolist(), detected=result['detected'],
                   elapsed_s=time.monotonic()-started)
        if result['detected']:
            from scipy.spatial.transform import Rotation
            actual = pose_matrix(result['estimated_pose'])
            row.update(position_error_mm=float(np.linalg.norm(actual[:3,3]-truth[:3,3])*1000),
                       rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual[:3,:3]@truth[:3,:3].T).magnitude())),
                       recovered_errors=result['components']['errors'], quality=result['quality'])
        rows.append(row)
        print(json.dumps(row), flush=True)
    summary = dict(source=a.source, cases=rows, detected=sum(r['detected'] for r in rows), total=len(rows),
                   scope='Synthetic recovery conditional on exact asset geometry, intrinsics and camera pose; not real-world metrology accuracy')
    valid = [r for r in rows if r['detected']]
    if valid:
        summary['accuracy'] = {key:dict(median=float(np.median([r[key] for r in valid])),
                                      p95=float(np.percentile([r[key] for r in valid],95)),
                                      max=float(max(r[key] for r in valid)))
                               for key in ['position_error_mm','rotation_error_deg']}
    (a.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    # A separate negative image proves the estimator does not invent a pose.
    blank = np.full((intr['height'],intr['width'],3),[60,145,65],np.uint8)
    negative, overlay = check_pose(blank, meta, scene, hulls)
    write_result(a.output/'empty_green', negative, overlay)
    assert negative['status']=='not_detected' and negative['estimated_pose'] is None
    if summary['detected'] != summary['total']:
        raise SystemExit('Some synthetic controllers were not detected; see summary.json')


if __name__ == '__main__':
    main()
