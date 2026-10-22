#!/usr/bin/env python3
"""K1-b: decompose K1-a reference heights and test mirrored FK/seeds.

Run on the compute node in an independent copy. This is a kinematic diagnostic,
not a native-contact or manipulation acceptance certificate.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from gamepad_vla.workstation.spec import load_spec, pose_to_matrix
from k1_height_common import write_json
from k1_natural_reference import ArmFeatures, solve_reference


def reflect_points(points, plane):
    out = np.array(points, dtype=float, copy=True)
    out[..., 1] = 2 * plane - out[..., 1]
    return out


def reflect_pose(pose, plane):
    # Reflect world y and local hand y; two reflections preserve handedness.
    out = np.array(pose, copy=True)
    mirror = np.diag([1., -1., 1.])
    out[:3, :3] = mirror @ out[:3, :3] @ mirror
    out[:3, 3] = reflect_points(out[:3, 3], plane)
    return out


def heights(solution, top, offset):
    if solution is None:
        return None
    return dict(wrist_above_table_mm=1000 * (solution['wrist_world_m'][2] - top),
                palm_above_table_mm=1000 * (solution['palm_world_m'][2] - top),
                median_grasp_offset_mm=1000 * offset,
                reference_clearance_mm=1000 * (solution['palm_world_m'][2] - top - offset))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, default=Path('configs/workcell/station_g20_v4_4.json'))
    parser.add_argument('--k1a', type=Path, default=Path('docs/reports/2026-10-22-gamepad-height'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = load_spec(args.spec)
    prior = json.loads((args.k1a / 'natural-reference.json').read_text())
    selected = json.loads((args.k1a / 'selected-pair.json').read_text())
    centre = np.array(selected['placement']['gamepad_centroid_world_m'])
    plane = (spec.left_arm.base_pose[1] + spec.right_arm.base_pose[1]) / 2
    top = spec.table.pose[2] + spec.table.half_extents[2]
    f = {side: ArmFeatures(getattr(spec, side + '_arm'), getattr(spec, side + '_hand'))
         for side in ('left', 'right')}
    qleft = np.array(prior['left']['reference']['solution']['q'])
    qright = np.array(prior['right']['reference']['solution']['q'])
    # Recover the obvious joint-sign mirror hypothesis by enumerating signs,
    # measured on every joint origin, not merely matching one tip position.
    samples = np.array([np.zeros(7), spec.left_arm.home_q, qleft,
                        qleft + np.array([.1, -.1, .2, -.1, .15, .1, -.1])])
    left_samples = f['left'].frames(samples)
    desired_origins = reflect_points(left_samples['joint_origins'], plane)
    desired_palms = np.array([reflect_pose(p, plane) for p in left_samples['palm']])
    sign_scores = []
    for signs in itertools.product((-1., 1.), repeat=7):
        actual = f['right'].frames(samples * signs)
        error = actual['joint_origins'] - desired_origins
        # The spherical wrist's coincident origins do not identify q6/q7 signs.
        # Include the complete palm pose so their signs are observable.
        palm_error = actual['palm'] - desired_palms
        score = np.mean(error ** 2) + .01 * np.mean(palm_error ** 2)
        sign_scores.append((float(score), signs,
                            float(np.max(np.linalg.norm(error, axis=-1)))))
    sign_scores.sort()
    signs = np.array(sign_scores[0][1])
    qmirror = qleft * signs
    frames_left = f['left'].frames(qleft)
    frames_right = f['right'].frames(qmirror)
    errors = {key: (frames_right[key][0] - reflect_points(frames_left[key][0], plane)).tolist()
              for key in ('shoulder', 'elbow', 'wrist', 'joint_origins')}
    target_palm = reflect_pose(frames_left['palm'][0], plane)
    direct_palm = frames_right['palm'][0]
    # A full pose target allows the same-R geometry's mirror residual to be
    # distinguished from an arbitrary guess at a joint-sign map.
    lo, hi = f['right'].chain.active_limits
    def residual(q):
        got = f['right'].frames(q)['palm'][0]
        angle = Rotation.from_matrix(target_palm[:3, :3] @ got[:3, :3].T).as_rotvec()
        return np.r_[got[:3, 3] - target_palm[:3, 3], .1 * angle]
    fit = least_squares(residual, np.clip(qmirror, lo, hi), bounds=(lo, hi),
                        max_nfev=400, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    mirror_hand = np.diag([1., -1., 1.])
    base_left = pose_to_matrix(spec.left_arm.base_pose)
    base_right = pose_to_matrix(spec.right_arm.base_pose)
    mounts = {side: pose_to_matrix(getattr(spec, side + '_hand').mount) for side in f}
    joint_equal = []
    for jl, jr in zip(f['left'].chain.joints, f['right'].chain.joints):
        joint_equal.append(dict(left=jl['name'], right=jr['name'],
            origin_max_abs=float(np.max(np.abs(jl['origin'] - jr['origin']))),
            axis_max_abs=float(np.max(np.abs(jl['axis'] - jr['axis'])))))
    out = dict(scope='finite FK / constrained-local-optimization symmetry diagnostic; no collision certificate',
        mirror_plane_y_m=plane, original_pad_centre_m=centre.tolist(),
        original_decomposition={side: heights(prior[side]['reference']['solution'], top, prior[side]['median_offset_m']) for side in f},
        original_home_q={side: list(getattr(spec, side + '_arm').home_q) for side in f},
        palm_local_centroids_m={side: f[side].palm_point_local.tolist() for side in f},
        palm_centroid_mirror_residual_mm=(1000 * (f['right'].palm_point_local - mirror_hand @ f['left'].palm_point_local)).tolist(),
        base_pose_reflected_position_residual_mm=(1000 * (base_right[:3, 3] - reflect_points(base_left[:3, 3], plane))).tolist(),
        base_rotation_comparison=(base_right[:3, :3].T @ np.diag([1., -1., 1.]) @ base_left[:3, :3]).tolist(),
        mount_reflection_max_abs=float(np.max(np.abs(mounts['right'][:3, :3] - mirror_hand @ mounts['left'][:3, :3] @ mirror_hand))),
        mount_translation_mirror_residual_mm=(1000 * (mounts['right'][:3, 3] - mirror_hand @ mounts['left'][:3, 3])).tolist(),
        joint_geometry_comparison=joint_equal,
        best_joint_signs=signs.tolist(), sign_map_joint_origin_max_residual_mm=1000 * sign_scores[0][2],
        sign_map_feature_errors_m=errors,
        sign_map_palm_translation_residual_mm=(1000 * (direct_palm[:3, 3] - target_palm[:3, 3])).tolist(),
        sign_map_palm_rotation_residual_deg=float(np.degrees(Rotation.from_matrix(target_palm[:3, :3] @ direct_palm[:3, :3].T).magnitude())),
        full_mirror_pose_fit=dict(q=fit.x.tolist(), success=bool(fit.success), nfev=fit.nfev,
            translation_residual_mm=(1000 * residual(fit.x)[:3]).tolist(),
            rotation_residual_deg=np.degrees(residual(fit.x)[3:] / .1).tolist()),
        reference_cases={})
    for case, padcentre in [('original_target', centre), ('symmetric_target', np.array([centre[0], plane, centre[2]]))]:
        record = {}
        for side in ('left', 'right'):
            home = np.array(spec.left_arm.home_q) * (signs if side == 'right' else 1.)
            arm = replace(getattr(spec, side + '_arm'), home_q=tuple(home))
            feature = ArmFeatures(arm, getattr(spec, side + '_hand'))
            oldsolutions = [prior['left']['reference']['solution']] + [v['solution'] for v in prior['left']['corners']]
            starts = [home] + [np.array(v['q']) * (signs if side == 'right' else 1.) for v in oldsolutions if v]
            result = solve_reference(feature, padcentre, 20, 10, warm_starts=starts)
            record[side] = dict(result=result, heights=heights(result['solution'], top, prior[side]['median_offset_m']))
            if result['solution']:
                record[side]['shared_left_median_heights'] = heights(result['solution'], top, prior['left']['median_offset_m'])
                print(case, side, record[side]['heights'], flush=True)
        out['reference_cases'][case] = record
    write_json(args.output, out)
    print('WROTE', args.output, flush=True)


if __name__ == '__main__':
    main()
