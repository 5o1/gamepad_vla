"""Fixed-palm operation IK using K0's original contact-point modes and limits.

This helper proposes joint states. Native hand/pad contacts, fixed grip fingers,
whole-scene collision, natural arm posture and camera checks belong to the caller.
An accepted geometric point fit here is not a contact acceptance certificate.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from gamepad_vla.feasibility import CONE, POINT_TOL, Finger


SURFACE_OFFSETS_M = np.array([0., .000025, .000050, .000100, .000150, .000200])


def _mode_trees(finger: Finger):
    """Cache independent clouds; a nearest point can never select another mode."""
    cache = getattr(finger, '_k1b_mode_trees', None)
    if cache is None:
        count = len(finger.q)
        cache = []
        for mode in range(len(finger.points)):
            section = slice(mode * count, (mode + 1) * count)
            cache.append((cKDTree(finger.xyz[section]),
                          cKDTree(np.c_[finger.xyz[section], .015 * finger.nrm[section]])))
        finger._k1b_mode_trees = cache
    return cache


def fit_goal_modes(finger: Finger, goal: dict, palm_in_pad: np.ndarray) -> dict:
    """Return multiple per-state, per-mode K0-tolerance operation IK proposals.

    The optimizer's surface offsets move inward along the original outward pad
    normal, but final point errors are measured against the unmodified target.
    K0's 0.25 mm point tolerance and 30 degree digital-control cone are unchanged.
    The stick/trigger endpoints retain K0's lack of a normal-cone requirement.
    """
    palm = np.asarray(palm_in_pad, dtype=float)
    original_points = np.array([s['point'] for s in goal['states']], dtype=float)
    outward = np.array([s['normal'] for s in goal['states']], dtype=float)
    # Row vectors: inverse rigid transform into the fixed palm frame.
    raw_targets = (original_points - palm[:3, 3]) @ palm[:3, :3]
    inward_normals = -outward @ palm[:3, :3]
    normal_cone = goal['name'] not in ('LS', 'RS', 'LT', 'RT')
    targets = (raw_targets[:, None, :] +
               SURFACE_OFFSETS_M[None, :, None] * inward_normals[:, None, :]).reshape(-1, 3)
    normals = np.repeat(inward_normals, len(SURFACE_OFFSETS_M), axis=0)
    originals = np.repeat(raw_targets, len(SURFACE_OFFSETS_M), axis=0)
    output = dict(finger=finger.name, alternatives=bool(goal.get('alternatives', False)),
                  states=[dict(state=j, solutions=[]) for j in range(len(raw_targets))])
    for mode, (position_tree, cone_tree) in enumerate(_mode_trees(finger)):
        _, nearest = (cone_tree.query(np.c_[targets, .015 * normals], workers=1)
                      if normal_cone else position_tree.query(targets, workers=1))
        q = finger.q[nearest].copy()
        point, normal = finger.points[mode], finger.normals[mode]

        def evaluate(x):
            frames = finger.chain.forward(x)
            xyz = frames[:, :3, :3] @ point + frames[:, :3, 3]
            nrm = frames[:, :3, :3] @ normal
            cone = (np.maximum(0., CONE - np.sum(nrm * normals, axis=1))
                    if normal_cone else np.zeros(len(x)))
            return np.c_[xyz - targets, .015 * cone], xyz, nrm

        # Exactly K0 Finger.fit's DLS iteration count and numerical parameters.
        for _ in range(35):
            error, _, _ = evaluate(q)
            jac = np.empty((len(q), 4, q.shape[1]))
            for k in range(q.shape[1]):
                delta = q.copy()
                delta[:, k] += 1e-5
                jac[:, :, k] = (evaluate(delta)[0] - error) / 1e-5
            jt = jac.transpose(0, 2, 1)
            step = np.linalg.solve(jt @ jac + 1e-7 * np.eye(q.shape[1]),
                                   -(jt @ error[..., None]))[..., 0]
            q = np.clip(q + np.clip(step, -.12, .12), finger.lower, finger.upper)
        _, xyz, nrm = evaluate(q)
        errors = np.linalg.norm(xyz - originals, axis=1)
        cosines = np.sum(nrm * normals, axis=1)
        accepted = (errors <= POINT_TOL)
        if normal_cone:
            accepted &= cosines >= CONE
        for flat in np.flatnonzero(accepted):
            state, offset = divmod(int(flat), len(SURFACE_OFFSETS_M))
            output['states'][state]['solutions'].append(dict(
                q=q[flat].tolist(), mode=mode, point_error_m=float(errors[flat]),
                normal_cos=float(cosines[flat]), surface_offset_m=float(SURFACE_OFFSETS_M[offset])))
    return output
