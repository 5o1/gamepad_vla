#!/usr/bin/env python3
"""Task X: bounded placement scan reusing Task S proposals/native acceptance.

Only per-run copies of scene/task JSON are written. No hardware or dynamics.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
import json
import multiprocessing
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh

from gamepad_vla.feasibility import Pad, mesh_for_geom, transform
from gamepad_vla.kinematics import SerialChain
from gamepad_vla.site import workspace_root
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix, matrix_to_pose, FixtureSpec, spec_to_dict
from natural_grip_fixed import FixedProblem, FixedNative, POST
from check_natural_grip_fixed import nearest_rank, reach_diagnostics
from k1_height_common import support_point, write_json

PROBLEMS = {}
BANK = {}
ARGS = None
BASE = None
EDGE, MID, TOP = .7184, .6252, .767


def placement(spec, d, h, delta):
    centre = np.array([EDGE-d/1000, MID, TOP+h/1000])
    camera = next(c for c in spec.cameras if c.name == 'camera_3')
    toward = np.array(camera.pose_optical[:3])-centre
    z = toward/np.linalg.norm(toward)
    x = np.cross([0., 0., 1.], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    frame = np.eye(4)
    frame[:3, :3] = np.column_stack((x, y, z)) @ Rotation.from_euler('x', delta, degrees=True).as_matrix()
    frame[:3, 3] = centre
    return frame


def fixtures(frame):
    support = transform(support_point(), frame)
    # Vertical post: 30 mm in world x, 40 mm along the table edge.
    post = FixtureSpec(POST, 'box', (*support[:2], (TOP+.020+support[2])/2, 1., 0., 0., 0.),
        True, 'design', 'Task X vertical 30x40 mm post, ending at K0 rear-shell support sample.',
        (.015, .020, (support[2]-TOP-.020)/2))
    # Conservative full-width extension of the 220x60x20 mm clamping band.
    lo = min(EDGE-.060, support[0]-.015)
    # At d=0 the rear-shell point lies just beyond the edge. Include that
    # small outboard foot too, so the post is actually supported by the base.
    hi = max(EDGE, support[0]+.015)
    base = FixtureSpec('gamepad_clamp_base', 'box', ((lo+hi)/2, MID, TOP+.010, 1., 0., 0., 0.),
        True, 'design', 'Task X 220x60x20 mm edge band extended to contain the complete post foot, inboard or outboard.',
        ((hi-lo)/2, .110, .010))
    return post, base


def at_placement(spec, frame):
    keep = tuple(f for f in spec.fixtures if f.name not in (POST, 'gamepad_clamp_base', 'gamepad_central_support'))
    return replace(spec, gamepad=replace(spec.gamepad, root_pose=tuple(matrix_to_pose(frame))),
                   fixtures=(*keep, *fixtures(frame)))


def visual_vertices(resources):
    """Same glTF scene graphs, scale and +90 degree X conversion as robotwin.visual.

    At neutral all moving-part pivot translations cancel their URDF link poses.
    Verify that cancellation rather than assuming it for an unknown binding.
    """
    root = Path(resources)
    directory = root/'common/xbox_one_s/visual'
    manifest = json.loads((directory/'manifest.json').read_text())
    binding = json.loads((root/'robotwin/full_controls/manifest.json').read_text())['visual_binding']
    urdf = root/'robotwin/full_controls/mobility.urdf'
    conversion = Rotation.from_euler('x', 90, degrees=True).as_matrix()
    points = []; checks = []
    for part in manifest['parts']:
        mesh = trimesh.load(directory/part['file'], force='scene')
        mesh = mesh.dump(concatenate=True)
        vertices = np.asarray(mesh.vertices)*manifest['meters_per_native_unit'] @ conversion.T
        record = binding['moving_parts'].get(part['name'])
        if record:
            chain = SerialChain(urdf, record['link'])
            link = chain.forward(np.zeros((1, len(chain.active))))[0]
            local = vertices-np.array(record['pivot_m_source_frame'])
            vertices = transform(local, link)
            checks.append(float(np.max(np.abs(link[:3, 3]-record['pivot_m_source_frame']))))
        points.append(vertices)
    return np.concatenate(points), dict(parts=len(points), vertices=sum(len(p) for p in points),
        maximum_neutral_pivot_translation_error_m=max(checks),
        geometry='all original visual glTF scene vertices, neutral controls; no boxes or hulls')


def project(spec, vertices, frame):
    camera = next(c for c in spec.cameras if c.name == 'camera_3')
    cf = pose_to_matrix(camera.pose_optical); k = camera.intrinsics
    world = transform(vertices, frame)
    xyz = (world-cf[:3, 3]) @ cf[:3, :3]
    uv = np.c_[k.fx*xyz[:, 0]/xyz[:, 2]+k.cx, k.fy*xyz[:, 1]/xyz[:, 2]+k.cy]
    margins = [float(uv[:, 0].min()), float(k.width-1-uv[:, 0].max()),
               float(uv[:, 1].min()), float(k.height-1-uv[:, 1].max())]
    return dict(passed=bool(min(margins)>=20 and xyz[:, 2].min()>0),
                margin_px=min(margins), margins_lrtb_px=margins,
                bbox_px=[uv.min(axis=0).tolist(), uv.max(axis=0).tolist()], minimum_depth_m=float(xyz[:, 2].min()))


def arm_score(check):
    m = check['natural']
    values = [(m['upper_tilt_deg']-20)/15, (m['elbow_angle_deg']-95)/25,
              m['forearm_pitch_deg']/25, m['forearm_heading_deg']/30]
    return float(np.mean(np.square(values)) + .1*(m['wrist_deviation_deg']/90)**2)


def spec_record(spec):
    record = spec_to_dict(spec)
    if record.get('appearance') and not Path(record['appearance']).is_absolute():
        record['appearance'] = str((spec.source_dir/record['appearance']).resolve())
    return record


def set_problem(p, frame):
    p.spec = at_placement(BASE, frame)
    p.fixed_pad = frame
    p.world_centre = transform(p.pad_centre, frame)
    p.post = next(f for f in p.spec.fixtures if f.name == POST)
    p.post_frame = pose_to_matrix(p.post.pose)
    p.fixed4 = np.r_[frame[:2, 3], frame[2, 3]-TOP, np.arctan2(-frame[2, 1], frame[2, 2])]


def arm_ik(p, source, frame, ideal=True):
    oldpad = pose_to_matrix(source['check']['gamepad_pose'])
    oldpalm = pose_to_matrix(source['check']['palm_world'])
    target = frame @ np.linalg.inv(oldpad) @ oldpalm
    q = np.array(source['check']['arm_q'])
    # Archived S and current v4.5 have identical corrected mounting frames.
    # LinkerTA negation/offset has already been applied; no extra J1 correction.
    lo, hi = p.feature.chain.active_limits
    q = np.clip(q, lo+1e-8, hi-1e-8)
    def residual(a):
        if time.time() > ARGS.deadline: raise TimeoutError('Task X compute deadline')
        palm = p.feature.frames(a)['palm'][0]
        rot = Rotation.from_matrix(target[:3, :3] @ palm[:3, :3].T).as_rotvec()
        m = p.feature.measure(a, p.world_centre)
        preference = np.array([m['upper_tilt_deg'][0]-20, m['elbow_angle_deg'][0]-95,
                               m['forearm_pitch_deg'][0], m['forearm_heading_deg'][0]])*.0002 if ideal else []
        return np.r_[(palm[:3, 3]-target[:3, 3])*1000, rot*20,
                     p.natural_residual(a)*4, preference]
    fit = least_squares(residual, q, bounds=(lo, hi), max_nfev=100,
                        ftol=1e-9, xtol=1e-9, gtol=1e-9)
    hq = [source['check']['hand_q'][n] for n in p.names]
    return np.r_[p.fixed4, fit.x, hq], dict(nfev=fit.nfev, palm_residual=float(np.linalg.norm(residual(fit.x)[:6])))


def job(item):
    row, side = item
    start = time.time(); p = PROBLEMS[side]
    frame = pose_to_matrix(row['pose']); set_problem(p, frame)
    native = FixedNative(p, ARGS.output/'models'/str(os.getpid())/row['id']/side)
    attempts = []
    sources = BANK[side][:ARGS.seeds]
    for i, source in enumerate(sources):
        x, ik = arm_ik(p, source, frame)
        check = native.evaluate(x)
        attempts.append(dict(seed=source['label'], method='rigid-grasp-arm-IK', x=x.tolist(), check=check, ik=ik))
    # One bounded Task S full-arm/hand proposal per point, allowing changed
    # relative palms/control reach, rather than merely translating old grasps.
    if ARGS.explore:
        anchor = min(attempts, key=lambda r: nearest_rank(r))
        x0 = np.array(anchor['x'])
        p.reach_scale = 3.; p.natural_weight = 3.
        def residual(z, hard=False):
            if time.time() > ARGS.deadline: raise TimeoutError('Task X compute deadline')
            x = np.r_[p.fixed4, z]
            if hard: return native.feasibility_residual(x, x0)
            return p.residual(x, x0, 'all')
        fit = least_squares(residual, x0[4:], bounds=(p.lower[4:], p.upper[4:]),
            x_scale=.5, diff_step=1e-4, max_nfev=ARGS.nfev, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        x = np.r_[p.fixed4, fit.x]; check = native.evaluate(x)
        attempts.append(dict(seed=anchor['seed'], method='Task-S-reach-all', x=x.tolist(), check=check, nfev=fit.nfev))
        if not check['passed'] and max(check['grip_point_error_m']) < .005 and check['natural']['deviation'] < 1.3:
            fit = least_squares(lambda z: residual(z, True), x[4:], bounds=(p.lower[4:], p.upper[4:]),
                x_scale=.5, diff_step=1e-3, max_nfev=ARGS.repair_nfev, ftol=1e-9, xtol=1e-9, gtol=1e-9)
            x = np.r_[p.fixed4, fit.x]; check = native.evaluate(x)
            attempts.append(dict(seed=anchor['seed'], method='Task-S-native-feasibility', x=x.tolist(), check=check, nfev=fit.nfev))
    for attempt in attempts:
        attempt['arm_score'] = arm_score(attempt['check'])
    pure = [a for a in attempts if a['check']['passed']]
    best = min(pure, key=lambda a:a['arm_score']) if pure else min(attempts, key=nearest_rank)
    out = dict(placement=row, side=side, attempts=attempts, best=best, found=bool(pure),
               elapsed_seconds=time.time()-start, pure_attempts=len(pure))
    write_json(ARGS.output/'results'/f"{row['id']}-{side}.json", out)
    return dict(id=row['id'], side=side, found=bool(pure), attempts=len(attempts),
                arm_score=best['arm_score'], reasons=best['check']['reasons'], seconds=out['elapsed_seconds'])


def prepare(args):
    base = load_spec(args.spec)
    v, provenance = visual_vertices(args.visual_resources)
    # Also record the native SDF triangle envelope for comparison, independently.
    pad = Pad(args.resources); pad.state({})
    collision = np.concatenate([mesh_for_geom(pad.model, pad.data, g).vertices
        for g in range(pad.model.ngeom) if pad.model.geom_dataid[g]>=0])
    rows = []
    for d in (0,25,50,75,100,125):
        for h in (190,220,250,280):
            for delta in (0,-10,10):
                f = placement(base, d, h, delta)
                row = dict(id=f'd{d:03d}-h{h:03d}-t{delta:+03d}', d_mm=d, h_mm=h,
                    tilt_delta_deg=delta, tilt_from_vertical_deg=float(np.degrees(np.arccos(f[2,2]))),
                    pose=list(matrix_to_pose(f)), visibility=project(base,v,f),
                    native_mesh_visibility=project(base,collision,f))
                rows.append(row)
    write_json(args.output/'grid.json', dict(rows=rows, visual_provenance=provenance,
        camera=asdict(next(c for c in base.cameras if c.name=='camera_3')),
        centre_definition='gamepad asset root, same design datum as Task S; xyz=[.7184-d/1000,.6252,.767+h/1000]',
        margin_pixels=20, prepared_epoch=time.time()))
    first = next(r for r in rows if r['visibility']['passed'])
    spec = at_placement(base, pose_to_matrix(first['pose']))
    write_json(args.output/'template-spec.json', spec_record(spec))
    task = json.loads(Path('configs/tasks/fixed_dpad_causal.json').read_text())
    task['workcell'] = str((args.output/'template-spec.json').resolve())
    task['resource_root'] = str(args.resources)
    task.pop('gamepad_root_pose', None)
    write_json(args.output/'task.json', task)
    print(json.dumps(dict(visible=sum(r['visibility']['passed'] for r in rows),total=len(rows),visual=provenance)),flush=True)


def main():
    global ARGS, BASE, PROBLEMS, BANK
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('stage', choices=('prepare','solve','controls'))
    a.add_argument('--output',type=Path,required=True)
    a.add_argument('--spec',type=Path,default=Path('configs/workcell/station_g20_v4_5.json'))
    a.add_argument('--resources',type=Path,default=workspace_root()/'assets/xbox_gamepad_physics_mujoco_84f43e6/resources')
    a.add_argument('--visual-resources',type=Path,default=workspace_root()/'assets/xbox_gamepad_physics_v0.4.1/resources')
    a.add_argument('--source',type=Path,default=workspace_root()/'runs/natural-grip-s-20261023/final-controls')
    a.add_argument('--workers',type=int,default=4)
    a.add_argument('--seeds',type=int,default=3)
    a.add_argument('--explore',action='store_true')
    a.add_argument('--nfev',type=int,default=65)
    a.add_argument('--repair-nfev',type=int,default=30)
    a.add_argument('--deadline',type=float,default=time.time()+3000)
    a.add_argument('--limit',type=int,default=0)
    args = ARGS = a.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    if args.stage=='prepare': prepare(args); return
    BASE = load_spec(args.spec)
    for side in ('left','right'):
        PROBLEMS[side] = FixedProblem(args.output/'task.json',side)
        BANK[side] = []
        for path in args.source.glob(f'control-{side}-*.json'):
            s = json.loads(path.read_text())
            if s['check']['passed']:
                s['label'] = path.stem; BANK[side].append(s)
        BANK[side].sort(key=lambda s:(-len(s['controls']),s['check']['natural']['wrist_deviation_deg']))
    rows = json.loads((args.output/'grid.json').read_text())['rows']
    rows = [r for r in rows if r['visibility']['passed']]
    if args.stage=='solve':
        items = [(r,s) for r in rows for s in ('left','right')
                 if not (args.output/'results'/f"{r['id']}-{s}.json").exists()]
        if args.limit: items=items[:args.limit]
        with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
            for f in as_completed([pool.submit(job,item) for item in items]):
                print(json.dumps(f.result()),flush=True)
    else:
        # Controls for the best verified posture at every visible point/side.
        items = [(r,s) for r in rows for s in ('left','right')
                 if (args.output/'results'/f"{r['id']}-{s}.json").exists()
                 and not (args.output/'controls'/f"{r['id']}-{s}.json").exists()]
        if args.limit: items=items[:args.limit]
        with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
            for f in as_completed([pool.submit(control_job,item) for item in items]):
                print(json.dumps(f.result()),flush=True)


def control_job(item):
    row,side=item; p=PROBLEMS[side];set_problem(p,pose_to_matrix(row['pose']))
    result=json.loads((ARGS.output/'results'/f"{row['id']}-{side}.json").read_text())
    best=result['best']; x=np.array(best['x'])
    if not result['found']:
        out=dict(placement=row,side=side,found=False,check=best['check'],x=best['x'],
                 arm_score=best['arm_score'],controls=None,checked_candidates=[])
        write_json(ARGS.output/'controls'/f"{row['id']}-{side}.json",out)
        return dict(id=row['id'],side=side,found=False,controls=None)
    native=FixedNative(p,ARGS.output/'control-models'/str(os.getpid())/row['id']/side)
    checked=[]
    for attempt in sorted((a for a in result['attempts'] if a['check']['passed']),key=lambda a:a['arm_score']):
        x=np.array(attempt['x']);check=native.evaluate(x)
        # Chain-length exclusion only skips logically impossible controls; all
        # non-excluded controls use Task S's unchanged finite IK/native workflow.
        diagnostics=reach_diagnostics(p,np.linalg.inv(p.fixed_pad)@p.unpack(x)[1])
        excluded={name for name,d in diagnostics.items() if d['radially_certified_unreachable']}
        goals=p.goals; p.goals=[g for g in goals if g['name'] not in excluded]
        controls,states,cd=native.controls(x) if check['passed'] else ([],[],{})
        p.goals=goals
        checked.append(dict(seed=attempt['seed'],method=attempt['method'],x=x.tolist(),check=check,
            arm_score=arm_score(check),controls=controls,states=states,control_diagnostics=cd,
            reach_diagnostics=diagnostics,chain_length_excluded=sorted(excluded),found=check['passed']))
    checked.sort(key=lambda c:(not c['found'],c['arm_score'],-len(c['controls'])))
    out=dict(placement=row,side=side,**checked[0],checked_candidates=checked,
             maximum_controls_found=max(len(c['controls']) for c in checked))
    write_json(ARGS.output/'controls'/f"{row['id']}-{side}.json",out)
    return dict(id=row['id'],side=side,found=out['found'],controls=out['controls'],
                maximum_controls_found=out['maximum_controls_found'],checked_candidates=len(checked))


if __name__=='__main__': main()
