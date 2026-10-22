#!/usr/bin/env python3
"""Replay completed Task S candidates and test controls at fixed arm/palm/grip."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import Counter
import json
import multiprocessing
import os
from pathlib import Path
import time

import numpy as np

from natural_grip_fixed import FixedProblem, FixedNative
from k1b_reach_diagnostic import target_distances
from k1_height_common import write_json
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS

PROBLEMS = {}
OUTPUT = None


def load_rows(searches):
    searches = [searches] if isinstance(searches, Path) else searches
    rows = []
    for stage_index, search in enumerate(searches):
        for path in (search/'checkpoints').glob('*.json'):
            row = json.loads(path.read_text())
            row['source_stage'] = search.name
            row['original_seed_index'] = row['seed_index']
            row['seed_index'] += stage_index*10000
            rows.append(row)
    return rows


def reach_diagnostics(p, palm_in_pad):
    diagnostics = target_distances(p, palm_in_pad)
    for goal in p.goals:
        f = p.fingers[goal['finger']]
        joints = f.chain.joints
        first = next(i for i, j in enumerate(joints) if j['name'] in f.chain.active)
        frames = f.chain.link_poses(np.zeros((1, len(f.chain.active))))
        base = frames[joints[first]['child']][0, :3, 3]
        # Triangle inequality over URDF translations and every K0 contact mode.
        # Ignoring rotational/mimic limits enlarges this reachable sphere, so
        # being outside is a necessary-condition failure, not a cloud estimate.
        radius = sum(np.linalg.norm(j['origin'][:3, 3]) for j in joints[first+1:])
        radius += max(np.linalg.norm(point) for point in f.points)
        bound_valid = all(j['type'] in ('fixed', 'revolute', 'continuous') for j in joints)
        cloud_max_radius = float(np.linalg.norm(f.xyz-base, axis=1).max())
        bound_valid = bound_valid and cloud_max_radius <= radius+1e-10
        entry = diagnostics[goal['name']]
        for state in entry['states']:
            distance = float(np.linalg.norm(np.array(state['target_in_palm_m'])-base))
            state['finger_base_distance_m'] = distance
            state['radial_point_error_lower_bound_m'] = max(0., distance-radius)
        reducer = min if goal.get('alternatives') else max
        lower = reducer(s['radial_point_error_lower_bound_m'] for s in entry['states'])
        entry.update(finger_base_in_palm_m=base.tolist(),
                     chain_radius_upper_bound_m=float(radius),
                     cloud_max_radius_m=cloud_max_radius,
                     radial_point_error_lower_bound_m=lower,
                     radially_certified_unreachable=bool(bound_valid and lower > .00025),
                     radius_scope='URDF link-length triangle inequality, all K0 contact modes; fixed current palm only')
    return diagnostics


def nearest_rank(row):
    c = row['check']
    depth = max([max(0., -b['distance_m']) for b in c['violations']]+[0.])
    point = max(0., max(c['grip_point_error_m'])-.00025)
    natural = max(0., c['natural']['deviation']-1.)*.01
    missing = len({'ring', 'pinky'}-set(c['grips']))*.005
    return (not c['passed'], max(depth, point, natural, missing),
            depth+point+natural+missing, c['natural']['wrist_deviation_deg'])


def control_one(item):
    side, row = item
    p = PROBLEMS[side]
    native = FixedNative(p, OUTPUT/'models'/str(os.getpid())/side)
    x = np.array(row['x'])
    fresh = native.evaluate(x)
    out = dict(side=side, source_seed_index=row['seed_index'], x=x.tolist(),
               source_stage=row['source_stage'], original_seed_index=row['original_seed_index'],
               check=fresh, controls=[], states=[], control_diagnostics={})
    out['controls'], out['states'], out['control_diagnostics'] = native.controls(x)
    pad, palm, *_ = p.unpack(x)
    out['reach_cloud_diagnostics'] = reach_diagnostics(p, np.linalg.inv(pad)@palm)
    out['scope'] = 'Native fixed-arm/palm/ring/pinky per-state checks; finite finger IK search, no trajectories.'
    return out


def export(search, controls, output):
    rows = load_rows(search)
    cc = [json.loads(f.read_text()) for f in controls.glob('control-*.json')]
    sides = {}
    for side in ('left', 'right'):
        rr = [r for r in rows if r['side'] == side]
        checked = [r for r in cc if r['side'] == side]
        pure = [r for r in rr if r['check']['passed']]
        checked_pure = [r for r in checked if r['check']['passed']]
        checked_ids = {r['source_seed_index'] for r in checked}
        unchecked_pure = [r for r in pure if r['seed_index'] not in checked_ids]
        if checked_pure or (checked and not unchecked_pure):
            ranked = checked_pure or checked
            ranked.sort(key=lambda r: (not r['check']['passed'], -len(r['controls']),
                         *nearest_rank(r)[1:],
                         r['check'].get('human_seed_palm_target_distance_m', float('inf'))))
            best = ranked[0]
        elif rr:
            # New pure-grip candidates may arrive after the control checks.
            # Preserve their success and mark controls pending in that case.
            closest = min(unchecked_pure or rr, key=nearest_rank)
            best = dict(side=side, source_seed_index=closest['seed_index'], x=closest['x'],
                        check=closest['check'], controls=None, states=[], control_diagnostics={})
        else:
            continue
        counts = Counter(r for e in rr for r in e['check']['reasons'])
        best.update(completed_search_candidates=len(rr), native_pure_candidates=len(pure),
                    controls_checked_candidates=len(checked), rejection_counts=dict(counts),
                    candidate_status='native_natural_pure_pass' if best['check']['passed'] else 'rejected_nearest_diagnostic',
                    home_q_candidate=best['check']['arm_q'] if best['check']['passed'] else None,
                    hand_q_candidate=best['check']['hand_q'] if best['check']['passed'] else None)
        if best['check']['passed']:
            hand = dict(best['check']['hand_q'])
            for name, (parent, multiplier) in G20_MIMIC_JOINTS[side].items():
                hand[name] = hand[parent]*multiplier
            best['hand_q_all_candidate'] = [hand[n] for n in G20_JOINT_NAMES]
        else:
            best['hand_q_all_candidate'] = None
        sides[side] = best
    result = dict(sides=sides, exported_epoch=time.time(),
        joint_units='radian', hand_joint_order=G20_JOINT_NAMES,
        selection_rule='native hard pass, maximal complete K0 controls, smaller wrist deviation, then palm-target preference',
        acceptance=dict(natural_upper_tilt_deg=[10,35], natural_forearm_pitch_deg=[-10,25],
            natural_horizontal_heading_max_deg=30, elbow_forward_required=True,
            wrist_deviation='soft only', point_tolerance_m=.00025,
            digital_normal_cone_deg=30, native_ring_pinky_max_penetration_m=.001,
            other_native_penetration_numerical_tolerance_m=1e-6,
            fixtures='all collidable fixed-scene fixtures plus table, explicitly checked for every arm/hand mesh',
            table_clearance_30mm_and_camera='diagnostics only', hands='separate recordings'),
        stage_counts={stage:{side:dict(completed=sum(r['side']==side and r['source_stage']==stage for r in rows),
            native_pure=sum(r['side']==side and r['source_stage']==stage and r['check']['passed'] for r in rows))
            for side in ('left', 'right')} for stage in sorted({r['source_stage'] for r in rows})},
        fixed_spec='configs/workcell/station_g20_v4_4_raised_interim.json',
        scope='Separate hands, fixed placement, native static contact geometry; no hardware or dynamic validation.')
    write_json(output, result)
    print(json.dumps({s:dict(completed=r['completed_search_candidates'], pure=r['native_pure_candidates'],
        controls_checked=r['controls_checked_candidates'], best_passed=r['check']['passed'],
        best_controls=r['controls'], reasons=r['check']['reasons'], natural=r['check']['natural'],
        grip_error=r['check']['grip_point_error_m']) for s, r in sides.items()}), flush=True)


def main():
    global PROBLEMS, OUTPUT
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--task', type=Path, required=True)
    a.add_argument('--search', type=Path, action='append', required=True)
    a.add_argument('--output', type=Path, required=True)
    a.add_argument('--limit', type=int, default=12)
    a.add_argument('--workers', type=int, default=2)
    a.add_argument('--export-only', action='store_true')
    args = a.parse_args()
    OUTPUT = args.output
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if not args.export_only:
        rows = load_rows(args.search)
        items = []
        for side in ('left', 'right'):
            p = PROBLEMS[side] = FixedProblem(args.task, side)
            rr = [r for r in rows if r['side'] == side]
            pure = [r for r in rr if r['check']['passed']]
            if pure:
                for r in pure:
                    pad, palm, *_ = p.unpack(np.array(r['x']))
                    d = target_distances(p, np.linalg.inv(pad)@palm)
                    r['reach_cloud_score'] = sum(v['required_cloud_distance_m'] for v in d.values())
                pure.sort(key=lambda r: (r['reach_cloud_score'], r['check']['natural']['wrist_deviation_deg']))
                selected = pure[:args.limit]
            else:
                selected = sorted(rr, key=nearest_rank)[:1]
            for r in selected:
                if not (OUTPUT/f"control-{side}-{r['seed_index']:04d}.json").exists():
                    items.append((side, r))
        write_json(OUTPUT/'selected.json', [dict(side=s, seed_index=r['seed_index'],
            native_pure_pass=r['check']['passed']) for s, r in items])
        with ProcessPoolExecutor(max_workers=args.workers,
                mp_context=multiprocessing.get_context('fork')) as pool:
            for future in as_completed([pool.submit(control_one, item) for item in items]):
                row = future.result()
                write_json(OUTPUT/f"control-{row['side']}-{row['source_seed_index']:04d}.json", row)
                print(json.dumps(dict(stage='controls_done', side=row['side'],
                    seed_index=row['source_seed_index'], passed=row['check']['passed'],
                    controls=row['controls'])), flush=True)
    export(args.search, OUTPUT, OUTPUT/'candidates.json')


if __name__ == '__main__':
    main()
