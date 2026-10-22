#!/usr/bin/env python3
"""K1-a layered position/height scan. Execute on the configured compute node only.

Relative stage reuses K0 point-IK arrays, then checks ALL K0 resting alternatives
against the native pad geometry. World stage changes no relative samples or
contact thresholds. It applies whole-mesh table clearance, all fixtures, multi-
start arm IK, the calibrated camera gate and the current natural-pose decision.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from gamepad_vla.feasibility import (Pad, Finger, FINGERS, CollisionScene, transform)
from gamepad_vla.kinematics import SerialChain
from gamepad_vla.tasks.fixed_gamepad import load_task
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix, matrix_to_pose
from k1_height_common import (SIDES, EXCLUDED, TABLE_GAP, MeshGeometry, CameraGate,
    relative_check, defer_pad_contacts, reject_reason, support_point, SUPPORT_NAME, with_support, set_placement,
    placements, write_json)


def relative(args):
    start = time.monotonic()
    cfg = load_task(args.task); spec = load_spec(cfg['workcell'])
    side = args.side; hand = getattr(spec,side+'_hand')
    cache = np.load(args.k0/'kinematics.npz')
    previous = json.loads((args.k0/'result.json').read_text())
    pars = previous['parameters']
    if pars['palm_samples'] != 4096 or pars['fk_samples_per_finger'] != 100000:
        raise ValueError('K1 requires the formal K0 cache, not a pilot')
    args.output.mkdir(parents=True,exist_ok=True)
    pad = Pad(cfg['resource_root']); goals = [g for g in pad.targets(side) if g['name'] not in EXCLUDED]
    rng = np.random.default_rng(21+(side=='right'))
    fingers = {f:Finger(hand.urdf,f,100000,rng) for f in FINGERS}
    scene = CollisionScene(spec,cfg['resource_root'],args.output)
    scene.set_pad([0,0,0,1,0,0,0],{})
    bank = []; rejects = Counter(); baseline_attempts = 0
    for row,pi in enumerate(cache['grip_ids']):
        pi = int(pi); palm = cache['palms'][pi]
        qgrip = {**dict(zip(fingers['ring'].chain.active,cache['ringq'][pi])),
                 **dict(zip(fingers['pinky'].chain.active,cache['pinkq'][pi]))}
        rests = []
        for j in range(12):
            q = qgrip.copy()
            for name in ('thumb','index','middle'):
                f = fingers[name]
                values = f.lower if j==0 else f.upper if j==1 else f.q[(pi*13+j*997)%len(f.q)]
                q.update(zip(f.chain.active,values))
            rests.append(q)
        # Keep K0 candidate generation, including its middle-key resting options.
        # They are NOT counted as controls in K1.
        for gi,g in enumerate(previous['goals']):
            if cache['coverage'][pi,gi]:
                q = rests[0].copy()
                j = int(np.where(cache[g['name']+'_ok'][row])[0][0])
                q.update(zip(fingers[g['finger']].chain.active,cache[g['name']+'_q'][row,j]))
                rests.insert(0,q)
        accepted = []; seen = set()
        for bi,q in enumerate(rests):
            key = tuple(sorted(q.items()))
            if key in seen: continue
            seen.add(key); baseline_attempts += 1
            scene.set_pad([0,0,0,1,0,0,0],{}); scene.set_hand(side,palm,q)
            clear,bad,grips,_ = relative_check(scene,side)
            if not clear or grips != {'ring','pinky'}:
                rejects[reject_reason(bad,grips)] += 1
                if any(not any('_'+f+'_' in b for f in ('thumb','index','middle')
                                   for b in (v['a'],v['b'])) for v in bad):
                    break
                continue
            operations = {}; control_rejects = Counter()
            for g in goals:
                states = []; good = []
                for j,state in enumerate(g['states']):
                    if not cache[g['name']+'_ok'][row,j]:
                        good.append(False); control_rejects[g['name']+':point_ik'] += 1; continue
                    sq = q.copy()
                    sq.update(zip(fingers[g['finger']].chain.active,cache[g['name']+'_q'][row,j]))
                    scene.set_pad([0,0,0,1,0,0,0],state['gamepad_q']);scene.set_hand(side,palm,sq)
                    ok,bad,grips,active = relative_check(scene,side,g['part'],g['finger'])
                    ok = ok and grips=={'ring','pinky'} and active
                    good.append(ok)
                    if ok:
                        states.append(dict(state=j,hand_q=sq,gamepad_q=state['gamepad_q']))
                    else: control_rejects[g['name']+':'+reject_reason(bad,grips,active)] += 1
                passed = any(good) if g.get('alternatives') else all(good)
                if passed: operations[g['name']] = states
            accepted.append(dict(baseline_index=bi,resting_hand_q=q,
                                  operations=operations,control_rejections=dict(control_rejects)))
        if accepted:
            bank.append(dict(palm_index=pi,palm_in_pad=list(matrix_to_pose(palm)),baselines=accepted))
        if row%300==0:
            print(json.dumps(dict(stage='relative',side=side,checked=row,valid_palms=len(bank))),flush=True)
    pad.state({})
    mesh = pad.mesh('shell')
    out = dict(side=side,sampled_palms=len(cache['palms']),grip_point_ik=len(cache['grip_ids']),
        point_ik_per_control={g['name']:int(cache[g['name']+'_ok'].all(axis=1).sum()
            if not g.get('alternatives') else cache[g['name']+'_ok'].any(axis=1).sum()) for g in goals},
        relative_grip_palms=len(bank),relative_baselines=sum(len(b['baselines']) for b in bank),
        relative_per_control={g['name']:sum(any(g['name'] in b['operations'] for b in e['baselines'])
                                           for e in bank) for g in goals},
        baseline_attempts=baseline_attempts,baseline_rejections=dict(rejects),goals=goals,bank=bank,
        shell_low_m=float(mesh.vertices[:,2].min()),support_point_m=support_point().tolist(),
        k0_parameters=pars,elapsed_seconds=time.monotonic()-start,
        method='K0 samples/point IK/native contacts; all original resting alternatives retained; '
               'no relaxed contact, normal, joint-limit or motion thresholds; middle keys excluded from coverage')
    write_json(args.output/'relative.json',out)
    print(json.dumps({k:out[k] for k in ('side','relative_grip_palms','relative_baselines','relative_per_control','elapsed_seconds')}),flush=True)


def multistart_ik(arm,hand,palms,seed,iterations=100):
    """K0's six DLS starts, retaining every accurate solution for collision/natural ranking."""
    chain = SerialChain(arm.urdf,arm.tip_link)
    zero_links = chain.link_poses(np.zeros((1,7)))
    zero = np.eye(4) if chain.root_link==arm.base_link else zero_links[arm.base_link][0]
    targets = zero@np.linalg.inv(pose_to_matrix(arm.base_pose))@palms@np.linalg.inv(pose_to_matrix(hand.mount))
    lo,hi = chain.active_limits
    children = [j['child'] for j in chain.joints if j['name'] in chain.active]
    axes = [j['axis'] for j in chain.joints if j['name'] in chain.active]
    rng = np.random.default_rng(seed)
    answers = []
    for start in range(6):
        q = np.tile(arm.home_q,(len(palms),1)) if start==0 else rng.uniform(lo,hi,(len(palms),7))
        for _ in range(iterations):
            poses = chain.link_poses(q); tip = poses[arm.tip_link]
            e = np.c_[targets[:,:3,3]-tip[:,:3,3],
                Rotation.from_matrix(targets[:,:3,:3]@tip[:,:3,:3].transpose(0,2,1)).as_rotvec()]
            jac = np.empty((len(q),6,7))
            for k,(child,axis) in enumerate(zip(children,axes)):
                v = poses[child][:,:3,:3]@axis
                jac[:,:3,k] = np.cross(v,tip[:,:3,3]-poses[child][:,:3,3]); jac[:,3:,k] = v
            step = jac.transpose(0,2,1)@np.linalg.solve(jac@jac.transpose(0,2,1)+1e-4*np.eye(6),e[...,None])
            q = np.clip(q+np.clip(step[...,0],-.25,.25),lo,hi)
        f = chain.forward(q)
        pe = np.linalg.norm(targets[:,:3,3]-f[:,:3,3],axis=1)
        re = np.linalg.norm(Rotation.from_matrix(targets[:,:3,:3]@f[:,:3,:3].transpose(0,2,1)).as_rotvec(),axis=1)
        answers.append((q,(pe<.0005)&(re<.005),pe,re))
        print(json.dumps(dict(stage='ik_start',start=start,targets=len(q),solved=int(answers[-1][1].sum()))),flush=True)
    return answers


def world(args):
    from k1_natural_reference import ArmFeatures
    start = time.monotonic(); args.output.mkdir(parents=True,exist_ok=True)
    cfg = load_task(args.task); spec = load_spec(cfg['workcell']); side = args.side
    rel = json.loads((args.relative/'relative.json').read_text()); bank = rel['bank']
    support = np.array(rel['support_point_m']); goals = rel['goals']
    places = placements(spec,rel['shell_low_m'])
    scene = CollisionScene(with_support(spec,places[-1]['pose'],support),cfg['resource_root'],args.output,moving_fixtures=(SUPPORT_NAME,))
    defer_pad_contacts(scene)
    geometry = MeshGeometry(scene)
    camera = CameraGate(next(c for c in spec.cameras if c.name=='camera_3'),geometry)
    arm,hand = getattr(spec,side+'_arm'),getattr(spec,side+'_hand')
    features = ArmFeatures(arm,hand)
    # Relative candidates alone enter world IK; no need to redo finger IK at 539 placements.
    palms = np.array([pose_to_matrix(p['pose'])@pose_to_matrix(e['palm_in_pad']) for p in places for e in bank])
    if not len(palms):
        write_json(args.output/'world.json',dict(side=side,placements=[],stop='no relative grip candidates'))
        return
    solutions = multistart_ik(arm,hand,palms,1021+(side=='right'))
    scene.set_pad([0,0,0,1,0,0,0],{}); scene.mj.mj_forward(scene.model,scene.data)
    pad_local_centroid = geometry.centroid('pad')
    padcentres = np.array([transform(pad_local_centroid,pose_to_matrix(p['pose'])) for p in places for _ in bank])
    metrics = [features.measure(s[0],padcentres) for s in solutions]
    records = []
    for pidx,place in enumerate(places):
        set_placement(scene,place['pose'],support)
        rejects = Counter(); accepted = []; grips30 = 0; arm_palms = 0
        for eidx,e in enumerate(bank):
            row = pidx*len(bank)+eidx; palm = palms[row]
            good_ik = [si for si,s in enumerate(solutions) if s[1][row]]
            if not good_ik:
                rejects['arm_ik'] += 1; continue
            arm_palms += 1
            # Compare all accurate starts, not merely the smallest Cartesian residual.
            good_ik.sort(key=lambda si:(float(metrics[si]['deviation'][row]),
                                       float(metrics[si]['wrist_deviation_deg'][row])))
            witnesses = []; table_pass = False; failures = Counter()
            for bi,b in enumerate(e['baselines']):
                baseline = b['resting_hand_q']
                scene.set_hand(side,palm,baseline);scene.set_pad(place['pose'],{})
                chosen = None; arm_alternatives = []
                for si in good_ik:
                    scene.set_arm(side,solutions[si][0][row])
                    clear,bad,grips,_ = scene.check(side)
                    if not clear:
                        failures[reject_reason(bad,grips)] += 1; continue
                    gap = geometry.table_clearance(side)
                    if min(gap.values()) < TABLE_GAP-1e-10:
                        failures['table_clearance_30mm'] += 1; continue
                    table_pass = True
                    gate = camera.measure((side,))
                    if not gate['passed']:
                        failures.update(gate['reasons']); continue
                    arm_alternatives.append(dict(arm_q=solutions[si][0][row].tolist(),arm_start=si,
                        arm_position_error_m=float(solutions[si][2][row]),arm_rotation_error_rad=float(solutions[si][3][row]),
                        table_clearance_m=gap,natural={k:float(v[row]) if np.asarray(v).dtype.kind!='b' else bool(v[row])
                        for k,v in metrics[si].items() if np.asarray(v).ndim==1}))
                    if chosen is None: chosen = si
                if chosen is None: continue
                si = chosen; qarm = solutions[si][0][row]
                scene.set_arm(side,qarm);scene.set_hand(side,palm,baseline);scene.set_pad(place['pose'],{})
                scene.check(side)
                gap = geometry.table_clearance(side);gate=camera.measure((side,))
                controls = []; states = []; op_rejects = Counter(b['control_rejections'])
                worst_margin = gate['margin_px']
                for control,ss in b['operations'].items():
                    goal = next(g for g in goals if g['name']==control); good = []
                    for state in ss:
                        scene.set_pad(place['pose'],state['gamepad_q']);scene.set_hand(side,palm,state['hand_q'])
                        clear,bad,grips,_ = scene.check(side,goal['part'],goal['finger'])
                        active = True  # Native evidence retained from relative bank.
                        gapstate = geometry.table_clearance(side)
                        gg = camera.measure((side,))
                        ok = clear  # Relative bank already requires both native grips and active contact.
                        reason = reject_reason(bad,grips,active)
                        if ok and min(gapstate.values()) < TABLE_GAP-1e-10:
                            ok = False; reason = 'table_clearance_30mm'
                        if ok and not gg['passed']:
                            ok = False; reason = '+'.join(gg['reasons'])
                        good.append(ok)
                        if ok:
                            states.append(dict(control=control,**state,camera=gg,table_clearance_m=gapstate))
                        else: op_rejects[control+':'+reason] += 1
                    if any(good) if goal.get('alternatives') else all(good):
                        controls.append(control)
                        worst_margin = min(worst_margin,*[s['camera']['margin_px'] for s in states if s['control']==control])
                natural = {k:float(v[row]) if np.asarray(v).dtype.kind!='b' else bool(v[row])
                           for k,v in metrics[si].items() if np.asarray(v).ndim==1}
                witnesses.append(dict(palm_index=e['palm_index'],baseline_index=b['baseline_index'],
                    palm_world=list(matrix_to_pose(palm)),palm_in_pad=e['palm_in_pad'],arm_q=qarm.tolist(),
                    arm_start=si,arm_position_error_m=float(solutions[si][2][row]),
                    arm_rotation_error_rad=float(solutions[si][3][row]),natural=natural,
                    resting_hand_q=baseline,controls=controls,states=states,
                    camera=gate,min_camera_margin_px=worst_margin,table_clearance_m=gap,
                    control_rejections=dict(op_rejects),arm_alternatives=arm_alternatives))
            grips30 += int(table_pass)
            if not witnesses:
                rejects[failures.most_common(1)[0][0] if failures else 'no_baseline'] += 1
            else:
                # Unique palm count stays the denominator; retain alternate resting witnesses for pairing.
                accepted.extend(witnesses)
        unique = {e['palm_index'] for e in accepted}
        record = dict(**place,relative_grip_palms=len(bank),arm_ik_palms=arm_palms,
            mesh_clearance_30mm_palms=grips30,camera_grip_palms=len(unique),
            per_control={g['name']:len({e['palm_index'] for e in accepted if g['name'] in e['controls']}) for g in goals},
            full_hand_palms=len({e['palm_index'] for e in accepted if len(e['controls'])==len(goals)}),
            rejected_palms=dict(rejects),witnesses=accepted)
        records.append(record)
        if pidx%49==48:
            print(json.dumps(dict(stage='height_done',side=side,height_mm=place['height_mm'],
                grips=sum(r['camera_grip_palms'] for r in records[-49:]))),flush=True)
    write_json(args.output/'world.json',dict(side=side,placements=records,elapsed_seconds=time.monotonic()-start,
        natural_rule='reviewer decision 3; 6 K0 starts retained; minimum natural deviation among accurate, collision-free starts',
        relative=rel['relative_grip_palms'],goals=goals))
    print(json.dumps(dict(stage='world_done',side=side,seconds=time.monotonic()-start)),flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['relative','world'])
    p.add_argument('--side',choices=SIDES,required=True)
    p.add_argument('--task',type=Path,default=Path('configs/tasks/fixed_dpad_causal.json'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--k0',type=Path)
    p.add_argument('--relative',type=Path)
    args = p.parse_args()
    (relative if args.stage=='relative' else world)(args)


if __name__ == '__main__': main()
